import os
import rospy
from threading import Thread
import time
import sys

from user_interface_msg.msg import Ufdbk, Ustate, Ucmd, Utarget
from geometry_msgs.msg import PoseStamped

from compliant_control.interface.user_interface import UserInterface
from compliant_control.mujoco.visualization import Visualization

from compliant_control.utilities.rate_counter import RateCounter


class UserInterfaceNode:
    """A node that starts the user interface."""

    def __init__(self, args) -> None:
        rospy.init_node("user_interface_node")
        self.pub_command = rospy.Publisher("/command", Ucmd, queue_size=10)
        self.pub_target = rospy.Publisher("/target", Utarget, queue_size=10)

        self.interface = UserInterface(self.command)
        self.interface.create_ui()

        self.visualize = "--visualize" in args

        if self.visualize:
            self.visualization = Visualization()
            visualize_thread = Thread(target=self.visualization.start)
            visualize_thread.start()
            target_thread = Thread(target=self.publish_target_loop)
            target_thread.start()

        rospy.Subscriber("/feedback", Ufdbk, self.feedback, queue_size=10)
        rospy.Subscriber("/vicon", PoseStamped, self.vicon_feedback, queue_size=10)
        rospy.Subscriber("/state", Ustate, self.state, queue_size=10)
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()

        self.interface.start()

    def command(self, command: str, args: list = None) -> None:
        """Send a command to the control interface."""
        self.interface.update_state("waiting")
        if self.visualize:
            if command == "Automove":
                self.visualization.target_mover.toggle()
            if command == "Reset":
                self.visualization.reset_target()
                self.interface.reset_wheels()
            if command in ["Start LLC Task", "arm", "Start HLT"]:
                self.visualization.reset_target()
                self.publish_target()
                time.sleep(0.1)
        msg = Ucmd()
        msg.command = command
        if args is not None:
            msg.args = args
        self.pub_command.publish(msg)

    def feedback(self, msg: Ufdbk) -> None:
        """Process the feedback."""
        self.interface.rates.con = msg.controller_rate
        if len(msg.kinova_pos) > 0:
            self.kinova_feedback(msg)
        if len(msg.dingo_pos) > 0:
            self.dingo_feedback(msg)

    def state(self, msg: Ustate) -> None:
        """Update the state."""
        self.interface.state.mode = msg.servoing_mode
        self.interface.state.comp_grav = msg.comp_grav
        self.interface.state.comp_fric = msg.comp_fric
        self.interface.state.imp_arm = msg.imp_arm
        self.interface.state.imp_null = msg.imp_null
        self.interface.state.imp_base = msg.imp_base
        self.interface.state.automove_target = msg.automove_target
        for n, joint in enumerate(self.interface.joints):
            joint.active = msg.active[n]
            joint.mode = msg.mode[n]
            joint.ratio = msg.ratio[n]
            joint.friction = msg.friction[n]
        self.interface.update_state()

    def kinova_feedback(self, msg: Ufdbk) -> None:
        """Process the Kinova feedback."""
        for joint in self.interface.joints:
            joint.pos = msg.kinova_pos[joint.index]
            joint.vel = msg.kinova_vel[joint.index]
            joint.eff = msg.kinova_tor[joint.index]
        self.interface.rates.kin = msg.kinova_rate
        self.interface.update_bars("Kinova")
        if self.visualize:
            self.visualization.set_qpos_value("Kinova", "position", msg.kinova_pos)

    def dingo_feedback(self, msg: Ufdbk) -> None:
        """Process the Dingo feedback."""
        for wheel in self.interface.wheels:
            last_position = wheel.pos
            wheel.encoder_position = msg.dingo_pos[wheel.index]
            wheel.vel = wheel.pos - last_position
            wheel.eff = msg.dingo_tor[wheel.index]
        self.interface.rates.din = msg.dingo_rate
        self.interface.update_bars("Dingo")
        if self.visualize:
            self.visualization.set_qpos_value("Dingo", "position", msg.dingo_pos)

    def vicon_feedback(self, msg: PoseStamped) -> None:
        """Process the Vicon feedback."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        quat_w = msg.pose.orientation.w
        quat_z = msg.pose.orientation.z
        if self.visualize:
            self.visualization.set_world_pos_value(x, y, quat_w, quat_z)

    def publish_target_loop(self) -> None:
        """A loop that publishes the target."""
        rate_counter = RateCounter(30)
        while True:
            self.publish_target()
            rate_counter.count()
            rate_counter.sleep()

    def publish_target(self) -> None:
        """Publish the target."""
        msg = Utarget()
        msg.relative_target = list(self.visualization.relative_target)
        msg.absolute_target = list(self.visualization.target)
        msg.pos_b = list(self.visualization.origin_arm)
        msg.quat_b = list(self.visualization.orientation_arm)
        self.pub_target.publish(msg)

    def start_spin_loop(self) -> None:
        """Start node spinning."""
        rospy.spin()


def main(args: any = None) -> None:
    """Main."""
    args = rospy.myargv(argv=sys.argv)
    UserInterfaceNode(args)
    os._exit(0)


if __name__ == "__main__":
    main()
