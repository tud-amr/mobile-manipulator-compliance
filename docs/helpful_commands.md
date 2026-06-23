## Helpful commands

### Making the robot compliant
This command does the same as the dinova bring-up from [here](https://github.com/INTERACT-tud-amr/dinova/blob/main/dinova_bringup/launch/dinova.launch). 
This makes the kinova compliant, but the base not compliant. 
```bash
roslaunch launcher dinova_bringup_compliant.launch lidar:=False
```

If you want a more simple version for kinova-arm or kinova-arm+base: without vicon, without compliant mode active at start (but it can be activated via topic or script):
```bash
roslaunch launcher robot_lfd.launch lidar:=False #base and arm
roslaunch launcher robot_lfd_kinova.launch lidar:=False #only arm
```

If you want to record a demonstration, run one of the robot_lfd(_kinova).launch files mentioned above, and the following script:
```bash
cd ros/noetic/src/lfd_interface/lfd_interface
python3 record_demo_state.py <robot_name> <demonstration_id>
```

If you just want to make the robot compliant (without doing a recording):
```bash
rostopic pub /dingo2/compliant/make_compliant True #don't forget to make it uncompliant again before doing ctrl+c)
```

You can also set the stiffness:
```bash
rostopic pub /dingo2/compliant/set_stiffness .....
```
