# Dingo + Kinova Compliance

This repo contains a framework for performing demonstrations and playing them back on the dingo + kinova robot. 
This repo is based on the code accompanying the paper [Current-Based Impedance Control for Interacting with Mobile Manipulators](https://arxiv.org/abs/2403.13079) to achieve compliant control on mobile manipulators using off-the-shelf components.

## Summary

Guidance Mode        |  Tracking Mode
:-------------------------:|:-------------------------:
![](assets/videos/guide.gif)  |  ![](assets/videos/track.gif)

As robots transition from industrial settings to human-centered spaces, integrating mobile manipulation and compliant control becomes vital for efficient and safe task execution. Compliance is commonly achieved through indirect force control methods such as admittance and impedance control. However, these methods require contact information that are typically derived from measurements of the joint torques or from a force/torque sensor mounted at the end-effector. Such equipment can be expensive, and off-the-shelf robots might not come equipped with them. Further challenges arise based on the control mode supported by the robot. For instance, while admittance control can be applied to position/velocity-controlled robots, compliance is only achieved concerning the measured forces. On the other hand, impedance control is exclusive to torque-controlled robots, yet it effectively achieves compliance to all forces interacting with the robot. Therefore, implementing compliant control on a mobile manipulator poses significant challenges. 

We leverage the direct correlation between the actuator current and the resulting joint torque, overcoming the typical reliance of impedance control on torque sensors. Additionally, this paper presents two operational modes for interacting with the mobile manipulator: one for guiding the robot around the workspace through interacting with the arm and another for executing a tracking task, both maintaining compliance to external forces.

## Application on the Robot
Instructions are provided [here](/docs/application_on_robot.md).

If you would like to run the controller in docker: check out the original repository: [here](https://github.com/tud-amr/mobile-manipulator-compliance)

## Helpful commands
Instructions are provided [here](/docs/helpful_commands.md)

## Calibration
Select Calibrate in the visualization tool. Joints = [0, 5] specifies which joints should be calibrated for (a) lag, (b) ratio friction, (c) friction.

Kinova provides current feedback and an estimated torque feedback. Measurements show they used these current/torque ratios:


Joint: | Size: | Ratio:
-------|-------|-------
| 0 | M | 0.85
| 1 | L | 0.25
| 2 | M | 0.85
| 3 | S | 1.75
| 4 | S | 1.75
| 5 | S | 1.75


## Troubleshooting

If you run into problems of any kind, do not hesitate to open an issue on this repository.

Solutions to common issues are presented [here](docs/troubleshooting.md)


