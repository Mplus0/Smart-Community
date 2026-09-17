# Gazebo static-contact test (ROS Noetic + Gazebo Classic 11)

This test only verifies gravity, collision geometry, inertia and wheel contact. It does not command the robot.

## Build and source

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Terminal 1: launch an empty Gazebo world

```bash
roslaunch gazebo_ros empty_world.launch
```

## Terminal 2: expand and spawn the robot above the ground

```bash
source ~/catkin_ws/devel/setup.bash
xacro $(rospack find robot_description)/urdf/robotcar.urdf.xacro > /tmp/robotcar_v3.urdf
rosparam set robot_description -t /tmp/robotcar_v3.urdf
rosrun gazebo_ros spawn_model -urdf -param robot_description -model robotcar -x 0 -y 0 -z 0.05
```

## Expected result

After falling a short distance, the robot should:

- remain upright;
- settle onto four wheel collisions;
- not explode, fly away or continuously jitter;
- keep the lower chassis above the ground;
- allow the four wheel joints to rotate freely if a joint effort/velocity is later applied.

Turn on Gazebo collision visualization if you want to inspect the simplified box and four wheel cylinders.

## This is NOT yet a mecanum-motion pass condition

Do not use lateral motion as a v0.3 acceptance test. The current wheel contact is deliberately isotropic. Proper mecanum lateral behavior is the next controller/contact-model step.
