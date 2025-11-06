# Turtlesim ROS 2 Tutorial

A hands-on guide to learning ROS 2 concepts using the Turtlesim simulator.

---

## 1. Launch Turtlesim

Run the Turtlesim node:

```bash
ros2 run turtlesim turtlesim_node
```

**Expected Result:** You should see a blue window with a small turtle in the middle.

---

## 2. Open a Second Terminal

Source ROS again in a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

Then list active nodes:

```bash
ros2 node list
```

**Expected Output:**

```
/turtlesim
```

---

## 3. Control the Turtle with Teleop

Run the teleop node in another terminal:

```bash
ros2 run turtlesim turtle_teleop_key
```

**Instructions:** Use your arrow keys to move the turtle around.

---

## 4. Check Active Topics

You can see what topics are currently active:

```bash
ros2 topic list
```

**Example Output:**

```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

---

## 📡 5. Inspect Topic Data

View real-time velocity commands being published:

```bash
ros2 topic echo /turtle1/cmd_vel
```

**Example Output:**

```yaml
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.8
```

---

## 6. Publish a Command Manually

You can publish velocity messages directly to move the turtle:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

---

## 7. Spawn a Second Turtle

You can call a service to spawn a new turtle:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
```

List turtles:

```bash
ros2 service call /turtlesim/list turtlesim/srv/ListTurtles "{}"
```

---

## 8. Move the Second Turtle

You can control the new turtle by publishing to its topic:

```bash
ros2 topic pub /turtle2/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

---

## 9. Reset or Clear the Screen

Reset the simulation:

```bash
ros2 service call /reset std_srvs/srv/Empty "{}"
```

Clear the background:

```bash
ros2 service call /clear std_srvs/srv/Empty "{}"
```

---



**Next Steps:** Try creating your own ROS 2 nodes to control the turtles programmatically
