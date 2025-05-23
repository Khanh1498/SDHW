# Trajectory-Tracking-Control-Homework-1 

## Objective:
This assignment will contribute to your term project and will comprise of two parts: 
1. **First Part:** 

You will implement a Pure Pursuit Steering Controller (Lateral Control) that allows the autonomous car to follow a given path by steering appropriately using the provided waypoints.
The first part of the assignment will be done for steering control alone (with a selected constant velocity)

2. **Second Part:** 

You will involve the integration of the previous PID based velocity control with steering control. Both controllers will be simulated and tested in the Gazebo environment, first separately, and then, in integration. You are free to implement both controllers in the same file or split them into separate
modules as you see fit.

## Input: Waypoint File (.txt)
The car should follow a path defined by a txt file. Each line includes (x, y, z) coordinates, however, z can be ignored for 2D control.
Example txt content (x,y,z):
277.9850064328933,-135.28087100778282,2.8375740978607227
278.0146965437528,-135.30389662347997,2.8368270682932866
278.0456500635735,-135.3279020526146,2.836048250233644

## Notes
### Odometry Subscription
- Subscribe to /odom to get the car’s current position and orientation.

### Waypoint Reader Logic (This can be done using any logic)
- You can create a publisher that reads the txt file and publishes waypoints over a topic (e.g., /waypoints).
- This node should find the nearest point respect to car in txt file help with /odom topic. 
- Help with point that found the point that is at the ```lookahead distance``` from the vehicle should be published. This published point will be track point for tracjectory tracking node.

### Pure Pursuit Control Logic (For the first part of the assignment)
- For each step, read the waypoint from ```/waypoints``` topic. From the list in relation to the selected lookahead distance (Note: While a lookahead distance based on the vehicle velocity is
preferable, for the first part of the assignment take it as a constant value, and discuss how this affects the performance).
- Calculate the angle required to reach the lookahead point.
- Calculate the steering angle.
- Output steering commands.

### Integration of Lateral and Longitudinal Controllers (2 nd part of assignment)
- Combine the Pure Pursuit based steering control with the speed from the PID controller.
- Publish combined motion commands to /cmd_vel.

### Tips for Implementation:
- Reference implementation idea: https://thomasfermi.github.io/Algorithms-for-
Automated-Driving/Control/PurePursuit.html

### Output:
- The car should steer through the waypoints smoothly.
- A visualization of the selected lookahead point (via RViz Marker) is encouraged.(This is not must)