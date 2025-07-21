# Welcome to Simple Path Planning for ROS! 👋

This little ROS package is all about making it super easy to create a straight path from point A to point B in your robot's world. If you need a simple, no-fuss way to generate a path between two frames, keep reading.

## What is here?

At the heart of this package is the `path_planner_node`. This node is interacted through its server `/set_plan`. It continuously calculates a path and broadcasts it on the `/planned_path` topic for other nodes to use.

## How to get started 🚀

Getting up and running is a piece of cake!


0.  **Building the code 🛠️**
  
    At startup, or if you've made any changes, you'll need to build the package. Just run this command from your catkin workspace:
    
    ```bash
    catkin build simple_path_planning
    ```

1.  **Launch everything:**
    ```bash
    roslaunch simple_path_planning path_planner.launch
    ```

3.  **Tell it to plan a path:**

       Call the `/set_plan` service to get things moving. Here's the example usage:
    
        ```bash
        rosservice call /set_plan "{
          target_frame: 'your_frame',
          loop_rate: 1.0,  # How often to recompute the path (Set to very low if you want a static path)
          angle_offset: 0.0, # Additional angle offset (in radians) to add to the target orientation
          num_waypoints: 100, # Number of waypoints to generate
          interpolate_xy: true, # If True, interpolate the x and y positions
          interpolate_z: true, # If True, interpolate the z position
          interpolate_yaw: true # If True, interpolate the yaw orientation
        }"
        ```

4.  **See the magic in RViz:**

    Open up RViz, add a `Path` display, and set the topic to `/planned_path`. You should see your path pop up in all its glory!

5.  **To stop planning:**

    When you're done, just tell the node to stop:

    ```bash
    rosservice call /stop_planning "{}"
    ```


