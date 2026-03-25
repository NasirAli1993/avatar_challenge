xArm7 Shape Drawing Node
========================

**Framework:** ROS2 Humble + MoveIt2
Overview

--------

This node controls the xArm7 robot to draw 2D geometric shapes) in 3D Cartesian space using MoveIt2 path planning. Shapes are loaded from a JSON configuration file.

* * *

Input Shape Definition
-------------------------

All shapes are defined in **`config/shapes.json`**.

### Supported Shape Types

1. **`polygon`**
   Defined by a list of 2D vertices.

2. **`arc`**
   Defined by center, radius, start angle, end angle, segments.

3. **`bspline`**
   Defined by control points, degree, and sample count.

### Common Fields for All Shapes

json
    "start_pose": {
      "position": [x, y, z],
      "orientation_rpy": [roll, pitch, yaw]
    }

* `position`: 3D starting point of the shape
* `orientation_rpy`: 3D orientation of the drawing plane

* * *

How to Add New Shapes
------------------------

1. Open `config/shapes.json`
2. Add a new object inside the `"shapes"` array
3. Choose type: `polygon`, `arc`, or `bspline`
4. Fill required parameters
5. Save and restart the node

Example (new square):

json
    {
      "type": "polygon",
      "vertices": [[0,0], [0,0.1], [0.1,0.1], [0.1,0]],
      "start_pose": {
        "position": [0.3, 0.2, 0.5],
        "orientation_rpy": [0, 0, 0]
      }
    }

* * *

Approach & Methodology
-------------------------

### How I Thought About the Problem

The goal is to **draw smooth 2D shapes on a 3D plane** using a robotic arm.

The robot cannot directly draw in 2D, so I must:

1. Define shapes in a local 2D plane
2. Transform the 2D points into 3D world space
3. Plan a safe Cartesian path
4. Execute motion via MoveIt2

### Key Decisions

1. **Use MoveIt2 Cartesian Path**
   Ensures straight-line motion in 3D space, which is required for drawing.

2. **Full 3D Orientation Support**
   Use quaternion rotation (`tf2::quatRotate`) to support arbitrarily tilted drawing planes.
   This allows drawing on any surface, not just the horizontal plane.

3. **JSON Input**
   Centralized shape configuration for easy editing without code changes.

4. **Visualization Markers**
   Publish trajectories to RViz for debugging and verification.

### Why These Decisions?

* **Cartesian path = stable drawing**
* **Full 3D rotation = maximum flexibility**
* **JSON = user-friendly shape design**
* **Markers = visible feedback**

### Assumptions

1. The robot is already powered on and move_group is running.
2. The starting pose is within the robot’s workspace.
3. The drawing plane orientation does not cause self-collision.
4. Shape size is small enough for the xArm7 to reach.

* * *

Files Included
-----------------

* `draw_shapes.cpp` – Main control node
* `config/shapes.json` – Shape definitions
* `package.xml` – ROS2 dependencies
* `CMakeLists.txt` – Build configuration

* * *

How to Run
-------------

The official test launch file is used to start the robot, MoveIt2, and shape drawing node:
   `ros2 launch avatar_challenge start.launch.py`

Once RViz and move_group are ready, run the shape drawing node:
    `ros2 run avatar_challenge draw_shapes`



![97eaf17f-f3b6-4fad-a6ce-6c3ec5070d48](file:///C:/Users/jli15/Pictures/Typedown/97eaf17f-f3b6-4fad-a6ce-6c3ec5070d48.png)

![91422e46-2db3-4267-a111-5714dc2d1a51](file:///C:/Users/jli15/Pictures/Typedown/91422e46-2db3-4267-a111-5714dc2d1a51.png)
