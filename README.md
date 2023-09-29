# checkpoint7-Manipulation-and-Perception

<p>The goal of this project is to use manipulation and perception to generate a pick-and-place task. UR5 robot is to pick an object on a table and place in a different location.</p>

<p>There are several tasks involve in this project including...<br>
<ol>
  <li>configure MoveIt1 package</li>
  <li>configure MoveIt2 package</li>
  <li>program robot sequence through MoveIt2 C++ API</li>
  <li>implement robot sequence with perception</li>
</ol>
</p>

## Plan and Execute Trajectory
<p>After configure MoveIt2, the robot simulation is able to plan and execute trajectory.</p>
    
    ros2 launch my_moveit2_config my_planning_execution.launch.py

[![moveit2](https://res.cloudinary.com/marcomontalbano/image/upload/v1695952125/video_to_markdown/images/google-drive--17BJeg6ZTq4cO7S-fNJ_wucOFRfbf2hWF-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://drive.google.com/file/d/17BJeg6ZTq4cO7S-fNJ_wucOFRfbf2hWF/view?usp=sharing "moveit2")
    
## MoveIt2 API
<p>Robot sequence is executed by using MoveIt2 API in c++.</p>

    ros2 launch moveit2_scripts pick_and_place.launch.py

[![moveit2_api](https://res.cloudinary.com/marcomontalbano/image/upload/v1695953382/video_to_markdown/images/google-drive--1mpQHmcZz1R-R_jcr0I8chm6ti2LuPC20-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://drive.google.com/file/d/1mpQHmcZz1R-R_jcr0I8chm6ti2LuPC20/view?usp=sharing "moveit2_api")

## MoveIt2 API with Perception
<p>A camera publishes point cloud topic and an action server "find_objects" shows detected object and background in Rviz2. The robot executes its trajectory while receiving a detected object's location from perception node.</p>

    ros2 launch moveit2_scripts pick_and_place_v2.launch.py

[![moveit2_pct](https://res.cloudinary.com/marcomontalbano/image/upload/v1695955289/video_to_markdown/images/google-drive--1BRZ70qehWzAUwY8TPY4oq7PY8Td_bY_3-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://drive.google.com/file/d/1BRZ70qehWzAUwY8TPY4oq7PY8Td_bY_3/view?usp=sharing "moveit2_pct")
