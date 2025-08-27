<img width="1167" height="646" alt="image" src="https://github.com/user-attachments/assets/0a65bff3-1cce-4e8c-93f3-8400f7bd0679" />**ROS2 Study using mobile robot**
1. PID Controller for waypoint following(controller_node)
- P control
- PID control
2. Aruco Marker dectection code()
- Aruco Marker Localization

- Affine Transformation
3. MAPF(Multi Agent Path Finding)
- A*
- CBS Planner
4. Domain Bridge
- Talker Listener example
- Server PC <-> AMR
5. Local Planner(Collision Avoidance Control)
- DWB control
- RL control (DQN)

#ROS_BRIDGE STUDY  
방법 1 listener 쪽에서 토픽명 remapping<br/>
<terminal1><br/>
export ROS_DOMAIN_ID=1<br/>
ros2 run demo_nodes_py listener --ros-args -r chatter:=/topic_to_domain1<br/>
<br/>
<terminal2><br/>
export ROS_DOMAIN_ID=2<br/>
ros2 run demo_nodes_py listener --ros-args -r chatter:=/topic_to_domain2<br/>
<br/>
<terminal3><br/>
export ROS_DOMAIN_ID=3<br/>
ros2 run domain_bridge domain_bridge ~/domain_bridge_test.yaml<br/>
<br/>
<terminal4><br/>
export ROS_DOMAIN_ID=3<br/>
ros2 run my_demo_pkg talker1<br/>
<br/>
<terminal5><br/>
export ROS_DOMAIN_ID=3<br/>
ros2 run domain_bridge domain_bridge ~/domain_bridge_test2.yaml<br/>
<br/>
<terminal6><br/>
export ROS_DOMAIN_ID=3<br/>
ros2 run my_demo_pkg talker2<br/>
<br/>

방법 2 yaml 에서 topic명 remapping<br/>
<br/>
<terminal1><br/>
export ROS_DOMAIN_ID=1<br/>
ros2 run demo_nodes_py listener<br/>
<br/>
<terminal2><br/>
export ROS_DOMAIN_ID=2<br/>
ros2 run demo_nodes_py listener<br/>
<br/>

<terminal3><br/>
export ROS_DOMAIN_ID=3<br/>
ros2 run domain_bridge domain_bridge ~/domain_bridge_remapped_topic.yaml<br/>
<br/>
<terminal4><br/>
export ROS_DOMAIN_ID=3<br/>
ros2 run my_demo_pkg talker1<br/>
<br/>
<terminal5><br/>
export ROS_DOMAIN_ID=3<br/>
ros2 run my_demo_pkg talker2<br/>
<br/>

6. GUI(PyQT)
- Monitoring the map with location, pose and path of agents
