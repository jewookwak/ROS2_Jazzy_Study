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
