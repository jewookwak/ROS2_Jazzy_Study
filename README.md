#ROS_BRIDGE STUDY  
방법 1 listener 쪽에서 토픽명 remapping<br/>
<terminal1> 
export ROS_DOMAIN_ID=1 
ros2 run demo_nodes_py listener --ros-args -r chatter:=/topic_to_domain1 

<terminal2> 
export ROS_DOMAIN_ID=2 
ros2 run demo_nodes_py listener --ros-args -r chatter:=/topic_to_domain2 

<terminal3> 
export ROS_DOMAIN_ID=3 
ros2 run domain_bridge domain_bridge ~/domain_bridge_test.yaml 

<terminal4> 
export ROS_DOMAIN_ID=3 
ros2 run my_demo_pkg talker1 

<terminal5> 
export ROS_DOMAIN_ID=3 
ros2 run domain_bridge domain_bridge ~/domain_bridge_test2.yaml 

<terminal6> 
export ROS_DOMAIN_ID=3 
ros2 run my_demo_pkg talker2 


방법 2 yaml 에서 topic명 remapping <br/>
 
<terminal1> 
export ROS_DOMAIN_ID=1 
ros2 run demo_nodes_py listener

<terminal2> 
export ROS_DOMAIN_ID=1 
ros2 run demo_nodes_py listener


<terminal3> 
export ROS_DOMAIN_ID=3 
ros2 run domain_bridge domain_bridge ~/domain_bridge_remapped_topic.yaml 

<terminal4> 
export ROS_DOMAIN_ID=3 
ros2 run my_demo_pkg talker1 

<terminal5> 
export ROS_DOMAIN_ID=3 
ros2 run my_demo_pkg talker2 
