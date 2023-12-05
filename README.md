# world_percept_assig3





### Task 1
 
source /knowrob_ws/devel/setup.bash
catkin_make

source /home/user/exchange/ssy236_liufe/devel/setup.bash
roslaunch world_percept_assig3 gazebo_ssy236.launch


rosrun world_percept_assig3 map_generator_node


rosservice call /get_scene_object_list "{object_name: "all"}"
rosservice call /get_scene_object_list "{object_name: "beer"}"


### Task 2




source /home/user/exchange/ssy236_liufe/devel/setup.bash
roslaunch world_percept_assig3 gazebo_ssy236.launch


roslaunch world_percept_assig3 reasoning.launch


rosparam load ./src/world_percept_assig3/config/loadKnowledge.yaml
rosrun world_percept_assig3 reasoning_node ./src/world_percept_assig3/config


rosservice call /assert_knowledge "{object_pose: {position: {x: 1, y: 1, z: 1}, orientation:{x: 0, y: 0, z: 0,w: 1}}, object_name: "Test"}"


rosrun world_percept_assig3 percept_node




### Task3


source /home/user/exchange/ssy236_liufe/devel/setup.bash
roslaunch world_percept_assig3 gazebo_ssy236.launch


roslaunch world_percept_assig3 reasoning.launch


rosparam load ./src/world_percept_assig3/config/loadKnowledge.yaml
rosrun world_percept_assig3 knowledge_node ./src/world_percept_assig3/config







### Task4


source /home/user/exchange/ssy236_liufe/devel/setup.bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/user/exchange/ssy236_andperei/src/world_percept_assig3
roslaunch world_percept_assig3 gazebo_ssy236.launch


rosrun world_percept_assig3 map_generator_node 


rosrun world_percept_assig3 percept_node 


rosrun world_percept_assig3 tiago_control_node 


rosservice call /goto_object "{obj: "bowl"}"


