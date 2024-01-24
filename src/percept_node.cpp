#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_msgs/ModelStates.h>
#include <world_percept_assig3/UpdateObjectList.h>
#include <world_percept_assig3/SetInitTiagoPose.h>

class WorldInfo
{
private:

    std::string subs_topic_name_;        ///< gazebo model state topic name
    std::string srv_update_obj_name_;        ///< name of the service provided by the map generator node
    ros::Subscriber sub_gazebo_data_;     ///< Subscriber gazebo model_states

    std::vector<std::string> v_seen_obj_;  ///< List of objects seen by the robot and sent to the map generator node

    ros::ServiceClient client_map_generator_; ///< Client to request the object list update in the map generator node

public:

    WorldInfo(ros::NodeHandle& nh)
    {
        ROS_WARN_STREAM("Created world info");

        // This objects will not be sent to the Map generator node
        v_seen_obj_.push_back("tiago");
        v_seen_obj_.push_back("ground_plane");

        subs_topic_name_="/gazebo/model_states";

        // create client and wait until service is advertised
        srv_update_obj_name_="update_object_list";

        client_map_generator_ = nh.serviceClient<world_percept_assig3::UpdateObjectList>(srv_update_obj_name_);

        // Wait for the service to be advertised
        ROS_INFO("Waiting for service %s to be advertised...", srv_update_obj_name_.c_str());
        bool service_found = ros::service::waitForService(srv_update_obj_name_, ros::Duration(30.0)); // You can adjust the timeout as needed

        if(!service_found)
        {
            ROS_ERROR("Failed to call service %s", srv_update_obj_name_.c_str());
            exit;
        }

        ROS_INFO_STREAM("Connected to service: "<<srv_update_obj_name_);

        // Create subscriber to receive the commanded turtle state. This state will be generated from a trajectory generator
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 100, &WorldInfo::topic_callback, this);
    };

    ~WorldInfo()
    {

    };

private:

/**
   * @brief Callback function to receive the Gazebo Model State topic
   *
   * @param msg message with the current Gazebo model state
   */
  void topic_callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {

    //Get robot Pose
    geometry_msgs::Pose tiago_pose;
    // Search for tiago pose
   auto it = std::find( msg->name.begin(),  msg->name.end(), "tiago");
    if (it != msg->name.end()) 
    {
        // Calculate the index
        int index = std::distance(msg->name.begin(), it);
        tiago_pose=msg->pose.at(index);
    }

    // search new objects in the scene 
    for (int i = 0; i < msg->name.size(); i++)
    {
        // Get obj position
        // Get tiago position
        // Compare distances, if within range then check the v_seen_list
        // if the obj is not in the list add it and send it to the srv

        // Get object pose
        //Identify the right variable that contains the correct object pose 
        geometry_msgs::Pose obj_pose= geometry_msgs::Pose();

        // get distance from tiago to obj[i]
        //obtain the dx distance between the robot and the objects 
        double dx= 0;
        //obtain the dy distance between the robot and the objects 
        double dy=0;
        // compute the distance between the tiago and the objects 
        double d=0;

        //IF the robot is closer to the seen objects, then request the service
        if (d<1.1)
        {
            //Identify the object seen 
            std::string s= "something";
            // Search for the obj name in the seen_list
            auto it = std::find(v_seen_obj_.begin(), v_seen_obj_.end(), s);

            // If the obj name is not found in the seen vector, this means that the robot has seen a new object for the first time and it should add it to the seen vector and call the service update_object_list
            if (it == v_seen_obj_.end()) {

                world_percept_assig3::UpdateObjectList srv;

                // send the new seen object to the service 
                srv.request.object_name="object";
                // send the pose of the seen object to the service 
                srv.request.object_pose= geometry_msgs::Pose();

                if (client_map_generator_.call(srv))
                {
                    ROS_INFO_STREAM("Object List Updated?: "<< (int)srv.response.confirmation);

                    if(srv.response.confirmation)
                    {
                        v_seen_obj_.push_back(s);

                        ROS_INFO_STREAM("Object ["<<s<<"] added to the list");
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("Failed to call service "<<srv_update_obj_name_);
                }
            }
        } //if d
    }//for msg size

    //If you want to print the objects that the robot has seen so far, just uncomment the for 
    //for (size_t i = 2; i < v_seen_obj_.size(); i++)
    // {
    //     ROS_INFO_STREAM("["<<i<<"]: "<<v_seen_obj_.at(i));
    // }
    
  } // callback

    

}; // Class 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "percept_node");
    ros::NodeHandle nh;

    WorldInfo myPercept(nh);

    ros::spin();

    return 0;
}
