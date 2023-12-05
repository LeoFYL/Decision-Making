#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <world_percept_assig3/GetSceneObjectList.h>
#include <world_percept_assig3/GotoObject.h>

class TiagoControlNode
{
private: 
    std::string subs_topic_name_;
    ros::Subscriber sub_gazebo_data_;
    ros::Publisher pub_key_vel_; 
    ros::Timer tf_timer_;
    ros::ServiceServer goto_obj_srv_; 
    std::string target_object_name_; 
    bool target_acquired_ = false; 
    geometry_msgs::Pose target_pose_;
    ros::ServiceClient get_scene_object_client_;
    geometry_msgs::Pose tiago_pose_;

public:
    TiagoControlNode(ros::NodeHandle& nh)
    {
        subs_topic_name_ = "/gazebo/model_states";
        sub_gazebo_data_ = nh.subscribe(subs_topic_name_, 10, &TiagoControlNode::modelStatesCallback, this);
        pub_key_vel_ = nh.advertise<geometry_msgs::Twist>("/key_vel", 10);
        
        std::string srv_get_scene_name_ = "/get_scene_object_list";
        get_scene_object_client_ = nh.serviceClient<world_percept_assig3::GetSceneObjectList>(srv_get_scene_name_);

        goto_obj_srv_ = nh.advertiseService("/goto_object", &TiagoControlNode::gotoObjectCallback, this);
        tf_timer_ = nh.createTimer(ros::Duration(0.1), &TiagoControlNode::tfTimerCallback, this);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
    {
        auto tiago_it = std::find(msg->name.begin(), msg->name.end(), "tiago");
        if (tiago_it != msg->name.end())
        {
            int tiago_index = std::distance(msg->name.begin(), tiago_it);
            tiago_pose_ = msg->pose[tiago_index];
        }
        else
        {
            ROS_ERROR("Tiago not found in the model states.");
        }
    }

    bool gotoObjectCallback(world_percept_assig3::GotoObject::Request& req, world_percept_assig3::GotoObject::Response& res)
    {
    target_object_name_ = req.obj;
    target_acquired_ = true;
    res.confirmation = true;  // 假设响应类型有一个名为 'confirmation' 的布尔成员
    
    //res.message = "Target object acquired.";  // 假设响应类型有一个名为 'message' 的字符串成员
    return true;
    }

    void tfTimerCallback(const ros::TimerEvent& event)
    {
        if (!target_acquired_)
            return;

        world_percept_assig3::GetSceneObjectList srv;
        srv.request.object_name = target_object_name_;
        if (get_scene_object_client_.call(srv) && srv.response.obj_found)
        {
            auto target_it = std::find(srv.response.objects.name.begin(), srv.response.objects.name.end(), target_object_name_);
            if (target_it != srv.response.objects.name.end())
            {
                int target_index = std::distance(srv.response.objects.name.begin(), target_it);
                target_pose_ = srv.response.objects.pose[target_index];
                moveTowardsTarget();
            }
            else
            {
                ROS_ERROR("Target object not found in the scene.");
                target_acquired_ = false;
            }
        }
        else
        {
            ROS_ERROR("Failed to call service get_scene_object_list.");
            target_acquired_ = false;
        }
    }

    void moveTowardsTarget()
    {
        Eigen::Vector2d tiago_position(tiago_pose_.position.x, tiago_pose_.position.y);
        Eigen::Vector2d target_position(target_pose_.position.x, target_pose_.position.y);
        Eigen::Vector2d Dpose_w = target_position - tiago_position;

        Eigen::Matrix2d Rtiago_w = q2Rot2D(tiago_pose_.orientation);
        Eigen::Vector2d Dpose_tiago = Rtiago_w.inverse() * Dpose_w;

        double theta = std::atan2(Dpose_tiago(1), Dpose_tiago(0));

        double Kvw = 1.1, Kvx = 0.1;
        geometry_msgs::Twist tiago_twist_cmd;
        tiago_twist_cmd.angular.z = Kvw * theta;
        tiago_twist_cmd.linear.x = Kvx * Dpose_tiago.norm() < 1.3 ? 0.0 : Dpose_tiago.norm();

        pub_key_vel_.publish(tiago_twist_cmd);
    }

    Eigen::Matrix2d q2Rot2D(const geometry_msgs::Quaternion& quaternion)
    {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quaternion, tf_quat);
        tf2::Matrix3x3 m(tf_quat);
        Eigen::Matrix2d rotation_matrix;
        rotation_matrix << m[0][0], m[0][1],
                           m[1][0], m[1][1];
        return rotation_matrix;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tiago_control_node");
    ros::NodeHandle nh;

    TiagoControlNode tiago_control(nh);

    ros::spin();
    return 0;
}