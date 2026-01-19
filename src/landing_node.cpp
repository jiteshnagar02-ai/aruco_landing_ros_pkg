#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/SetMode.h>
#include <tf/transform_datatypes.h>



class LandingNode
{
public:
    LandingNode(ros::NodeHandle &nh) : loop_rate(50)
    {
        vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 4);
        pose_sub = nh.subscribe("/landpose", 4, &LandingNode::poseCallback, this);
        modeChange = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    }

    void poseCallback(const geometry_msgs::Pose &pose)
    {
        double yaw_error = tf::getYaw(pose.orientation);

        if(yaw_error<0.1) {
        cam_pose.twist.linear.x = -0.2 * pose.position.y;
        cam_pose.twist.linear.y = -0.2 * pose.position.x;
        }
        
        cam_pose.twist.angular.z = -0.5 * yaw_error;

        if (abs(pose.position.x) < 0.09 && abs(pose.position.y) < 0.09)
        {
            cam_pose.twist.linear.z = -0.09 * pose.position.z;
        }

        if (abs(pose.position.z) < 0.4)
        {
            publish = false ;
            mavros_msgs::SetMode mode;
            mode.request.base_mode = 0 ;
            mode.request.custom_mode = "AUTO.LAND" ;

            if (modeChange.call(mode) && mode.response.mode_sent)
            {
                ROS_INFO("Sum: %ld", (long int)mode.response.mode_sent); // Access the response fields
            }
            else
            {
                ROS_ERROR("Failed to call service base_mode");
            }
        }
    }

    void publisher()
    {
        while (ros::ok())
        {
            if(publish) {
            vel_pub.publish(cam_pose);
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::Publisher vel_pub;
    ros::Subscriber pose_sub;
    geometry_msgs::TwistStamped cam_pose;
    ros::Rate loop_rate;
    ros::ServiceClient modeChange;
    bool publish = true ;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "landing_node");
    ros::NodeHandle nh;
    LandingNode land(nh);
    land.publisher();
    return 0;
}