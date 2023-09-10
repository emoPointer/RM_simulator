#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <referee_system/referee_system.h>
#include <tf/tf.h>

ros::Publisher pub;

geometry_msgs::PoseStamped PointMsg(double x, double y, double z, double roll, double pitch, double yaw)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;

    tf::Quaternion q;
    q.setRPY(roll*M_PI/180, pitch*M_PI/180, yaw*M_PI/180);
    msg.pose.orientation.x = q.getX();
    msg.pose.orientation.y = q.getY();
    msg.pose.orientation.z = q.getZ();
    msg.pose.orientation.w = q.getW();
    return msg;
}

void decision_callback(const referee_system::referee_system& msg)
{
    geometry_msgs::PoseStamped goal_msg = PointMsg(1, 2, 0, 0, 0, 60);
    pub.publish(goal_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "decision_node");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("referee_system", 10, decision_callback);
    pub=nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    ros::spin();
    return 0;
}
