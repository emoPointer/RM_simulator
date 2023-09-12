#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <referee_system/referee_system.h>
#include <tf/tf.h>

ros::Publisher pub;
geometry_msgs::PoseStamped PointMsg(double x, double y, double z)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;

    tf::Quaternion q;
    q.setRPY(0,0,0);
    msg.pose.orientation.x = q.getX();
    msg.pose.orientation.y = q.getY();
    msg.pose.orientation.z = q.getZ();
    msg.pose.orientation.w = q.getW();
    return msg;
}

void decision_callback(const referee_system::referee_system& msg)
{ 
    if(msg.color==1)//我方是红色，敌方是蓝色。
    //正常哨兵守边，如果基地血量下降， 回退到基地路旁，守1边，另一边让操作手进入攻击可能躲到后面的敌人
    {
        pub.publish(PointMsg(1.9, 1.64, 0.004));
    
        if(msg.red_base_blood<1500) //受到攻击立刻回防
        {
            pub.publish(PointMsg(2.38,1.85,0.0034));
     }
        else if((msg.r1_blood<=100 || msg.r3_blood<=100 ) && msg.b1_blood>=100 && msg.b3_blood>=100 && msg.r7_blood>=300)  //大优势前压攻击
         {
           pub.publish(PointMsg(-2.7,2.2,0.002));

         }
          else if(msg.r7_blood>=500 && msg.r3_blood>=149 && msg.b3_blood<=140)  // 0.001 0.08 0.01  //  小优势可适当前压

           {
           pub.publish(PointMsg(0.001,0.08,0.01));
         }
         else if(msg.r1_blood<=100 && msg.r7_blood <=400 && msg.b1_blood>=100 )
         {
            
            pub.publish(PointMsg(3.01,-0.98,0.001));
         }
  }
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
