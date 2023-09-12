#include <ros/ros.h>
#include <referee_system/referee_system.h>



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "manual_input");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<referee_system::referee_system>("manual_input", 10);

    referee_system::referee_system msg;

    ros::Rate rate(1);
    while (true)
    {
        msg.remaining_time=-1;  //倒计时为-1说明为自动发送的消息，可直接忽略
        msg.color=false;
        msg.bule_base_blood=1500;
        msg.red_base_blood=1500;
        msg.r1_blood=0;
        msg.b1_blood=1;
        msg.r2_blood=2;
        msg.b2_blood=3;
        msg.r3_blood=4;
        msg.b3_blood=5;
        msg.r4_blood=200;
        msg.b4_blood=200;
        msg.r5_blood=200;
        msg.b5_blood=200;
        msg.r6_blood=0;
        msg.b6_blood=1;
        msg.r7_blood=600;
        msg.b7_blood=600;
        msg.r_outpost=0;
        msg.b_outpost=4;
        msg.r_darts=true;
        msg.b_darts=true;
        msg.energy=false;
        msg.bullet_dose=50;

        pub.publish(msg);
        
        rate.sleep();
    }

    return 0;
}
