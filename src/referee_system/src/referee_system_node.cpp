#include <ros/ros.h>
#include <referee_system/referee_system.h>

ros::Publisher pub;
int coutdown_time = 0;
referee_system::referee_system last_state;

void InputCallback(referee_system::referee_system msg)
{
    //新消息时间为-1（自动发布） countdown没有设置直接return
    if (msg.remaining_time == -1 && coutdown_time == 0)
        return;
    //新消息时间为-1（自动发布） countdown开始倒计时并发布话题
    else if (msg.remaining_time == -1 && coutdown_time > 0)
    {
        coutdown_time--;
        last_state.remaining_time = coutdown_time;
        pub.publish(last_state);
    }
    //新消息时间不为-1（手动发布）更新状态
    else
    {
        last_state = msg;
        coutdown_time = msg.remaining_time;
        pub.publish(last_state);
    }
}

int main(int argc, char *argv[])
{


    ros::init(argc, argv, "referee_system");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("manual_input", 10, InputCallback);
    pub = nh.advertise<referee_system::referee_system>("referee_system", 10);
    ros::spin();
    return 0;
}
