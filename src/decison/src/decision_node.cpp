#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

enum State {
    WAIT,
    NAV_FORWARD,
    NAV_HOME,
    OUT_OF_BULLET
};

class SentinelRobot {
public:
    SentinelRobot(ros::NodeHandle& nh) : nh_(nh), state_(WAIT) {
        // 初始化ROS订阅和发布
        sub_referee_ = nh.subscribe("referee_system", 10);
        pub_nav_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10); // 发布导航目标点
        sub_tf_ = nh.subscribe("/tf", 10);
        //pub_allow_aim_ = nh_.advertise<std_msgs::Bool>("allow_aim", 1);
        //pub_allow_rotate_ = nh_.advertise<std_msgs::Bool>("allow_rotate", 1);
        //pub_allow_hit_back_ = nh_.advertise<std_msgs::Bool>("allow_hit_back", 1);
        //pub_robot_status_ = nh_.advertise<std_msgs::Int16>("robot_status", 1);
        //pub_bullet_dose_ = nh_.advertise<std_msgs::Int16>("bullet_dose", 1);

 
    void run() {
        ros::Rate rate(1); // 控制循环频率
        while (ros::ok()) {
            switch (state_) {
                case WAIT:
                    // 在等待状态下执行逻辑
                    waitForMatchStart();
                    break;

                case NAV_FORWARD:
                    // 在导航状态下执行逻辑
                    navigateToForwardPoint();
                    break;

                case NAV_HOME:
                    // 在回家状态下执行逻辑
                    navigateToHomeBase();
                    break;

                case OUT_OF_BULLET:
                    // 在没子弹状态下执行逻辑
                    handleOutOfBullet();
                    break;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_referee_;
    ros::Subscriber sub_tf_;
    ros::Subscriber sub_nav_;
    ros::Publisher pub_nav_;
    ros::Publisher pub_allow_aim_;
    ros::Publisher pub_allow_rotate_;
    ros::Publisher pub_allow_hit_back_;
    ros::Publisher pub_robot_status_;
    ros::Publisher pub_bullet_dose_;   
    ros::Subscriber result_sub_;
    bool navigation_in_progress_;
    ros::Time navigation_start_time_;
    
    State state_;

    // 其他成员变量

    
    // 处理裁判系统消息
    void refereeCallback(const referee_system::referee_system& msg) {
        bool color = msg.color;  // 敌方颜色 0红色 1蓝色
        int remaining_time = msg.remaining_time;  // 剩余时间
        int bullet_dose = msg.bullet_dose;  // 哨兵子弹
        int my_base_blood = color ? msg.red_base_blood : msg.bule_base_blood;       // 我方基地血量
        int ememy_base_blood = color ? msg.bule_base_blood : msg.red_base_blood; 

    // 解析消息并根据需要更新状态       
    if (remaining_time <= 0) {
        // 如果剩余时间小于等于0，进入STAGE_MATCH_FINISHED状态
        state_ = WAIT;
    } else if (remaining_time > 600 && bullet_dose > 200) {
        // 如果剩余时间大于600并且哨兵子弹大于200，进入STAGE_MATCH_PLAYING状态
        state_ = NAV_FORWARD;
    } else if (remaining_time < 60 || my_gaurd_blood < 110) {
        // 如果剩余时间小于1分钟或哨兵血量少于110，进入STAGE_MATCH_FINISHED状态
        state_ = NAV_HOME;
    } else if (bullet_dose < 5) {
        // 如果子弹少于5，进入STAGE_OUT_OF_BULLETS状态
        state_ = OUT_OF_BULLETS;
    } else {
        // 默认情况，进入STAGE_MATCH_START状态
        state_ = WAIT;
    }
}


    class NavigationController {
public:
    NavigationController(ros::NodeHandle nh) : nh_(nh) {
        
        // 创建 move_base 结果订阅者
        result_sub_ = nh_.subscribe("move_base/result", 1, &NavigationController::navigationResultCallback, this);

        // 初始化其他变量
        navigation_in_progress_ = false;
    }

    void waitForMatchStart() {
        if (remaining_time > 0 && my_base_blood > 0) {
            // 在剩余比赛时间大于0且基地血量大于0时等待比赛开始
            // 如果满足条件，生成导航目标点并发布
            geometry_msgs::PoseStamped target_pose = GenerateTargetPose(-3, -1.5, 0); // 生成导航目标点的函数
            pub_.publish(target_pose); // 发布导航目标点
        }
    }

    void generateNavigationGoal() {
        // 生成导航目标点
        double x = 1.0;
        double y = 2.0;
        double z = 0.0;
        geometry_msgs::PoseStamped target_pose = GenerateTargetPose(x, y, z);

        // 发布导航目标点
        pub_.publish(target_pose);

        // 标记导航状态为进行中
        navigation_in_progress_ = true;
        navigation_start_time_ = ros::Time::now();
    }

    void checkNavigationStatus() {
        if (navigation_in_progress_) {
            // 在这里处理导航状态，可以订阅 move_base 的状态话题或等待 move_base 结束
            // 如果导航超时，可以取消导航或采取其他适当的行动

        }
    }

    //void navigationResultCallback( ？) {
        //if (navigation_in_progress_) {
            //ROS_INFO("Navigation Succeeded!");
        //} else {
            //ROS_WARN("Navigation Failed!");
        //}

        // 标记导航状态为已完成
        //navigation_in_progress_ = false;

        // 在导航完成后等待一段时间，然后生成新的导航目标
        //ros::Duration wait_duration(5.0); // 等待时间（以秒为单位）
        //ros::Duration elapsed_time = ros::Time::now() - navigation_start_time_;

        //if (elapsed_time >= wait_duration) {
            // 生成新的导航目标点
            //GenerateTargetPose();

        //}
    //}


    // 在回家状态下执行逻辑
    void navigateToHomeBase() {
        // 发送导航目标点
        geometry_msgs::PoseStamped target_pose = GenerateTargetPose(-5.5, 4.5 , 0.0); 
        pub_.publish(target_pose); // 发布导航目标点
        // 监听导航状态并处理超时

    }

    // 在没子弹状态下执行逻辑
    void handleOutOfBullet() {
        // 处理没有子弹时的逻辑，例如走到角落避免被击中
        geometry_msgs::PoseStamped target_pose = GenerateTargetPose(-5.5, 4.5 , 0.0); 
        pub_.publish(target_pose); // 发布导航目标点
    }

    // 其他辅助方法
geometry_msgs::PoseStamped GenerateTargetPose(double x, double y, double z) {
    // 创建一个 PoseStamped 消息
    geometry_msgs::PoseStamped target_pose;
    
    // 填充 header
    target_pose.header.stamp = ros::Time::now();
    target_pose.header.frame_id = "map";  // 假设目标点在 "map" 坐标系中

    // 填充坐标
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;

    // 设置方向为默认值
    target_pose.pose.orientation.x = 0.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 1.0;

    return target_pose;
}
    
};
    
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "sentinel_robot");
    ros::NodeHandle nh;

    SentinelRobot robot(nh);
    robot.run();

    return 0;
}
