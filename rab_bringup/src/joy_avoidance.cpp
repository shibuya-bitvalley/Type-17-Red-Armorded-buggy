/* joy_control.launchからpublishされるcmd_velのデータをsubscribe、
 * base_scanもsubscribeし、進みたい方向に障害物があればcmd_velを0にしてpublish */

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class JoyAvoidance{
 public:
   JoyAvoidance(ros::NodeHandle &nh);
   
 private:
   void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
   void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& vel);
   
   // Subscrier
   ros::Subscriber sub_scan;
   ros::Subscriber sub_vel;
   // Publisher
   ros::Publisher pub_vel;
   
   //param for topic
   std::string sub_cmdvel_topic_;
   std::string scan_topic_;
   std::string pub_cmdvel_topic_;
};

JoyAvoidance::JoyAvoidance(ros::NodeHandle &nh){
   ros::NodeHandle n("~");
   n.param<std::string>("sub_cmdvel_topic", sub_cmdvel_topic_, "\"/cmd_vel_old\"");
   n.param<std::string>("scan_topic", scan_topic_, "\"/base_scan\"");
   n.param<std::string>("pub_cmdvel_topic", pub_cmdvel_topic_, "\"/cmd_vel\"");
   sub_scan = nh.subscribe(scan_topic_, 10, &JoyAvoidance::sensorCallback, this);
   sub_vel = nh.subscribe(sub_cmdvel_topic_, 1, &JoyAvoidance::cmdvelCallback, this);
}

void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
   // 位置に応じてフラグ変更(前右左くらい？)
}

void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& vel){
   // フラグに応じてcmd_velの値を調整しをpublish
}
