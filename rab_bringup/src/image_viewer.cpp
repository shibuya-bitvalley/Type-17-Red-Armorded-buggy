#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

class odom{
 public:   
   int flag;   //画像を表示するためのフラグ
 private:
   void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg);
};
   
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
   double orientation_z = 0;
   double orientation_w = 0;
   double th0 = 0;
   th0 = 2 * asin(msg->pose.pose.orientation.z / 2);
   double th1 = 0;
   th1 = 2 * acos(msg->pose.pose.orientation.w / 2);
   ROS_INFO("Seq: [%d]", msg->header.seq);
   ROS_INFO("Position-> x: [%lf], y: [%lf]", msg->pose.pose.position.x, msg->pose.pose.position.y);
   ROS_INFO("Orientation-> th0: [%lf], th1: [%lf]", th0 * 180 / 3.14, th1 * 180 / 3.14);
}

//画像表示を行うためのフラグをいじる関数
int changeflag(const nav_msgs::Odometry::ConstPtr& msg){
   double orientation_z = 0;
   double orientation_w = 0;
   double th0 = 0;
   th0 = 2 * asin(msg->pose.pose.orientation.z / 2);
   double th1 = 0;
   th1 = 2 * acos(msg->pose.pose.orientation.w / 2);
}

int main(int argc, char **argv){
   ros::init(argc, argv, "image_viewer");
   ros::NodeHandle n;
   ros::Subscriber sub;
   sub = n.subscribe("/diff_drive_controller/odom", 1000, chatterCallback);
   ros::spin();
}