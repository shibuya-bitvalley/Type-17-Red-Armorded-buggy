#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

class OdomListener{
 public:
   OdomListener();
   void run();
   void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
   std::string odom_topic_;
   
 private:
     
};

OdomListener::OdomListener(){
   //ros::NodeHandle n;
   ROS_INFO("コンストラクタ呼び出し");
   //ros::Subscriber sub;
   
   // Subscribe Topicの設定
   //n.param<std::string>("odom_topic", odom_topic_, "\"/diff_drive_controller/odom\"");
   //ROS_INFO("Subscribe topic : %s", odom_topic_.c_str());
   //sub = n.subscribe("/diff_drive_controller/odom", 1000, &OdomListener::OdomCallback, this);
   
   ROS_INFO("コンストラクタ終わり");
}

void OdomListener::run(){
   ros::Rate rate(1.0);
   while(ros::ok()){
      ROS_WARN("running");
      ros::spinOnce();
      rate.sleep();
   }   
}

void OdomListener::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
   double orientation_z = 0;
   double orientation_w = 0;
   double th0 = 0;
   th0 = 2 * asin(msg->pose.pose.orientation.z / 2);
   double th1 = 0;
   th1 = 2 * acos(msg->pose.pose.orientation.w / 2);
   ROS_INFO("Seq: [%d]", msg->header.seq);
   ROS_INFO("Position-> x: [%lf], y: [%lf]", msg->pose.pose.position.x, msg->pose.pose.position.y);
}

int main(int argc, char **argv){
   ros::init(argc, argv, "odom_listener");
   OdomListener listener;
   ros::NodeHandle n;
   //ros::Subscriber sub = n.subscribe("/diff_drive_controller/odom", 1000, &OdomListener::OdomCallback, &odom_listener);
   listener.run();
   return 0;
}

  
   
   