#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <nav_msgs/Odometry.h>

class OdomListener{
 public:
   OdomListener(ros::NodeHandle &nh);

 private:
   void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
   
   // param for topic
   std::string odom_topic_;   
   
   // subscriber
   ros::Subscriber odom_sub;
     
};

OdomListener::OdomListener(ros::NodeHandle &nh){
   // using parameter server
   ros::NodeHandle n("~");
   
   // Subscribe Topicの設定
   n.param<std::string>("odom_topic", odom_topic_, "\"/diff_drive_controller/odom\"");
   ROS_INFO("Subscribe topic : %s", odom_topic_.c_str());
   odom_sub = nh.subscribe(odom_topic_, 1000, &OdomListener::OdomCallback, this);
   
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
   ros::NodeHandle nh;
   OdomListener listener(nh);
   ros::spin();
   return 0;
}

  
   
   