#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

/*
class odom{
 public:   
   int flag;   //画像を表示するためのフラグ
 private:
   void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg);
};
*/
   
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

/*
//画像表示を行うためのフラグをいじる関数
int changeflag(const nav_msgs::Odometry::ConstPtr& msg){
   double orientation_z = 0;
   double orientation_w = 0;
   double th0 = 0;
   th0 = 2 * asin(msg->pose.pose.orientation.z / 2);
   double th1 = 0;
   th1 = 2 * acos(msg->pose.pose.orientation.w / 2);
}
*/

int main(int argc, char **argv){
   ros::init(argc, argv, "image_viewer");
   ros::NodeHandle n("~");
   ros::Subscriber sub;
   std::string odom_topic_;
   n.param<std::string>("odom_topic", odom_topic_, "/diff_drive_controller/odom"); 
   sub = n.subscribe(odom_topic_, 1000, chatterCallback);
   
   cv::Mat first_img;
   std::string first_path_;
   n.param<std::string>("first_path", first_path_, ros::package::getPath("rab_bringup") + "/picture/sample.JPG");
   if(n.getParam("first_path", first_path_)){
      ROS_INFO("Got Param: %s", first_path_.c_str());
   }else{
      n.setParam("first_path", first_path_.c_str());
      ROS_WARN("Set 'first_path' default!");
   }
   
   first_img = cv::imread(first_path_, 1);
   // 画像が読み込まれなかったら終了
   if(first_img.empty()) return -1;
      
   // 画像表示
   cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
   cv::imshow("Image", first_img);
   cv::waitKey(0);
   ros::spin();
   return 0;
}
