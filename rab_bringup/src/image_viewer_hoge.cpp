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
double pose_x;
double pose_y;
   
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg/*, double pose_x, double pose_y*/){
   /*
   double orientation_z = 0;
   double orientation_w = 0;
   double th0 = 0;
   th0 = 2 * asin(msg->pose.pose.orientation.z / 2);
   double th1 = 0;
   th1 = 2 * acos(msg->pose.pose.orientation.w / 2);
   ROS_INFO("Seq: [%d]", msg->header.seq);
   */ 
   //ROS_INFO("Position-> x: [%lf], y: [%lf]", msg->pose.pose.position.x, msg->pose.pose.position.y);
   //ROS_INFO("Orientation-> th0: [%lf], th1: [%lf]", th0 * 180 / 3.14, th1 * 180 / 3.14);
   
   pose_x = msg->pose.pose.position.x;
   pose_y = msg->pose.pose.position.y;
   //ROS_WARN("position = %lf, %lf", pose_x, pose_y);
}

int isInSquare(const nav_msgs::Odometry::ConstPtr& msg, double x, double y, double meter){
   double current_x = msg->pose.pose.position.x;
   double current_y = msg->pose.pose.position.y;
   double plus_x = x + meter;
   double minus_x = x - meter;
   double plus_y = y + meter;
   double minus_y = y - meter;
   if(minus_x < current_x && current_x < plus_x && minus_y < current_y && current_y < plus_y){
      return 0;
   }else{
      return 1;
   }
}

int main(int argc, char **argv){
   ros::init(argc, argv, "image_viewer");
   ros::NodeHandle n("~");
   ros::Subscriber sub;
   int flag = 0;
   const nav_msgs::Odometry::ConstPtr odom;
   
   // subscribe topicの設定
   std::string odom_topic_;
   n.param<std::string>("odom_topic", odom_topic_, "/diff_drive_controller/odom"); 
   sub = n.subscribe(odom_topic_, 1000, odomCallback);
   ROS_WARN("position = %lf, %lf", pose_x, pose_y);
   
   // 画像読み込み１枚目
   cv::Mat first_img;
   std::string first_path_;
   n.param<std::string>("first_path", first_path_, ros::package::getPath("rab_bringup") + "/picture/sample.JPG");
   first_img = cv::imread(first_path_, 1);
   // 場所の設定
   double first_x_;
   n.param("first_x", first_x_, 0.0);
   double first_y_;
   n.param("first_y", first_y_, 0.0);
     
   // 2枚目
   cv::Mat second_img;
   std::string second_path_;
   n.param<std::string>("second_path", second_path_, ros::package::getPath("rab_bringup") + "/picture/ubuntu-logo.png");
   second_img = cv::imread(second_path_, 1);
   // 場所の設定
   double second_x_;
   n.param("second_x", second_x_, 0.0);
   double second_y_;
   n.param("second_y", second_y_, 0.0);
   
   // デフォルト画像
   cv::Mat default_img;
   std::string default_path_;
   n.param<std::string>("default_path", default_path_, ros::package::getPath("rab_bringup") + "/picture/nabe.jpg");
   default_img = cv::imread(default_path_, 1);
   /*
   // 画像が読み込まれなかったら終了
   if(first_img.empty() || second_img.empty()) return -1;
      
   // 画像表示
   cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

   // フラグの設定
   int count;
   //nav_msgs::Odomety pose;
   if(flag == 0 && isInSquare(odom, 0.0, 0.0, 2.0) == 0){
      flag = 1;
      cv::imshow("Image", first_img);
      cv::waitKey(1);
      count = 0;
   }else if(flag == 0 && isInSquare(odom, 10.0, 10.0, 2.0) == 0){
      flag = 1;
      cv::imshow("Image", second_img);
      cv::waitKey(1);
      count = 0;
   }else{
      flag = 0;
      count+=1;
   }
   if(count > 10){
      cv::imshow("Image", default_img);
      ROS_WARN("count = %d", count);
   }
   */
   ros::spin();
   return 0;
}
