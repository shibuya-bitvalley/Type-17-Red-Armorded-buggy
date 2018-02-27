#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class DigitalSignage{
 public:
   DigitalSignage(ros::NodeHandle &nh);

 private:
   int isInSquare(const nav_msgs::Odometry::ConstPtr& msg, double x, double y, double meter);
   void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
   void publish_image(const nav_msgs::Odometry::ConstPtr& msg);
   int check_image();   

   int flag;   //画像を表示するためのフラグ
   double pose_x;
   double pose_y;
   
   // subscriber
   ros::Subscriber sub_odom;

   // param for topic
   std::string odom_topic_;
   std::string first_path_;
   std::string second_path_;
   std::string default_path_;
   double first_x_;
   double first_y_;
   double second_x_;
   double second_y_;
   
   cv::Mat first_img;
   cv::Mat second_img;
   cv::Mat default_img;
   
   int count;

};
   
DigitalSignage::DigitalSignage(ros::NodeHandle &nh){
   // using parameter server
   ros::NodeHandle n("~");
   count = -10;
   
   // 画像読み込み１枚目
   n.param<std::string>("first_path", first_path_, ros::package::getPath("rab_bringup") + "/picture/sample.JPG");
   first_img = cv::imread(first_path_, 1);
   // 表示場所の設定
   n.param("first_x", first_x_, 0.0);
   n.param("first_y", first_y_, 0.0);
   
   // 2枚目
   n.param<std::string>("second_path", second_path_, ros::package::getPath("rab_bringup") + "/picture/ubuntu-logo.png");
   second_img = cv::imread(second_path_, 1);
   // 表示場所の設定
   n.param("second_x", second_x_, 0.0);
   n.param("second_y", second_y_, 0.0);
   
   // デフォルト画像
   n.param<std::string>("default_path", default_path_, ros::package::getPath("rab_bringup") + "/picture/nabe.jpg");
   default_img = cv::imread(default_path_, 1);
   
   // subscribe topicの設定
   n.param<std::string>("odom_topic", odom_topic_, "\"/diff_drive_controller/odom\"");
   ROS_INFO("Subscribe topic : %s",odom_topic_.c_str());
   sub_odom = nh.subscribe(odom_topic_, 100, &DigitalSignage::odomCallback, this);
}

int DigitalSignage::isInSquare(const nav_msgs::Odometry::ConstPtr& msg, double x, double y, double meter){
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

void DigitalSignage::publish_image(const nav_msgs::Odometry::ConstPtr& msg){
   // 画像表示
   if(count > 250 || count <= -1){
      flag = 0;
      cv::imshow("Image", default_img);
      cv::waitKey(1);
   }
   if(flag <= 10 && isInSquare(msg, first_x_, first_y_, 2.0) == 0){
      flag = 1;	 
      cv::imshow("Image", first_img);
      cv::waitKey(1);	  
      count = 0;
   }else if(flag <= 10 && isInSquare(msg, second_x_, second_y_, 2.0) == 0){
      flag = 1;	 
      cv::imshow("Image", second_img);
      cv::waitKey(1);	  
      count = 0;
   }else{
      flag = 1;
      count+=1;
   }
   ROS_WARN("count = %d", count);
}

void DigitalSignage::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
   /*
    double orientation_z = 0;
    double orientation_w = 0;
    double th0 = 0;
    th0 = 2 * asin(msg->pose.pose.orientation.z / 2);
    double th1 = 0;
    th1 = 2 * acos(msg->pose.pose.orientation.w / 2);
    ROS_INFO("Seq: [%d]", msg->header.seq);
    */ 
   ROS_INFO("Position-> x: [%lf], y: [%lf]", msg->pose.pose.position.x, msg->pose.pose.position.y);
   //ROS_INFO("Orientation-> th0: [%lf], th1: [%lf]", th0 * 180 / 3.14, th1 * 180 / 3.14);
   publish_image(msg);
}

/*
int DigitalSignage::check_image(){
   // 画像が読み込まれなかったら終了
   if(first_img.empty() || second_img.empty() || default_img.empty()){
      ROS_ERROR("ERROR");
      return -1;	 
   }else{
      ROS_INFO("load success");
      return 0;
   }  
}*/   

int main(int argc, char **argv){
   ros::init(argc, argv, "image_viewer");
   ros::NodeHandle nh;
   DigitalSignage digital_signage(nh);

   // window作成
   cv::namedWindow("Image", CV_WINDOW_AUTOSIZE | CV_GUI_NORMAL);
   cv::moveWindow("Image", 0, 0);

   ros::spin();
   return 0;
}
