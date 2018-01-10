#include <ros/ros.h>
#include <ros/package.h>
//#include <nav_msgs/Odometry.h>
//#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

/*
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
*/

int main(int argc, char **argv){
   ros::init(argc, argv, "image_viewer");
   ros::NodeHandle n("~");
   //ros::Subscriber sub;
   //sub = n.subscribe("/odom", 1000, chatterCallback);
   
   cv::Mat first_img;
   std::string first_path;
   std::string image_path_;
   n.param<std::string>("image_path", first_path, ros::package::getPath("rab_bringup") + "/picture/sample.JPG");
   if(n.getParam("image_path", image_path_)){
      ROS_INFO("Got Param: %s", image_path_.c_str());
   }else{
      n.setParam("image_path", image_path_.c_str());
      ROS_WARN("Set 'image_path' default!");
   }
   //ROS_INFO("first path = %s", first_path.c_str());
   first_img = cv::imread(first_path, 1);
   // 画像が読み込まれなかったら終了
   if(first_img.empty()) return -1;
   
   // 画像表示
   cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
   cv::imshow("Image", first_img);
   cv::waitKey();
   ros::spin();
   return 0;
}
