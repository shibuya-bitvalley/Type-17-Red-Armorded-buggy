#include <ros/ros.h>

int main(int argc, char **argv){
   ros::init(argc, argv, "param_test");
   ros::NodeHandle n;
   //パラメーターセットのためのテスト
   std::string s;
   if(n.getParam("my_pram", s)){
      ROS_INFO("Got param: %s", s.c_str());
   }else{
      ROS_ERROR("Faild to get param 'my_param'");
   }
   ros::spin();
}

   
   