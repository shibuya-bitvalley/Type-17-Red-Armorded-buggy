#include <ros/ros.h>

int main(int argc, char **argv){
   ros::init(argc, argv, "param_test");
   ros::NodeHandle n("~");
   //パラメーターセットのためのテスト
   std::string my_param_;
   int second_param_;
   
   n.param<std::string>("my_param", my_param_, "default_hoge");
   if(n.getParam("my_param", my_param_)){
      ROS_INFO("Got param: %s", my_param_.c_str());
   }else{
      ROS_WARN("Faild to get param 'my_param'");
      n.setParam("my_param", my_param_.c_str());
      ROS_INFO("Set param default '%s'", my_param_.c_str());
   }
   ROS_INFO("my_param = %s", my_param_.c_str());
   
   n.param("second_param", second_param_, 19);
   if(n.getParam("second_param", second_param_)){
      ROS_INFO("Got param: %d", second_param_);
   }else{
      ROS_WARN("Faild to get param 'second_param'");
      n.setParam("second_param", second_param_);
      ROS_INFO("Set param default '%d'", second_param_);
   }
   ROS_INFO("second_param = %d", second_param_);
   
   ros::spin();
}

   
   