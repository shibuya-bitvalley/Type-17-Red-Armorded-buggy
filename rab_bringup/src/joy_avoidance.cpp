/* joy_control.launchからpublishされるcmd_velのデータをsubscribe、
   base_scanもsubscribeし、進みたい方向に障害物があればcmd_velを0にしてpublish */

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
   
   // param for topic
   std::string sub_cmdvel_topic_;
   std::string scan_topic_;
   std::string pub_cmdvel_topic_;
   
   // declaration of menber variable
   double front;
   double right;
   double left;
   int flag;
};

JoyAvoidance::JoyAvoidance(ros::NodeHandle &nh){
   ros::NodeHandle n("~");
   n.param<std::string>("sub_cmdvel_topic", sub_cmdvel_topic_, "\"/cmd_vel_old\"");
   n.param<std::string>("scan_topic", scan_topic_, "\"/base_scan\"");
   n.param<std::string>("pub_cmdvel_topic", pub_cmdvel_topic_, "\"/cmd_vel\"");
   // Subscriber
   sub_scan = nh.subscribe("/base_scan", 10, &JoyAvoidance::sensorCallback, this);
   sub_vel = nh.subscribe("/cmd_vel_old", 1, &JoyAvoidance::cmdvelCallback, this);
   // Publisher
   pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
}

void JoyAvoidance::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
   // 位置に応じてフラグ変更
   this->front = scan->ranges.size() / 2; // 正面方向
   this->right = scan->ranges.size() / 4; // 右90度方向
   this->left = scan->ranges.size() * 3 / 4; // 左90度方向
   ROS_INFO("front range = %lf", scan->ranges[front]);
   ROS_INFO("right limit range = %lf", scan->ranges[right]);
   ROS_INFO("left limit range = %lf", scan->ranges[left]);
   // 正面
   if(1.5 < scan->ranges[front]  && scan->ranges[front] <= 2.0){
      flag = 1;
   }else if(1.0 < scan->ranges[front] && scan->ranges[front] <= 1.5){
      flag = 2;
   }else if(scan->ranges[front] <= 1.0){
      flag = 3;
   }else{
      flag = 0;
   }
   // 右舷
   if(0.75 <= scan->ranges[right] && scan->ranges[right] < 1.0){
      flag = 4;
   }else if(0.5 < scan->ranges[right] && scan->ranges[right] <= 0.75){
      flag = 5;
   }else if(scan->ranges[right] <= 0.5){
      flag = 6;
   }else{
      flag = 0;
   }
   // 左舷
   if(0.75 <= scan->ranges[left] && scan->ranges[left] < 1.0){
      flag = 7;
   }else if(0.5 < scan->ranges[left] && scan->ranges[left] <= 0.75){
      flag = 8;
   }else if(scan->ranges[left] <= 0.5){
      flag = 9;
   }else{
      flag = 0;
   }
   //ROS_ERROR("flag : %d", flag);
}

void JoyAvoidance::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& vel){
   // フラグに応じてcmd_velの値を調整しをpublish
   geometry_msgs::Twist cmd_vel;
   // front
   if(flag == 1 && vel->linear.x > 0.0){
      cmd_vel.linear.x = vel->linear.x * 0.8;
   }else if(flag == 2 && vel->linear.x > 0.0){
      cmd_vel.linear.x = vel->linear.x * 0.5;
   }else if(flag == 3 && vel->linear.x > 0.0){
      cmd_vel.linear.x = 0.0;
   }else{
      cmd_vel.linear.x = vel->linear.x;
   }
   // right
   if(flag == 4 && vel->angular.z < 0.0){
      cmd_vel.angular.z = vel->angular.z * 0.8;
   }else if(flag == 5 && vel->angular.z < 0.0){
      cmd_vel.angular.z = vel->angular.z * 0.5;
   }else if(flag == 6 && vel->angular.z < 0.0){
      cmd_vel.angular.z = 0.0;
   }else{
      cmd_vel.angular.z = vel->angular.z;
   }
   // left
   if(flag == 7 && vel->angular.z > 0.0){
      cmd_vel.angular.z = vel->angular.z * 0.8;
   }else if(flag == 8 && vel->angular.z > 0.0){
      cmd_vel.angular.z = vel->angular.z * 0.5;
   }else if(flag == 9 && vel->angular.z > 0.0){
      cmd_vel.angular.z = 0.0;
   }else{
      cmd_vel.angular.z = vel->angular.z;
   }
   cmd_vel.linear.z = vel->linear.z;
   cmd_vel.linear.y = 0.0;
   pub_vel.publish(cmd_vel);
}

int main(int argc, char** argv){
   ros::init(argc, argv, "joy_avoidance");
   ros::NodeHandle n;
   JoyAvoidance joy_avoidance(n);
   ros::spin();
}