/* joy_control.launchからpublishされるcmd_velのデータをsubscribe、
   base_scanもsubscribeし、進みたい方向に障害物があればcmd_velを0にしてpublish */

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class JoyAvoidance{
 public:
   JoyAvoidance(ros::NodeHandle &nh);
   int isInArea(const sensor_msgs::LaserScan::ConstPtr& scan, 
		double angle_1, double angle_2, double min_dist, double max_dist);
   
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
   int front;
   int right;
   int left;
   int flag;
};

JoyAvoidance::JoyAvoidance(ros::NodeHandle &nh){
   ros::NodeHandle n("~");
   n.param<std::string>("sub_cmdvel_topic", sub_cmdvel_topic_, "/cmd_vel_old");
   n.param<std::string>("scan_topic", scan_topic_, "/base_scan");
   n.param<std::string>("pub_cmdvel_topic", pub_cmdvel_topic_, "/cmd_vel");
   // Subscriber
   sub_scan = nh.subscribe("/base_scan", 10, &JoyAvoidance::sensorCallback, this);
   sub_vel = nh.subscribe("/cmd_vel_old", 1, &JoyAvoidance::cmdvelCallback, this);
   // Publisher
   pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
}

int JoyAvoidance::isInArea(const sensor_msgs::LaserScan::ConstPtr& scan, 
			   double angle_1, double angle_2, double min_dist, double max_dist){
   double distance = 50.0;
   for(int i = angle_1; i <= angle_2; i++){
      if(min_dist < scan->ranges[i] && scan->ranges[i] < max_dist){
	 distance = scan->ranges[i];
      }
   }
   if(distance != 50.0){
      return 1;
   }else{
      return 0;
   }
}

void JoyAvoidance::sensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
   // 位置に応じてフラグ変更
   this->front = scan->ranges.size() / 2; // 正面方向
   this->right = scan->ranges.size() * 3 / 8; // 右45度方向
   this->left = scan->ranges.size() * 5 / 8; // 左45度方向
   // 右舷前方
   if(isInArea(scan, right, front, 1.5, 2.0) == 1){
      flag = 1;
      //ROS_INFO("右舷前方に障害物あり")
   }else if(isInArea(scan, right, front, 1.0, 1.5) == 1){
      flag = 2;
   }else if(isInArea(scan, right, front, 0.0, 1.0) == 1){
      flag = 3;
   }else{
      flag = 0;
   }
   // 左舷前方
   if(isInArea(scan, front, left, 1.5, 2.0) == 1){
      flag = 4;
   }else if(isInArea(scan, front, left, 1.0, 1.5) == 1){
      flag = 5;
   }else if(isInArea(scan, front, left, 0.0, 1.0) == 1){
      flag = 6;
   }else{
      flag = 0;
   }
}

void JoyAvoidance::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& vel){
   // フラグに応じてcmd_velの値を調整しをpublish
   geometry_msgs::Twist cmd_vel;
   // 右舷前方
   if(flag == 1 && vel->linear.x > 0.0 && vel->angular.z < 0.0){
      cmd_vel.linear.x = vel->linear.x * 0.8;
      cmd_vel.angular.z = vel->angular.z * 0.8;
   }else if(flag == 2 && vel->linear.x > 0.0 && vel->angular.z < 0.0){
      cmd_vel.linear.x = vel->linear.x * 0.5;
      cmd_vel.angular.z = vel->angular.z * 0.5;
   }else if(flag == 3 && vel->linear.x > 0.0 && vel->angular.z < 0.0){
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
   }else{
      cmd_vel.linear.x = vel->linear.x;
      cmd_vel.angular.z = vel->angular.z;
   }
   // 左舷前方
   if(flag == 4 && vel->linear.x > 0.0 && vel->angular.z > 0.0){
      cmd_vel.linear.x = vel->linear.x * 0.8;
      cmd_vel.angular.z = vel->angular.z * 0.8;
   }else if(flag == 5 && vel->linear.x > 0.0 && vel->angular.z > 0.0){
      cmd_vel.linear.x = vel->linear.x * 0.5;
      cmd_vel.angular.z = vel->angular.z * 0.5;
   }else if(flag == 6 && vel->linear.x > 0.0 && vel->angular.z > 0.0){
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
   }
   cmd_vel.linear.y = 0.0;
   pub_vel.publish(cmd_vel);
}

int main(int argc, char** argv){
   ros::init(argc, argv, "joy_avoidance");
   ros::NodeHandle n;
   JoyAvoidance joy_avoidance(n);
   ros::spin();
}