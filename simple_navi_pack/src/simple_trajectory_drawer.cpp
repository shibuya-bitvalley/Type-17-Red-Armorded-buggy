/*
   This program was written by Naoki Akai.
   akai@coi.nagoya-u.ac.jp
*/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#define DRAW_INTERVAL (0.5) // m

std::string world_frame = "/world";
std::string path_file_name = "/tmp/simple_trajectory.txt";
std::vector<geometry_msgs::Point> path;
ros::Publisher path_pub;

void make_path(void) {
	FILE *fp = fopen(path_file_name.c_str(), "r");
	if (fp == NULL) {
		ROS_ERROR("cannot open path file");
		exit(1);
	}
	int c = 0;
	double xo, yo, yawo;
	int val = fscanf(fp, "%lf %lf %lf\n", &xo, &yo, &yawo);
	double x, y, yaw;
	geometry_msgs::Point p;
	while ((fscanf(fp, "%lf %lf %lf\n", &x, &y, &yaw)) != EOF) {
		double dx = x - xo;
		double dy = y - yo;
		double dl = sqrt(dx * dx + dy * dy);
		if (dl > DRAW_INTERVAL) {
			p.x = x;
			p.y = y;
			path.push_back(p);
			xo = x;
			yo = y;
			yawo = yaw;
		}
	}
	fclose(fp);
}

void draw_path(void) {
	geometry_msgs::Point p;
	visualization_msgs::Marker target_path;
	target_path.header.frame_id = world_frame;
	target_path.header.stamp = ros::Time::now();
	target_path.ns = "target_path";
	target_path.action = visualization_msgs::Marker::ADD;
	target_path.pose.orientation.w = 1.0;
	target_path.lifetime = ros::Duration(2);
	target_path.frame_locked = true;
	target_path.id = 1;
	target_path.type = visualization_msgs::Marker::LINE_STRIP;
	target_path.scale.x = 0.15;
	target_path.scale.y = 1.0;
	target_path.scale.z = 1.0;
	target_path.color.r = 1.0;
	target_path.color.g = 0.0;
	target_path.color.b = 0.0;
	target_path.color.a = 1.0;
	for (int i = 0; i < path.size(); i++) {
		target_path.points.push_back(path[i]);
		target_path.points.push_back(path[i]);
	}
	path_pub.publish(target_path);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "simple_trajectory_drawer");
	ros::NodeHandle nh("~");
	nh.param("/simple_trajectory_drawer/world_frame", world_frame, world_frame);
	nh.param("/simple_trajectory_drawer/path_file_name", path_file_name, path_file_name);
	ros::Rate loop_rate(1);
	path_pub = nh.advertise<visualization_msgs::Marker>("/simple_trajectory", 1);
	make_path();
	while (ros::ok()) {
		ros::spinOnce();
		draw_path();
		loop_rate.sleep();
	}
	return 0;
}
