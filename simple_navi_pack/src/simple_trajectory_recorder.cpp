/*
    This program was written by Naoki Akai.
    akai@coi.nagoya-u.ac.jp
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "simple_trajectory_recorder");
	ros::NodeHandle nh("~");
	tf::TransformListener tf_listener;
	tf::StampedTransform world_to_base;
	std::string world_frame = "/world";
	std::string base_frame = "/base_link";
	std::string path_file_name = "/tmp/simple_trajectory.txt";
	double dist_interval = 0.5; // m
	double angle_interval = 2.0; // deg
	nh.param("/simple_trajectory_recorder/world_frame", world_frame, world_frame);
	nh.param("/simple_trajectory_recorder/base_frame", base_frame, base_frame);
	nh.param("/simple_trajectory_recorder/path_file_name", path_file_name, path_file_name);
	nh.param("/simple_trajectory_recorder/dist_interval", dist_interval, dist_interval);
	nh.param("/simple_trajectory_recorder/angle_interval", angle_interval, angle_interval);
	FILE *fp = fopen(path_file_name.c_str(), "w");
	if (fp == NULL) {
		ROS_ERROR("cannot open path file, %s", path_file_name.c_str());
		exit(1);
	}
	angle_interval *= M_PI / 180.0;
	bool is_first = true;
	double xo, yo, yawo;
	ros::Rate loop_rate(20);
	while (ros::ok()) {
		ros::Time now = ros::Time::now();
		try {
			tf_listener.waitForTransform(world_frame, base_frame, now, ros::Duration(1));
			tf_listener.lookupTransform(world_frame, base_frame, now, world_to_base);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());
			continue;
		}
		double x = world_to_base.getOrigin().x();
		double y = world_to_base.getOrigin().y();
		tf::Quaternion q(world_to_base.getRotation().x(),
			world_to_base.getRotation().y(),
			world_to_base.getRotation().z(),
			world_to_base.getRotation().w());
		double roll, pitch, yaw;
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		if (is_first) {
			fprintf(fp, "%lf %lf %lf\n", x, y, yaw);
			fprintf(stderr, "%lf %lf %lf\n", x, y, yaw);
			xo = x;
			yo = y;
			yawo = yaw;
			is_first = false;
		} else {
			double dx = x - xo;
			double dy = y - yo;
			double dl = sqrt(dx * dx + dy * dy);
			double dyaw = yaw - yawo;
			if (dyaw < -M_PI)	dyaw += 2.0 * M_PI;
			if (dyaw > M_PI)	dyaw -= 2.0 * M_PI;
			if (dl > dist_interval || fabs(dyaw) > angle_interval) {
				fprintf(fp, "%lf %lf %lf\n", x, y, yaw);
				fprintf(stderr, "%lf %lf %lf\n", x, y, yaw);
				xo = x;
				yo = y;
				yawo = yaw;
			}
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	fclose(fp);
	return 0;
}
