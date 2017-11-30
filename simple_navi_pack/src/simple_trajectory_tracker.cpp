/*
    This program was written by Naoki Akai.
    akai@coi.nagoya-u.ac.jp
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define DEBUG

typedef struct {
	double x, y, yaw;
} point_t;

int main(int argc, char **argv) {
	ros::init(argc, argv, "simple_trakectory_tracker");
	ros::NodeHandle nh("~");
	tf::TransformListener tf_listener;
	tf::StampedTransform world_to_base;
	std::string world_frame = "/world";
	std::string base_frame = "/base_link";
	std::string path_file_name = "/tmp/simple_trajectory.txt";
	double look_ahead_dist = 1.0; // m
	double max_vel = 1.0;
	double kv = 0.6;
	std::string twist_cmd_name = "/twist_cmd";
	bool debug = true;
	bool iterate_tracking = false;
	nh.param("/simple_trajectory_tracker/world_frame", world_frame, world_frame);
	nh.param("/simple_trajectory_tracker/base_frame", base_frame, base_frame);
	nh.param("/simple_trajectory_tracker/path_file_name", path_file_name, path_file_name);
	nh.param("/simple_trajectory_tracker/look_ahead_dist", look_ahead_dist, look_ahead_dist);
	nh.param("/simple_trajectory_tracker/max_vel", max_vel, max_vel);
	nh.param("/simple_trajectory_tracker/kv", kv, kv);
	nh.param("/simple_trajectory_tracker/twist_cmd_name", twist_cmd_name, twist_cmd_name);
	nh.param("/simple_trajectory_tracker/debug", debug, debug);
	nh.param("/simple_trajectory_tracker/iterate_tracking", iterate_tracking, iterate_tracking);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>(twist_cmd_name, 100);
	ros::Rate loop_rate(20);

	// load path
	std::vector<point_t> path;
	FILE *fp = fopen(path_file_name.c_str(), "r");
	if (fp == NULL) {
		ROS_ERROR("cannot open path file, %s", path_file_name.c_str());
		exit(1);
	}
	point_t p;
	while (fscanf(fp, "%lf %lf %lf", &p.x, &p.y, &p.yaw) != EOF)
		path.push_back(p);
	fclose(fp);

	// start trajectory tracking
	int target_index = 0;
	double eo = 0.0;
	geometry_msgs::TwistStamped twist_cmd;
	while (ros::ok()) {
		// read robot pose from tf tree
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

		// compute twist cmd using PD control
		static int target_index = 1;
		double dx = path[target_index].x - path[target_index - 1].x;
		double dy = path[target_index].y - path[target_index - 1].y;
		double th = atan2(dy, dx);
		double x2p = dx * cos(th) + dy * sin(th);
		dx = path[target_index].x - x;
		dy = path[target_index].y - y;
		double xrp = dx * cos(th) + dy * sin(th);
		double yrp = -dx * sin(th) + dy * cos(th);
		double dl = fabs(x2p - xrp);
		if (dl < look_ahead_dist) {
			target_index++;
			if (target_index >= path.size() && iterate_tracking) // reach at the goal
				target_index = 1;
			else if (target_index >= path.size() && !iterate_tracking)
				break;
			dx = path[target_index].x - x;
			dy = path[target_index].y - y;
		}
		double t = atan2(dy, dx);
		double e = t - yaw;
		if (e < -M_PI)	e += 2.0 * M_PI;
		if (e > M_PI)	e -= 2.0 * M_PI;
		twist_cmd.header.stamp = ros::Time::now();
		twist_cmd.twist.linear.x = max_vel - kv * fabs(e);
		twist_cmd.twist.angular.z = 0.4 * e + 0.01 * eo;
		eo = e;
		twist_pub.publish(twist_cmd);
		ros::spinOnce();
		loop_rate.sleep();

		if (debug) {
			fprintf(stderr, "robot pose: x = %lf, y = %lf, yaw = %lf\n", x, y, yaw);
			fprintf(stderr, "target path pose: x = %lf, y = %lf, yaw = %lf\n", path[target_index].x, path[target_index].y, path[target_index].yaw);
			fprintf(stderr, "vel = %lf, ang_vel = %lf\n", twist_cmd.twist.linear.x, twist_cmd.twist.angular.z);
			fprintf(stderr, "\n");
		}
	}
	// publish stop command
	for (int i = 0; i < 10; i++) {
		twist_cmd.header.stamp = ros::Time::now();
		twist_cmd.twist.linear.x = 0.0;
		twist_cmd.twist.angular.z = 0.0;
		twist_pub.publish(twist_cmd);
		usleep(100000);
	}
	return 0;
}
