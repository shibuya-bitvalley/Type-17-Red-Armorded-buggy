/*
   This program was written by Naoki Akai.
   akai@coi.nagoya-u.ac.jp
*/
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

ros::Publisher avoid_cmd_pub, local_map_pub;
cv::Mat local_map;
geometry_msgs::TwistStamped current_cmd;
double old_target_w;
bool do_stop = false, do_back = false;
bool is_first_stop = true;
double first_stop_time, first_back_time;

// param
double map_width = 10.0; // m
double map_height = 10.0; // m
double map_pixel = 0.05; // m
double robot_width = 1.0; // m
double predict_time = 5.0; // m
double step_time = 0.1; // m
double kv = 0.8; // %
double w_range = 0.4; // rad/sec
std::string source_twist_cmd_name = "/twist_cmd";
std::string output_twist_cmd_name = "/twist_cmd_with_avoidance";
std::string scan_name = "/scan";
bool use_emergency_stop = true;
double stop_x_size = 0.6; // m
double stop_y_size = 0.4; // m
double stop_time = 3.0; // sec
double back_time = 5.0; // sec
double back_speed = 0.2; // m/sec

void init_local_map(void) {
	int w = (int)(map_width / map_pixel);
	int h = (int)(map_height / map_pixel);
	local_map = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
}

void cmd_callback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
	current_cmd = *msg;
}

bool check_collision(double v, double w, double *free_dist, int green) {
	double x0, y0, t0, d0;
	double dd = v * 0.1;
	double dt = w * 0.1;
	double dist = v * predict_time;
	double cos90 = cos(M_PI / 2.0);
	double sin90 = sin(M_PI / 2.0);
	x0 = y0 = t0 = d0 = 0.0;
	for (double t = 0.0; t < predict_time; t += step_time) {
		x0 += dd * cos(t0);
		y0 += dd * sin(t0);
		t0 += dt;
		d0 += dd;
		double tt = t0 + M_PI / 2.0;
		for (double l = -robot_width / 2; l <= robot_width / 2; l += map_pixel) {
			double xx = x0 + l * cos(tt);
			double yy = y0 + l * sin(tt);
			double x = xx * cos90 - yy * sin90;
			double y = xx * sin90 + yy * cos90;
			int u = local_map.cols / 2 + (int)(x / map_pixel);
			int v = local_map.rows - (int)(y / map_pixel);
			if (0 <= u && u < local_map.cols && 0 <= v && v < local_map.rows) {
				if (local_map.at<cv::Vec3b>(v, u)[2] == 255) {
					*free_dist = d0;
					return true;
				} else {
					local_map.at<cv::Vec3b>(v, u)[1] = green;
				}
			}
		}
		if (d0 > dist)	break;
	}
	*free_dist = dist;
	return false;
}

bool do_emergency_stop(const sensor_msgs::LaserScan::ConstPtr &msg) {
	int i = 0, c = 0;
	for (double t = msg->angle_min; t <= msg->angle_max; t += msg->angle_increment) {
		double r = msg->ranges[i];
		if (r < msg->range_min || msg->range_max < r) {
			i++;
			continue;
		}
		double angle = t + M_PI / 2.0;
		double x = r * cos(angle);
		double y = r * sin(angle);
		if (fabs(x) < stop_x_size && y < stop_y_size) {
			c++;
			if (c >= 5)
				return true;
		}
		i++;
	}
	return false;
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg) {
	// memorize current path following command
	double v0 = current_cmd.twist.linear.x;
	double w0 = current_cmd.twist.angular.z;

	// mapping urg data
	local_map = cv::Scalar::all(0);
	int i = 0;
	for (double t = msg->angle_min; t <= msg->angle_max; t += msg->angle_increment) {
		double r = msg->ranges[i];
		if (r < msg->range_min || msg->range_max < r) {
			i++;
			continue;
		}
		double angle = t + M_PI / 2.0;
		double x = r * cos(angle);
		double y = r * sin(angle);
		int u = local_map.cols / 2 + (int)(x / map_pixel);
		int v = local_map.rows - (int)(y / map_pixel);
		if (1 <= u && u < local_map.cols -  1 && 1 <= v && v < local_map.rows - 1) {
			local_map.at<cv::Vec3b>(v, u)[2] = 255;
			local_map.at<cv::Vec3b>(v, u - 1)[2] = 255;
			local_map.at<cv::Vec3b>(v, u + 1)[2] = 255;
			local_map.at<cv::Vec3b>(v - 1, u)[2] = 255;
			local_map.at<cv::Vec3b>(v + 1, u)[2] = 255;
		}
		i++;
	}

	// check collision and search obstacle avoidance command
	double free_dist, target_v, target_w;
	if (check_collision(v0, w0, &free_dist, 50)) {
		double v = v0 * kv;
		bool is_first = true;
		double max_e;
		for (double w = old_target_w - w_range; w <= old_target_w + w_range; w += 0.02) {
			check_collision(v, w, &free_dist, 50);
			double dw = w0 - w;
			double e = free_dist * exp(-(dw * dw) / (2.0 * 0.5 * 0.5));
			if (is_first) {
				target_v = v;
				target_w = w;
				max_e = e;
				is_first = false;
			} else if (max_e < e) {
				target_v = v;
				target_w = w;
				max_e = e;
			}
		}
		check_collision(target_v, target_w, &free_dist, 255);
	} else {
		target_v = v0;
		target_w = w0;
	}
	old_target_w = target_w;

	// emergency stop and back
	double current_time = ros::Time::now().toSec();
	bool do_stop = false;
	if (use_emergency_stop)
		do_stop = do_emergency_stop(msg);
	if (do_stop) {
		target_v = target_w = 0.0;
		if (is_first_stop) {
			first_stop_time = current_time;
			is_first_stop = false;
		} else if (current_time - first_stop_time > stop_time && !do_back) {
			first_back_time = current_time;
			do_back = true;
		}
	} else {
		is_first_stop = true;
	}

	if (do_back) {
		target_v = -back_speed;
		if (target_v > 0.0)	target_v *= -1.0;
		target_w = 0.0;
		if (current_time - first_back_time > back_time) {
			target_v = target_w = 0.0;
			do_back = false;
		}
	}

	// publish
	geometry_msgs::TwistStamped avoid_cmd;
	avoid_cmd.header = msg->header;
	avoid_cmd.twist.linear.x = target_v;
	avoid_cmd.twist.angular.z = target_w;
	avoid_cmd_pub.publish(avoid_cmd);
	cv_bridge::CvImage local_map_msg(msg->header, "bgr8", local_map);
	local_map_pub.publish(local_map_msg.toImageMsg());
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "simple_obstacle_avoidance");
	ros::NodeHandle nh("~");
	nh.param("/simple_obstacle_avoidance/map_width", map_width, map_width);
	nh.param("/simple_obstacle_avoidance/map_height", map_height, map_height);
	nh.param("/simple_obstacle_avoidance/map_pixel", map_pixel, map_pixel);
	nh.param("/simple_obstacle_avoidance/robot_width", robot_width, robot_width);
	nh.param("/simple_obstacle_avoidance/predict_time", predict_time, predict_time);
	nh.param("/simple_obstacle_avoidance/step_time", step_time, step_time);
	nh.param("/simple_obstacle_avoidance/kv", kv, kv);
	nh.param("/simple_obstacle_avoidance/w_range", w_range, w_range);
	nh.param("/simple_obstacle_avoidance/source_twist_cmd_name", source_twist_cmd_name, source_twist_cmd_name);
	nh.param("/simple_obstacle_avoidance/output_twist_cmd_name", output_twist_cmd_name, output_twist_cmd_name);
	nh.param("/simple_obstacle_avoidance/scan_name", scan_name, scan_name);
	nh.param("/simple_obstacle_avoidance/use_emergency_stop", use_emergency_stop, use_emergency_stop);
	nh.param("/simple_obstacle_avoidance/stop_x_size", stop_x_size, stop_x_size);
	nh.param("/simple_obstacle_avoidance/stop_y_size", stop_y_size, stop_y_size);
	nh.param("/simple_obstacle_avoidance/stop_time", stop_time, stop_time);
	nh.param("/simple_obstacle_avoidance/back_time", back_time, back_time);
	nh.param("/simple_obstacle_avoidance/back_speed", back_speed, back_speed);
	ros::Subscriber cmd_sub = nh.subscribe(source_twist_cmd_name, 100, cmd_callback);
	ros::Subscriber scan_sub = nh.subscribe(scan_name, 10, scan_callback);
	avoid_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>(output_twist_cmd_name, 100);
	local_map_pub = nh.advertise<sensor_msgs::Image>("/local_map_for_simple_obstacle_avoidance", 100);
	init_local_map();
	ros::spin();
	return 0;
}
