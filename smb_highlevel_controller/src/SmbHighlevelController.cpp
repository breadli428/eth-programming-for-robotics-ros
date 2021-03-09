#include "smb_highlevel_controller/SmbHighlevelController.hpp"
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle), tfListener(tfBuffer)
{
  if (!readParameters()) {
	  ROS_ERROR("Could not read parameters.");
	  ros::requestShutdown();
  }
//  subscriber_scan_ = nodeHandle_.subscribe(subscriberTopic_scan_, queue_size, &SmbHighlevelController::topicCallback_scan, this);
//  subscriber_rslidar_points_ = nodeHandle_.subscribe(subscriberTopic_rslidar_points_, queue_size, &SmbHighlevelController::topicCallback_rslidar_points, this);
  subscriber_scan_ = nodeHandle_.subscribe(subscriberTopic_scan_, queue_size, &SmbHighlevelController::topicCallback_pillar, this);

  publisher_twist_ = nodeHandle_.advertise<geometry_msgs::Twist>(publisherTopic_cmd_vel, queue_size);
  vis_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>(publisherTopic_visualization_marker, 0);

  service = nodeHandle_.advertiseService(service_start_stop, &SmbHighlevelController::startStop, this);

}

SmbHighlevelController::~SmbHighlevelController()
{
}


// Calculate minimum range from topic "/scan"

float SmbHighlevelController::minimumRange(const sensor_msgs::LaserScan::_ranges_type& data)
{
	float minimum_range;
	minimum_range = *std::min_element(data.begin(), data.end());
	return minimum_range;
}

//void SmbHighlevelController::topicCallback_scan(const sensor_msgs::LaserScan& message)
//{
//	float minimum_range;
//	minimum_range = minimumRange(message.ranges);
//	ROS_INFO_STREAM("Minimum Range: " << minimum_range);
//}


// Calculate total points from topic "/rslidar_points"
//
//int SmbHighlevelController::totalPoints(const sensor_msgs::PointCloud2::_row_step_type& data1, const sensor_msgs::PointCloud2::_height_type& data2)
//{
//	int total_points;
//	total_points = data1 * data2;
//	return total_points;
//}
//
//void SmbHighlevelController::topicCallback_rslidar_points(const sensor_msgs::PointCloud2& message)
//{
//	int total_points;
//	total_points = totalPoints(message.row_step, message.height);
//	ROS_INFO_STREAM("Total Points: " << total_points);
//}


float SmbHighlevelController::detectPillar(const sensor_msgs::LaserScan& message)
{
	sensor_msgs::LaserScan::_ranges_type data;
	data = message.ranges;

	unsigned int minimum_range_index;
	minimum_range_index = std::min_element(data.begin(),data.end()) - data.begin();

	float orientation;
	orientation = message.angle_min + message.angle_increment * minimum_range_index;

	return orientation;
}


void SmbHighlevelController::topicCallback_pillar(const sensor_msgs::LaserScan& message)
{
	float remaining_dist;
	remaining_dist = minimumRange(message.ranges);
	ROS_INFO_STREAM("Remaining Distance: " << remaining_dist);

	float orientation;
	orientation = detectPillar(message);
	ROS_INFO_STREAM("Pillar Orientation: " << orientation);

	float angular_gain;
	if (!nodeHandle_.getParam("angular_gain", angular_gain)){
		  ROS_ERROR("Could not read angular gain.");
	}


	geometry_msgs::Twist cmd_vel;
	if (start_flag){
		cmd_vel.linear.x = 0.5;
		cmd_vel.angular.z = angular_gain * orientation;
	}
	else{
        cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.0;

	}
	publisher_twist_.publish(cmd_vel);

	// <--Point position in frame rslidar-->
	geometry_msgs::PointStamped point_in, point_out;
	point_in.header.frame_id = "rslidar";
	point_in.point.x = remaining_dist * std::cos(orientation);
	point_in.point.y = remaining_dist * std::sin(orientation);
	ROS_INFO_STREAM("Pillar Position x: " << point_in.point.x);
	ROS_INFO_STREAM("Pillar Position y: " << point_in.point.y);



	// <--Method1: Direct publication in "rslidar" frame-->

	visualization_msgs::Marker marker;
	marker.header.frame_id = "rslidar";
	marker.header.stamp = ros::Time();
	marker.ns = "sphere_namespace_method_1";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = point_in.point.x;
	marker.pose.position.y = point_in.point.y;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	vis_pub_.publish(marker);


	// <--Method2: Frame transformation using TF2-->

	try {
		tfBuffer.transform(point_in, point_out, "odom");

	} catch (tf2::TransformException &exception) {
		ROS_WARN("%s", exception.what());
		ros::Duration(1.0).sleep();
	}

	marker.header.frame_id = "odom";
	marker.header.stamp = ros::Time();
	marker.ns = "sphere_namespace_method_2";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = point_out.point.x;
	marker.pose.position.y = point_out.point.y;
	marker.pose.position.z = point_out.point.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;
	marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    vis_pub_.publish(marker);


}

bool SmbHighlevelController::startStop(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
	start_flag = request.data;

	if (start_flag){
		response.message = "Robot started";
	}
	else{
		response.message = "Robot stopped";
	}
    ROS_INFO_STREAM(response.message);
    response.success = true;
    return true;
}




bool SmbHighlevelController::readParameters()
{
	if (!nodeHandle_.getParam("subscriber_topic_scan", subscriberTopic_scan_)) return false;
	if (!nodeHandle_.getParam("subscriber_topic_rslidar_points", subscriberTopic_rslidar_points_)) return false;
	if (!nodeHandle_.getParam("publisher_topic_cmd_vel", publisherTopic_cmd_vel)) return false;
	if (!nodeHandle_.getParam("publisher_topic_visualization_marker", publisherTopic_visualization_marker)) return false;
	if (!nodeHandle_.getParam("service_start_stop", service_start_stop)) return false;

	if (!nodeHandle_.getParam("queue_size", queue_size)) return false;
	return true;
}


} /* namespace */
