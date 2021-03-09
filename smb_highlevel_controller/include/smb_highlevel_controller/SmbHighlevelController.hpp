#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/SetBool.h>


namespace smb_highlevel_controller {

/*!
 * Class containing the Highlevel Controller
 */
class SmbHighlevelController {
public:
	/*!
	 * Constructor.
	 */
	SmbHighlevelController(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SmbHighlevelController();

private:
	/*!
	 * Reads and verifies the ROS parameters.
	 * @return true if successful.
	 */
	bool readParameters();

	/*!
	 * ROS topic callback method.
	 * @param message the received message.
	 */

	//! ROS node handle.
	ros::NodeHandle& nodeHandle_;

	//! ROS topic subscriber of "/scan".
	ros::Subscriber subscriber_scan_;

	//! ROS topic subscriber of "/rslidar_points".
	ros::Subscriber subscriber_rslidar_points_;

	//! ROS topic name to subscribe to.
	std::string subscriberTopic_scan_;

	//! ROS topic name to subscribe to.
	std::string subscriberTopic_rslidar_points_;

	//! ROS topic publisher of "/cmd_vel".
	ros::Publisher publisher_twist_;

	//! ROS topic publisher of "/visualization_marker".
	ros::Publisher vis_pub_;

	//! ROS topic name to subscribe to.
	std::string publisherTopic_cmd_vel;

	//! ROS topic name to publish marker.
	std::string publisherTopic_visualization_marker;

	//! ROS service.
	std::string service_start_stop;

	//! ROS queue size to subscribe to.
	int queue_size;

	//! ROS service server.
	ros::ServiceServer service;

	tf2_ros::Buffer tfBuffer;

	tf2_ros::TransformListener tfListener;

	float minimumRange(const sensor_msgs::LaserScan::_ranges_type& data);

//	void topicCallback_scan(const sensor_msgs::LaserScan& message);

//	int totalPoints(const sensor_msgs::PointCloud2::_row_step_type& data1, const sensor_msgs::PointCloud2::_height_type& data2);

//	void topicCallback_rslidar_points(const sensor_msgs::PointCloud2& message);

	float detectPillar(const sensor_msgs::LaserScan& message);

	void topicCallback_pillar(const sensor_msgs::LaserScan& message);

	bool startStop(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);

	bool start_flag = 1;


};

} /* namespace */
