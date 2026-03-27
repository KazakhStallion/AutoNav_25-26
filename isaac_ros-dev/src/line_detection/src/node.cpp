#include "line_detection/detection.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "autonav_interfaces/srv/anv_lines.hpp"
#include "autonav_interfaces/msg/line_points.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mutex>
#include <cstring>

class LineDetectorNode : public rclcpp::Node {

public:

	LineDetectorNode() : Node("lines"),
		tf_buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
		tf_listener(tf_buffer) {

		// Set parameters with ZED camera defaults
		this->declare_parameter("camera_topic", "/zed/zed_node/rgb/color/rect/image");
		this->declare_parameter("depth_camera_topic", "/zed/zed_node/depth/depth_registered");
		this->declare_parameter("camera_info_topic", "/zed/zed_node/rgb/color/rect/camera_info");
		this->declare_parameter("line_points_topic", "line_points");
		this->declare_parameter("target_frame", "map");
		this->declare_parameter("enable_timer", true);
		this->declare_parameter("publish_interval_ms", 250);
		this->declare_parameter("max_rgb_depth_delta_ms", 120);
		this->declare_parameter("tf_lookup_timeout_ms", 100);
		this->declare_parameter("line_hold_timeout_ms", 0);
		
		std::string camera_topic = this->get_parameter("camera_topic").as_string();
		std::string depth_camera_topic = this->get_parameter("depth_camera_topic").as_string();
		std::string camera_info_topic = this->get_parameter("camera_info_topic").as_string();
		std::string line_points_topic = this->get_parameter("line_points_topic").as_string();
		target_frame_ = this->get_parameter("target_frame").as_string();
		this->get_parameter("enable_timer", enable_timer_);
		publish_interval_ms_ = std::max<int64_t>(50, this->get_parameter("publish_interval_ms").as_int());
		max_rgb_depth_delta_ms_ = std::max<int64_t>(0, this->get_parameter("max_rgb_depth_delta_ms").as_int());
		tf_lookup_timeout_ms_ = std::max<int64_t>(0, this->get_parameter("tf_lookup_timeout_ms").as_int());
		line_hold_timeout_ms_ = std::max<int64_t>(0, this->get_parameter("line_hold_timeout_ms").as_int());

		RCLCPP_INFO(this->get_logger(), "Line Detection Config");
		RCLCPP_INFO(this->get_logger(), "Camera topic: %s", camera_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Depth topic: %s", depth_camera_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Camera info: %s", camera_info_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Output topic: %s", line_points_topic.c_str());
		RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
		RCLCPP_INFO(this->get_logger(), "Timer enabled: %s", enable_timer_ ? "true" : "false");
		RCLCPP_INFO(this->get_logger(), "Publish interval: %ld ms", publish_interval_ms_);
		RCLCPP_INFO(this->get_logger(), "RGB/depth max delta: %ld ms", max_rgb_depth_delta_ms_);
		RCLCPP_INFO(this->get_logger(), "TF lookup timeout: %ld ms", tf_lookup_timeout_ms_);
		RCLCPP_INFO(this->get_logger(), "Line hold timeout: %ld ms", line_hold_timeout_ms_);
		RCLCPP_INFO(this->get_logger(), "==================================");

		// Subscribe to camera topics
		auto get_latest_msg = [this](sensor_msgs::msg::Image::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(callback_lock);
			latest_img = msg;
		};
		auto get_latest_depth_msg = [this](sensor_msgs::msg::Image::SharedPtr msg) {
			std::lock_guard<std::mutex> lock(depth_callback_lock);
			latest_depth_img = msg;
		};
		
		_zed_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
			camera_topic, 10, get_latest_msg);

		_zed_depth_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
			depth_camera_topic, 10, get_latest_depth_msg);

		_camera_model_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
			camera_info_topic, 1, std::bind(&LineDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

		// Publishers
		_line_pub = this->create_publisher<autonav_interfaces::msg::LinePoints>(
			line_points_topic, 1);
			
		_line_timer = this->create_wall_timer(
			std::chrono::milliseconds(publish_interval_ms_),
			std::bind(&LineDetectorNode::line_callback, this));

		_line_point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
			"lines_pointcloud", 10);
			
		// Create service for line detection
		_line_service = this->create_service<autonav_interfaces::srv::AnvLines>(
			"line_service",
			std::bind(&LineDetectorNode::line_service, this, std::placeholders::_1, std::placeholders::_2));

		_clear_lines_service = this->create_service<std_srvs::srv::Trigger>(
			"clear_remembered_lines",
			std::bind(&LineDetectorNode::clearRememberedLines, this, std::placeholders::_1, std::placeholders::_2));
			
		RCLCPP_INFO(this->get_logger(), "LineDetectorNode initialized - waiting for camera data...");
	}

private:

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _zed_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _zed_depth_subscriber;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_model_sub;

	rclcpp::Service<autonav_interfaces::srv::AnvLines>::SharedPtr _line_service;
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _clear_lines_service;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _line_point_cloud_pub; 
	rclcpp::Publisher<autonav_interfaces::msg::LinePoints>::SharedPtr _line_pub;
	rclcpp::TimerBase::SharedPtr _line_timer;

	std::mutex callback_lock;
	std::mutex depth_callback_lock;
	std::mutex camera_params_lock;
	
	sensor_msgs::msg::Image::SharedPtr latest_img;
	sensor_msgs::msg::Image::SharedPtr latest_depth_img;
	
	tf2_ros::Buffer tf_buffer;
	tf2_ros::TransformListener tf_listener;
	
	// Store camera intrinsics directly - no PinholeCameraModel to avoid dangling pointer issues
	double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
	
	bool enable_timer_;
	bool configured_ = false;
	std::string target_frame_;
	int64_t publish_interval_ms_ = 250;
	int64_t max_rgb_depth_delta_ms_ = 120;
	int64_t tf_lookup_timeout_ms_ = 100;
	int64_t line_hold_timeout_ms_ = 0;
	autonav_interfaces::msg::LinePoints last_valid_message_;
	rclcpp::Time last_valid_detection_time_{0, 0, RCL_ROS_TIME};
	bool has_last_valid_message_ = false;

	void line_service(
		const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
		std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response);
	void clearRememberedLines(
		const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
		std::shared_ptr<std_srvs::srv::Trigger::Response> response);
	
	void line_callback();

	std::vector<Eigen::Vector3d> map_transform(
		const sensor_msgs::msg::Image::SharedPtr depth_msg, 
		int2* line_points, 
		int line_points_len); 

	bool imagesAreSynchronized(
		const sensor_msgs::msg::Image::SharedPtr & camera_msg,
		const sensor_msgs::msg::Image::SharedPtr & depth_msg);

	autonav_interfaces::msg::LinePoints makeLinePointsMessage(
		const std::vector<Eigen::Vector3d> & points,
		const builtin_interfaces::msg::Time & stamp) const;

	void publishLinePoints(const autonav_interfaces::msg::LinePoints & message);
	void publishLivePointCloud(
		const std::vector<Eigen::Vector3d> & points,
		const builtin_interfaces::msg::Time & stamp);
	void publishEmptyPointCloud(const builtin_interfaces::msg::Time & stamp);
	void cacheAndPublishLinePoints(
		const std::vector<Eigen::Vector3d> & points,
		const builtin_interfaces::msg::Time & stamp);
	void publishHeldOrEmpty(const char * reason);

	void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
};

void LineDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
	std::lock_guard<std::mutex> lock(camera_params_lock);
	
	// Extract camera intrinsics
	fx_ = msg->k[0];  // Focal length X
	fy_ = msg->k[4];  // Focal length Y
	cx_ = msg->k[2];  // Principal point X
	cy_ = msg->k[5];  // Principal point Y
	
	if (!configured_) {
		configured_ = true;
		RCLCPP_INFO(this->get_logger(), "Camera parameters initialized: fx=%.1f, fy=%.1f, cx=%.1f, cy=%.1f",
			fx_, fy_, cx_, cy_);
		
		if (enable_timer_) {
			RCLCPP_INFO(this->get_logger(), "Publishing enabled on topic: %s", 
				this->get_parameter("line_points_topic").as_string().c_str());
		}
	}
}

sensor_msgs::msg::PointCloud2 createPointCloud(
	const std::vector<std::array<float, 3>>& points, 
	const std::string& frame_id,
	const builtin_interfaces::msg::Time& timestamp) 
{
	sensor_msgs::msg::PointCloud2 pointcloud;
	pointcloud.header.frame_id = frame_id;
	pointcloud.header.stamp = timestamp;
	pointcloud.height = 1;
	pointcloud.width = points.size();
	pointcloud.is_dense = false;
	pointcloud.is_bigendian = false;

	sensor_msgs::PointCloud2Modifier modifier(pointcloud);
	modifier.setPointCloud2FieldsByString(1, "xyz");
	modifier.resize(points.size());

	sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");

	for (const auto& point : points) {
		*iter_x = point[0];
		*iter_y = point[1];
		*iter_z = point[2];
		++iter_x;
		++iter_y;
		++iter_z;
	}

	return pointcloud;
}

bool LineDetectorNode::imagesAreSynchronized(
	const sensor_msgs::msg::Image::SharedPtr & camera_msg,
	const sensor_msgs::msg::Image::SharedPtr & depth_msg)
{
	if (!camera_msg || !depth_msg) {
		return false;
	}

	const rclcpp::Time camera_stamp(camera_msg->header.stamp);
	const rclcpp::Time depth_stamp(depth_msg->header.stamp);
	const rclcpp::Duration stamp_delta =
		(camera_stamp >= depth_stamp) ? (camera_stamp - depth_stamp) : (depth_stamp - camera_stamp);
	const rclcpp::Duration max_delta = rclcpp::Duration::from_nanoseconds(
		max_rgb_depth_delta_ms_ * 1000000LL);

	if (stamp_delta > max_delta) {
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 3000,
			"Skipping line update: RGB/depth timestamps differ by %.1f ms (limit %.1f ms)",
			stamp_delta.seconds() * 1000.0, max_delta.seconds() * 1000.0);
		return false;
	}

	return true;
}

autonav_interfaces::msg::LinePoints LineDetectorNode::makeLinePointsMessage(
	const std::vector<Eigen::Vector3d> & points,
	const builtin_interfaces::msg::Time & stamp) const
{
	auto message = autonav_interfaces::msg::LinePoints();
	message.header.frame_id = target_frame_;
	message.header.stamp = stamp;
	message.points.reserve(points.size());
	for (const auto & point : points) {
		geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = point.x();
		vec_msg.y = point.y();
		vec_msg.z = point.z();
		message.points.emplace_back(vec_msg);
	}
	return message;
}

void LineDetectorNode::publishLinePoints(const autonav_interfaces::msg::LinePoints & message)
{
	_line_pub->publish(message);
}

void LineDetectorNode::publishLivePointCloud(
	const std::vector<Eigen::Vector3d> & points,
	const builtin_interfaces::msg::Time & stamp)
{
	std::vector<std::array<float, 3>> point_cloud_points;
	point_cloud_points.reserve(points.size());
	for (const auto & point : points) {
		point_cloud_points.push_back(
			{static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())});
	}

	sensor_msgs::msg::PointCloud2 pc = createPointCloud(point_cloud_points, target_frame_, stamp);
	_line_point_cloud_pub->publish(pc);
}

void LineDetectorNode::publishEmptyPointCloud(const builtin_interfaces::msg::Time & stamp)
{
	sensor_msgs::msg::PointCloud2 pc = createPointCloud({}, target_frame_, stamp);
	_line_point_cloud_pub->publish(pc);
}

void LineDetectorNode::cacheAndPublishLinePoints(
	const std::vector<Eigen::Vector3d> & points,
	const builtin_interfaces::msg::Time & stamp)
{
	last_valid_message_ = makeLinePointsMessage(points, stamp);
	last_valid_detection_time_ = this->now();
	has_last_valid_message_ = !last_valid_message_.points.empty();
	publishLinePoints(last_valid_message_);
}

void LineDetectorNode::publishHeldOrEmpty(const char * reason)
{
	publishEmptyPointCloud(this->now());

	if (has_last_valid_message_ && !last_valid_message_.points.empty()) {
		const rclcpp::Time now = this->now();

		if (line_hold_timeout_ms_ > 0) {
			const rclcpp::Duration hold_timeout =
				rclcpp::Duration::from_nanoseconds(line_hold_timeout_ms_ * 1000000LL);
			const rclcpp::Duration age = now - last_valid_detection_time_;
			if (age > hold_timeout) {
				auto empty_message = autonav_interfaces::msg::LinePoints();
				empty_message.header.frame_id = target_frame_;
				empty_message.header.stamp = now;
				has_last_valid_message_ = false;
				last_valid_message_ = autonav_interfaces::msg::LinePoints();
				last_valid_detection_time_ = now;
				RCLCPP_WARN_THROTTLE(
					get_logger(), *get_clock(), 3000,
					"Clearing held line obstacle set after %s because cached data is %.1f ms old",
					reason, age.seconds() * 1000.0);
				publishLinePoints(empty_message);
				return;
			}
		}

		auto held_message = last_valid_message_;
		held_message.header.stamp = now;
		RCLCPP_WARN_THROTTLE(
			get_logger(), *get_clock(), 3000,
			"Reusing remembered line obstacle set after %s",
			reason);
		publishLinePoints(held_message);
		return;
	}

	RCLCPP_WARN_THROTTLE(
		get_logger(), *get_clock(), 3000,
		"Skipping line publish after %s because no valid line set is cached yet",
		reason);

	auto empty_message = autonav_interfaces::msg::LinePoints();
	empty_message.header.frame_id = target_frame_;
	empty_message.header.stamp = this->now();
	publishLinePoints(empty_message);
}

void LineDetectorNode::clearRememberedLines(
	const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
	std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
	(void)request;

	has_last_valid_message_ = false;
	last_valid_message_ = autonav_interfaces::msg::LinePoints();
	last_valid_detection_time_ = this->now();

	auto empty_message = autonav_interfaces::msg::LinePoints();
	empty_message.header.frame_id = target_frame_;
	empty_message.header.stamp = this->now();
	publishLinePoints(empty_message);
	publishEmptyPointCloud(empty_message.header.stamp);

	response->success = true;
	response->message = "Cleared remembered line obstacle set";
	RCLCPP_INFO(this->get_logger(), "Cleared remembered line obstacle set");
}

/**
 * Converts a list of image indices to target frame coordinates.
 */
std::vector<Eigen::Vector3d> LineDetectorNode::map_transform(
	const sensor_msgs::msg::Image::SharedPtr depth_msg, 
	int2* line_points, 
	int line_points_len) 
{
	std::vector<Eigen::Vector3d> depth_line_points;
	
	// Early validation
	if (line_points_len <= 0) {
		return depth_line_points;
	}
	
	if (!line_points || !depth_msg || depth_msg->data.empty()) {
		RCLCPP_ERROR(get_logger(), "Invalid input data");
		return depth_line_points;
	}

	// Get camera parameters
	double fx, fy, cx, cy;
	{
		std::lock_guard<std::mutex> lock(camera_params_lock);
		if (!configured_) {
			RCLCPP_ERROR(get_logger(), "Camera not configured!");
			return depth_line_points;
		}
		fx = fx_;
		fy = fy_;
		cx = cx_;
		cy = cy_;
	}

	// Verify encoding
	if (depth_msg->encoding != "32FC1") {
		RCLCPP_ERROR(get_logger(), "Unexpected depth encoding: %s (expected 32FC1)", 
			depth_msg->encoding.c_str());
		return depth_line_points;
	}

	const size_t row_step = depth_msg->step;
	const size_t bytes_per_pixel = sizeof(float);
	const uint8_t* depth_ptr_u8 = depth_msg->data.data();
	std::string frame_id = depth_msg->header.frame_id;
	std::vector<std::array<float, 3>> pc_vec;
	pc_vec.reserve(std::min(line_points_len, 10000));

	// Check if transform is available
	bool transform_available = false;
	geometry_msgs::msg::TransformStamped transform;
	
	try {
		const rclcpp::Time depth_stamp(depth_msg->header.stamp);
		transform = tf_buffer.lookupTransform(
			target_frame_,
			frame_id, 
			depth_stamp,
			rclcpp::Duration::from_nanoseconds(tf_lookup_timeout_ms_ * 1000000LL)
		);
		transform_available = true;
	} catch (const tf2::TransformException& ex) {
		RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
			"TF not available at image stamp (%s <- %s): %s",
			target_frame_.c_str(), frame_id.c_str(), ex.what());
	}

	int valid_count = 0;
	int invalid_depth = 0;
	int out_of_bounds = 0;
	int tf_success = 0;

	// Process each line point
	for (int i = 0; i < line_points_len; i++) {
		// Bounds checking
		if (line_points[i].x < 0 || line_points[i].x >= (int)depth_msg->width ||
			line_points[i].y < 0 || line_points[i].y >= (int)depth_msg->height) {
			out_of_bounds++;
			continue;
		}

		// Get depth value (meters)
		const size_t offset = (size_t)line_points[i].y * row_step + (size_t)line_points[i].x * bytes_per_pixel;
		if (offset + sizeof(float) > depth_msg->data.size()) {
			continue;
		}
		
		float depth_m;
		std::memcpy(&depth_m, depth_ptr_u8 + offset, sizeof(float));
		
		// 0.1m to 20m range
		if (depth_m < 0.1f || depth_m > 20.0f || std::isnan(depth_m) || std::isinf(depth_m)) {
			invalid_depth++;
			continue;
		}
		
		valid_count++;

		// Manual 3D proj
		double x_normalized = (line_points[i].x - cx) / fx;
		double y_normalized = (line_points[i].y - cy) / fy;
		
		double px = x_normalized * depth_m;
		double py = y_normalized * depth_m;
		double pz = depth_m;
		
		if (std::isnan(px) || std::isnan(py) || std::isnan(pz)) {
			continue;
		}

		// Add to pointcloud
		pc_vec.push_back({static_cast<float>(px), static_cast<float>(py), static_cast<float>(pz)});

		// TF to target frame if available
		if (transform_available) {
			try {
				geometry_msgs::msg::PointStamped camera_point;
				camera_point.header = depth_msg->header;
				camera_point.point.x = px;
				camera_point.point.y = py;
				camera_point.point.z = pz;
				
				geometry_msgs::msg::PointStamped target_point;
				tf2::doTransform(camera_point, target_point, transform);
				
				if (!std::isnan(target_point.point.x) && !std::isnan(target_point.point.y)) {
					depth_line_points.emplace_back(
						target_point.point.x,
						target_point.point.y,
						0.0  // Project to ground plane
					);
					tf_success++;
				}
			} catch (const std::exception& ex) {
			}
		}
	}

	RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
		"Processing: %d valid, %d invalid_depth, %d out_of_bounds -> %d transformed points",
		valid_count, invalid_depth, out_of_bounds, tf_success);

	// Publish live detections only; remembered obstacles are represented by /line_points.
	try {
		if (!depth_line_points.empty()) {
			publishLivePointCloud(depth_line_points, depth_msg->header.stamp);
		} else {
			publishEmptyPointCloud(depth_msg->header.stamp);
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR_ONCE(get_logger(), "Pointcloud publish failed: %s", e.what());
	}

	return depth_line_points;
}

void LineDetectorNode::line_service(
	const std::shared_ptr<autonav_interfaces::srv::AnvLines::Request> request,
	std::shared_ptr<autonav_interfaces::srv::AnvLines::Response> response)
{
	(void)request;

	// Get latest images
	sensor_msgs::msg::Image::SharedPtr camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(callback_lock);
		return latest_img;
	}();
	
	sensor_msgs::msg::Image::SharedPtr depth_camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(depth_callback_lock);
		return latest_depth_img;
	}();

	if (!camera_msg || !depth_camera_msg) {
		RCLCPP_ERROR(this->get_logger(), "No camera images available");
		return;
	}
	if (!imagesAreSynchronized(camera_msg, depth_camera_msg)) {
		return;
	}

	// Convert camera image to grayscale
	cv_bridge::CvImagePtr cv_ptr;
	try {
		if (camera_msg->encoding == "bgra8" || camera_msg->encoding == "bgr8") {
			cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
			cv::Mat gray;
			cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
			cv_ptr->image = gray;
		} else {
			cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::MONO8);
		}
	} catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		return;
	}

	// Detect lines
	std::pair<int2*,int*> line_pair = lines::detect_line_pixels(cv_ptr->image);
	int2* line_points = line_pair.first;
	int* line_points_len = line_pair.second;

	std::vector<Eigen::Vector3d> transformed_points = map_transform(depth_camera_msg, line_points, *line_points_len);

	// Populate response
	for (const auto & point: transformed_points) {
		geometry_msgs::msg::Vector3 vec_msg;
		vec_msg.x = point.x();
		vec_msg.y = point.y();
		vec_msg.z = point.z();
		response->points.emplace_back(vec_msg);
	}

	// Free memory
	delete[] line_points;
	delete line_points_len;
}

void LineDetectorNode::line_callback()
{
	// Check prerequisites
	if (!configured_){
		return;
	}
	
	if (!enable_timer_) return;

	// Get latest images
	sensor_msgs::msg::Image::SharedPtr camera_msg = [this]() {
		std::lock_guard<std::mutex> lock(callback_lock);
		return latest_img;
	}();
	
	sensor_msgs::msg::Image::SharedPtr depth_msg = [this]() {
		std::lock_guard<std::mutex> lock(depth_callback_lock);
		return latest_depth_img;
	}();

	if (!camera_msg || !depth_msg) {
		publishHeldOrEmpty("missing camera/depth image");
		return;
	}
	if (!imagesAreSynchronized(camera_msg, depth_msg)) {
		publishHeldOrEmpty("RGB/depth desynchronization");
		return;
	}

	// Convert to grayscale
	cv_bridge::CvImagePtr cv_ptr;
	try {
		if (camera_msg->encoding == "bgra8" || camera_msg->encoding == "bgr8") {
			cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
			cv::Mat gray;
			cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
			cv_ptr->image = gray;
		} else {
			cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::MONO8);
		}
	} catch (cv_bridge::Exception& e) {
		RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
		publishHeldOrEmpty("cv_bridge exception");
		return;
	}
	
	if (cv_ptr->image.empty() || cv_ptr->image.type() != CV_8UC1) {
		RCLCPP_ERROR(this->get_logger(), "Invalid image after conversion");
		publishHeldOrEmpty("invalid grayscale image");
		return;
	}

	// Detect lines
	std::pair<int2*,int*> line_pair;
	try {
		line_pair = lines::detect_line_pixels(cv_ptr->image);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Line detection failed: %s", e.what());
		publishHeldOrEmpty("line detection failure");
		return;
	}
	
	int2* line_points = line_pair.first;
	int* line_points_len = line_pair.second;

	RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
		"Detected %d line pixels", *line_points_len);
	
	if (*line_points_len == 0) {
		publishHeldOrEmpty("empty line detection");
		delete[] line_points;
		delete line_points_len;
		return;
	}

	// Transform to target frame
	std::vector<Eigen::Vector3d> transformed_points;
	try {
		transformed_points = map_transform(depth_msg, line_points, *line_points_len);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", e.what());
		publishHeldOrEmpty("transform failure");
		delete[] line_points;
		delete line_points_len;
		return;
	}

	if (transformed_points.empty()) {
		publishHeldOrEmpty("no transformed points");
		delete[] line_points;
		delete line_points_len;
		return;
	}

	// Cache and publish the latest detection set so brief misses do not flicker the line costmap.
	cacheAndPublishLinePoints(transformed_points, depth_msg->header.stamp);
	if (!transformed_points.empty()) {
		RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
			"Published %zu line points", transformed_points.size());
	}

	// Free memory
	delete[] line_points;
	delete line_points_len;
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<LineDetectorNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
