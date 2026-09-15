/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "image_overlay_node.hpp"

#ifdef ROS2
// ==================== ROS2 Implementation ====================

ImageOverlayNode::ImageOverlayNode(const rclcpp::NodeOptions& options)
    : Node("image_overlay_node", options)
{
    // Read from register_keys (same structure as control_command.yaml)
    this->declare_parameter<std::string>("register_keys.overlay_reprojected_topic", "/odin1/reprojected_image");
    this->declare_parameter<std::string>("register_keys.overlay_camera_topic", "/odin1/image/undistorted");
    this->declare_parameter<std::string>("register_keys.overlay_output_topic", "/odin1/overlay_image");
    this->declare_parameter<double>("register_keys.overlay_alpha", 0.6);

    reprojected_topic_ = this->get_parameter("register_keys.overlay_reprojected_topic").as_string();
    camera_topic_ = this->get_parameter("register_keys.overlay_camera_topic").as_string();
    overlay_topic_ = this->get_parameter("register_keys.overlay_output_topic").as_string();
    alpha_ = this->get_parameter("register_keys.overlay_alpha").as_double();

    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s and %s", 
                reprojected_topic_.c_str(), camera_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s (alpha=%.2f)", overlay_topic_.c_str(), alpha_);

    // Independent subscriptions - no synchronization needed
    reproj_sub_ = this->create_subscription<Image>(
        reprojected_topic_, 10,
        std::bind(&ImageOverlayNode::reprojCallback, this, std::placeholders::_1));
    
    camera_sub_ = this->create_subscription<Image>(
        camera_topic_, 10,
        std::bind(&ImageOverlayNode::cameraCallback, this, std::placeholders::_1));

    overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>(overlay_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "ImageOverlayNode initialized (no-sync mode)");
}

void ImageOverlayNode::reprojCallback(Image::ConstSharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_reproj_img_ = cv_ptr->image.clone();
            latest_header_ = msg->header;
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (reproj): %s", e.what());
        return;
    }
    publishOverlay();
}

void ImageOverlayNode::cameraCallback(Image::ConstSharedPtr msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_camera_img_ = cv_ptr->image.clone();
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception (camera): %s", e.what());
        return;
    }
    publishOverlay();
}

void ImageOverlayNode::publishOverlay()
{
    cv::Mat reproj_copy, camera_copy;
    std_msgs::msg::Header header_copy;
    
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_reproj_img_.empty() || latest_camera_img_.empty()) {
            return;
        }
        reproj_copy = latest_reproj_img_.clone();
        camera_copy = latest_camera_img_.clone();
        header_copy = latest_header_;
    }

    if (reproj_copy.size() != camera_copy.size()) {
        RCLCPP_WARN(this->get_logger(),
            "Image sizes don't match: reproj(%dx%d) vs camera(%dx%d)",
            reproj_copy.cols, reproj_copy.rows, 
            camera_copy.cols, camera_copy.rows);
        return;
    }

    // Create overlay using alpha blending
    // Replace white background in reproj with camera image, keep colored points
    cv::Mat overlay = camera_copy.clone();
    
    // Blend: where reproj has color (non-white), show reproj color semi-transparently
    // where reproj is white (background), show camera image
    
    for (int y = 0; y < reproj_copy.rows; ++y) {
        for (int x = 0; x < reproj_copy.cols; ++x) {
            cv::Vec3b reproj_pixel = reproj_copy.at<cv::Vec3b>(y, x);
            // Check if pixel is not white (has point cloud color)
            if (reproj_pixel[0] < 250 || reproj_pixel[1] < 250 || reproj_pixel[2] < 250) {
                // Blend reproj color with camera color
                cv::Vec3b cam_pixel = camera_copy.at<cv::Vec3b>(y, x);
                overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(
                    static_cast<uchar>(alpha_ * reproj_pixel[0] + (1 - alpha_) * cam_pixel[0]),
                    static_cast<uchar>(alpha_ * reproj_pixel[1] + (1 - alpha_) * cam_pixel[1]),
                    static_cast<uchar>(alpha_ * reproj_pixel[2] + (1 - alpha_) * cam_pixel[2])
                );
            }
            // else: keep camera image (already in overlay)
        }
    }

    auto overlay_msg = cv_bridge::CvImage(header_copy, "bgr8", overlay).toImageMsg();
    overlay_pub_->publish(*overlay_msg);
}

// ==================== ROS2 Main ====================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ImageOverlayNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#else
// ==================== ROS1 Implementation ====================

ImageOverlayNode::ImageOverlayNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
{
    // Read from register_keys (same structure as control_command.yaml)
    pnh_.param<std::string>("register_keys/overlay_reprojected_topic", reprojected_topic_, "/odin1/reprojected_image");
    pnh_.param<std::string>("register_keys/overlay_camera_topic", camera_topic_, "/odin1/image/undistorted");
    pnh_.param<std::string>("register_keys/overlay_output_topic", overlay_topic_, "/odin1/overlay_image");
    pnh_.param<double>("register_keys/overlay_alpha", alpha_, 0.6);

    ROS_INFO("Subscribing to: %s and %s", reprojected_topic_.c_str(), camera_topic_.c_str());
    ROS_INFO("Publishing to: %s (alpha=%.2f)", overlay_topic_.c_str(), alpha_);

    // Independent subscriptions - no synchronization needed
    reproj_sub_ = nh_.subscribe(reprojected_topic_, 10, &ImageOverlayNode::reprojCallback, this);
    camera_sub_ = nh_.subscribe(camera_topic_, 10, &ImageOverlayNode::cameraCallback, this);

    overlay_pub_ = nh_.advertise<sensor_msgs::Image>(overlay_topic_, 10);

    ROS_INFO("ImageOverlayNode initialized (no-sync mode)");
}

void ImageOverlayNode::reprojCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_reproj_img_ = cv_ptr->image.clone();
            latest_header_ = msg->header;
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception (reproj): %s", e.what());
        return;
    }
    publishOverlay();
}

void ImageOverlayNode::cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_camera_img_ = cv_ptr->image.clone();
        }
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception (camera): %s", e.what());
        return;
    }
    publishOverlay();
}

void ImageOverlayNode::publishOverlay()
{
    cv::Mat reproj_copy, camera_copy;
    std_msgs::Header header_copy;
    
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_reproj_img_.empty() || latest_camera_img_.empty()) {
            return;
        }
        reproj_copy = latest_reproj_img_.clone();
        camera_copy = latest_camera_img_.clone();
        header_copy = latest_header_;
    }

    if (reproj_copy.size() != camera_copy.size()) {
        ROS_WARN_THROTTLE(2, "Image sizes don't match: reproj(%dx%d) vs camera(%dx%d)",
                 reproj_copy.cols, reproj_copy.rows, camera_copy.cols, camera_copy.rows);
        return;
    }

    // Create overlay using alpha blending
    cv::Mat overlay = camera_copy.clone();
    
    for (int y = 0; y < reproj_copy.rows; ++y) {
        for (int x = 0; x < reproj_copy.cols; ++x) {
            cv::Vec3b reproj_pixel = reproj_copy.at<cv::Vec3b>(y, x);
            if (reproj_pixel[0] < 250 || reproj_pixel[1] < 250 || reproj_pixel[2] < 250) {
                cv::Vec3b cam_pixel = camera_copy.at<cv::Vec3b>(y, x);
                overlay.at<cv::Vec3b>(y, x) = cv::Vec3b(
                    static_cast<uchar>(alpha_ * reproj_pixel[0] + (1 - alpha_) * cam_pixel[0]),
                    static_cast<uchar>(alpha_ * reproj_pixel[1] + (1 - alpha_) * cam_pixel[1]),
                    static_cast<uchar>(alpha_ * reproj_pixel[2] + (1 - alpha_) * cam_pixel[2])
                );
            }
        }
    }

    sensor_msgs::ImagePtr overlay_msg = cv_bridge::CvImage(header_copy, "bgr8", overlay).toImageMsg();
    overlay_pub_.publish(overlay_msg);
}

// ==================== ROS1 Main ====================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_overlay_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ImageOverlayNode node(nh, pnh);

    ros::spin();
    return 0;
}
#endif
