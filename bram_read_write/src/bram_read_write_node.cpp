#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include "BRAM-uio-driver/src/bram_uio.h"

class BramReadWriteNode : public rclcpp::Node
{
public:
    BramReadWriteNode()
        : Node("bram_read_write_node")
    {

	 // Create a publisher for the topic "control_number" with queue size of 10
        control_number_publisher_ = this->create_publisher<std_msgs::msg::Int32>("control_number", 10);

        // Subscribe to the processed image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/processed_image", 10,
            std::bind(&BramReadWriteNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
	BRAM bram(0,8192);
        try
        {
            // Convert ROS Image message to OpenCV image
            cv::Mat cv_image = cv_bridge::toCvCopy(msg, "32FC1")->image;
	    
	    int bram_iter = 0;
            // Iterate through the image and convert to bytearray
            for (int i = 0; i < cv_image.rows; ++i) {
           	 for (int j = 0; j < cv_image.cols; ++j) {
               		 float pixel_value = cv_image.at<float>(i, j);

               		 // Pack float into 4-byte representation
               		 int floatBytes;
			 std::memcpy(&floatBytes, &pixel_value, sizeof(float));
			 bram[bram_iter] = floatBytes;
			 bram_iter++; 
                }
    	    }

            // Debug: Print the byte array size
            RCLCPP_INFO(this->get_logger(), "Received image and converted to byte array (size: %zu)", cv_image.rows*cv_image.cols);

            sleep(0.1);

	    // Create a message of type Int32
            auto message = std_msgs::msg::Int32();
            message.data = bram[128];  // Example value to publish

            // Publish the message to the "control_number" topic
            RCLCPP_INFO(this->get_logger(), "Publishing control_number: %d", message.data);
            control_number_publisher_->publish(message);
	}
	catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to process image: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr control_number_publisher_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BramReadWriteNode>());
    rclcpp::shutdown();
    return 0;
}
