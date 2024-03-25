#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <Eigen/Dense>
#include <memory>

#include <pointcloud_obstacle_detection/ground_detection.hpp>

using namespace pointcloud_obstacle_detection;


class PointCloudSubscriberNode : public rclcpp::Node {
public:
    PointCloudSubscriberNode() : Node("point_cloud_subscriber") {

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "input_pointcloud", 10, std::bind(&PointCloudSubscriberNode::PointCloudCallback, this, std::placeholders::_1)
        );

        publisher3_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points_raw", 10);
        publisher1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_points", 10);
        publisher2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("non_ground_points", 10);
        pre_processor = std::make_unique<PointCloudGrid>(pre_processor_config);

        post_processor_config.cellSizeX = 2;
        post_processor_config.cellSizeY = 2;
        post_processor_config.cellSizeZ = 0.5;
        post_processor_config.groundInlierThreshold = 0.05; 
        post_processor = std::make_unique<PointCloudGrid>(post_processor_config);

    }

private:
    std::unique_ptr<PointCloudGrid> pre_processor;
    std::unique_ptr<PointCloudGrid> post_processor;
    GridConfig pre_processor_config;
    GridConfig post_processor_config;
    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2::SharedPtr gp_raw = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr gp = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr ngp = std::make_shared<sensor_msgs::msg::PointCloud2>();
        // Convert the ROS 2 PointCloud2 message to a PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
        Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);

        CloudXYZ pcl_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl_cloud);

        std::cout << "Input Cloud: " << pcl_cloud_ptr->points.size() << std::endl;

        //PRE
        pre_processor->setInputCloud(pcl_cloud_ptr, orientation);
        CloudPair pre_result = pre_processor->segmentPoints();
        CloudXYZ pre_ground_points = pre_result.first;
        CloudXYZ pre_non_ground_points = pre_result.second;

        //POST
        post_processor->setInputCloud(pre_ground_points, orientation);

        CloudPair post_result = post_processor->segmentPoints();
        CloudXYZ post_ground_points = post_result.first;
        CloudXYZ post_non_ground_points  = post_result.second;

        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = post_non_ground_points->begin(); it != post_non_ground_points->end(); ++it)
        {
            pre_non_ground_points->points.push_back(*it);
        }

        std::cout << "GN Cloud: " << post_ground_points->points.size() << std::endl;
        std::cout << "NG Cloud: " << pre_non_ground_points->points.size() << std::endl;

        // Convert PCL PointCloud to ROS PointCloud2 message
        pcl::toROSMsg(*pre_ground_points, *gp_raw);
        pcl::toROSMsg(*post_ground_points, *gp);
        pcl::toROSMsg(*pre_non_ground_points, *ngp);

        gp_raw->header.frame_id = "base_link";
        gp_raw->header.stamp = this->now();

        // Set the frame ID and timestamp
        gp->header.frame_id = "base_link";
        gp->header.stamp = this->now();
        // Set the frame ID and timestamp
        ngp->header.frame_id = "base_link";
        ngp->header.stamp = this->now();

        publisher3_->publish(*gp_raw);
        // Publish the message
        publisher1_->publish(*gp);
        publisher2_->publish(*ngp);

    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher1_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher3_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
