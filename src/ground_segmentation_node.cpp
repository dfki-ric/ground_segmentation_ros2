#define PCL_NO_PRECOMPILE
#include "rclcpp/rclcpp.hpp"
#include <pointcloud_obstacle_detection/ground_detection_types.hpp>
#include <pointcloud_obstacle_detection/ground_detection.hpp>
#include "common.hpp"

#include "ground_segmentation/msg/grid_map.hpp"
#include "ground_segmentation/msg/grid_cell.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/extract_indices.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>
#include <memory>
#include <chrono>
#include <vector>
#include <limits>

using namespace pointcloud_obstacle_detection;
using PointType = PointXYZILID;

class GroundSegmentatioNode : public rclcpp::Node {
public:
    GroundSegmentatioNode(rclcpp::NodeOptions options) : Node("ground_segmentation",options) {

        subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ground_segmentation/input", 10, std::bind(&GroundSegmentatioNode::PointCloudCallback, this, std::placeholders::_1)
        );

        publisher_ground_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/ground_points", 10);
        publisher_obstacle_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/obstacle_points", 10);
        publisher_raw_ground_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/filtered_input", 10);
        publisher_start_cells = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ground_segmentation/start_cells", 10);

        pub_tp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/TP", 10);
        pub_fn = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/FN", 10);
        pub_fp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_segmentation/FP", 10);

        pre_grid_map_publisher = this->create_publisher<ground_segmentation::msg::GridMap>("/ground_segmentation/pre_grid_map", 10);
        post_grid_map_publisher = this->create_publisher<ground_segmentation::msg::GridMap>("/ground_segmentation/post_grid_map", 10);

        double cellSizeX = this->get_parameter("cellSizeX").as_double();
        double cellSizeY = this->get_parameter("cellSizeY").as_double();
        double cellSizeZ = this->get_parameter("cellSizeZ").as_double();
        double startCellDistanceThreshold = this->get_parameter("startCellDistanceThreshold").as_double();
        double slopeThresholdDegrees = this->get_parameter("slopeThresholdDegrees").as_double();
        double groundInlierThreshold = this->get_parameter("groundInlierThreshold").as_double();

        show_benchmark = this->get_parameter("show_benchmark").as_bool();
        show_seed_cells = this->get_parameter("show_seed_cells").as_bool();

        pre_processor_config.cellSizeX = cellSizeX;
        pre_processor_config.cellSizeY = cellSizeY;
        pre_processor_config.cellSizeZ = cellSizeZ;
        pre_processor_config.startCellDistanceThreshold = startCellDistanceThreshold;
        pre_processor_config.slopeThresholdDegrees = slopeThresholdDegrees;
        pre_processor_config.groundInlierThreshold = groundInlierThreshold;             

        post_processor_config = pre_processor_config;
        post_processor_config.cellSizeZ = 0.5;
        post_processor_config.processing_phase = 2;

        pre_processor = std::make_unique<PointCloudGrid<PointType>>(pre_processor_config);
        post_processor = std::make_unique<PointCloudGrid<PointType>>(post_processor_config);

        // controller feedback (via TF)
        buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

        precision = 0.0;
        recall = 0.0;
    }

private:

    bool show_benchmark;
    bool show_seed_cells;

    double precision, recall;

    std::vector<double> runtime;
    std::vector<double> recall_arr;
    std::vector<double> prec_arr;
    std::vector<double> recall_o_arr;
    std::vector<double> prec_o_arr;

    std::shared_ptr<tf2_ros::Buffer> buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    ProcessPointCloud<PointType> processor;
    std::unique_ptr<PointCloudGrid<PointType>> pre_processor;
    std::unique_ptr<PointCloudGrid<PointType>> post_processor;
    GridConfig pre_processor_config;
    GridConfig post_processor_config;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_raw_ground_points;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_ground_points;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_obstacle_points;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_start_cells;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_tp;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_fn;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_fp;

    rclcpp::Publisher<ground_segmentation::msg::GridMap>::SharedPtr pre_grid_map_publisher;
    rclcpp::Publisher<ground_segmentation::msg::GridMap>::SharedPtr post_grid_map_publisher;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;

    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2::SharedPtr raw_ground_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr ground_points = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr obstacle_points = std::make_shared<sensor_msgs::msg::PointCloud2>();

        sensor_msgs::msg::PointCloud2::SharedPtr msg_TP = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr msg_FP = std::make_shared<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::msg::PointCloud2::SharedPtr msg_FN = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // Convert the ROS 2 PointCloud2 message to a PCL PointCloud
        typename pcl::PointCloud<PointType> input_cloud;
        typename pcl::PointCloud<PointType> transformed_cloud;
        pcl::fromROSMsg(*msg, input_cloud);

        std::string target_frame = this->get_parameter("target_frame").as_string();
        double maxX = this->get_parameter("maxX").as_double();
        double minX = this->get_parameter("minX").as_double();
        double maxY = this->get_parameter("maxY").as_double();
        double minY = this->get_parameter("minY").as_double();
        double maxZ = this->get_parameter("maxZ").as_double();
        double minZ = this->get_parameter("minZ").as_double();
        bool downsample = this->get_parameter("downsample").as_bool();
        double downsample_resolution = this->get_parameter("downsample_resolution").as_double();

        typename pcl::PointCloud<PointType>::Ptr input_cloud_ptr;
        if (target_frame != msg->header.frame_id){
            try {
                // Lookup transform from lidar frame to base_link frame
                const geometry_msgs::msg::TransformStamped transformStamped = buffer->lookupTransform(
                    target_frame, msg->header.frame_id, tf2::TimePointZero);

                // Transform each point
                pcl_ros::transformPointCloud(input_cloud, transformed_cloud, transformStamped);

                // Process transformed point cloud
                // Your processing logic here
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Transform exception: %s", ex.what());
                return;
            }        
            input_cloud_ptr = std::make_shared<typename pcl::PointCloud<PointType>>(transformed_cloud);
        }
        else{
            input_cloud_ptr = std::make_shared<typename pcl::PointCloud<PointType>>(input_cloud);
        }
        
        //TODO: Read robot orientation
        Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
        Eigen::Vector4f min{minX,minY,minZ, 1};
        Eigen::Vector4f max{maxX,maxY,maxZ,1};

        //Start time
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        typename pcl::PointCloud<PointType>::Ptr filtered_cloud_ptr = processor.FilterCloud(input_cloud_ptr, downsample, downsample_resolution, min, max);

        //PRE
        pre_processor->setInputCloud(filtered_cloud_ptr, orientation);
        std::pair< typename pcl::PointCloud<PointType>::Ptr,  typename pcl::PointCloud<PointType>::Ptr> pre_result = pre_processor->segmentPoints();
        typename pcl::PointCloud<PointType>::Ptr pre_ground_points = pre_result.first;
        typename pcl::PointCloud<PointType>::Ptr pre_non_ground_points = pre_result.second;

        //POST
        post_processor->setInputCloud(pre_ground_points, orientation);
        std::pair< typename pcl::PointCloud<PointType>::Ptr,  typename pcl::PointCloud<PointType>::Ptr> post_result = post_processor->segmentPoints();
        typename pcl::PointCloud<PointType>::Ptr post_ground_points = post_result.first;
        typename pcl::PointCloud<PointType>::Ptr post_non_ground_points  = post_result.second;

        typename pcl::PointCloud<PointType>::Ptr final_non_ground_points(new pcl::PointCloud<PointType>());

        *final_non_ground_points = *pre_non_ground_points + *post_non_ground_points;

        if (show_benchmark){
            //End time
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            double rt = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() * 0.001;
            // Estimation
            static double      precision, recall, precision_wo_veg, recall_wo_veg;
            static double      precision_o, recall_o;
            static std::vector<int> TPFNs; // TP, FP, FN, TF order
            static std::vector<int> TPFNs_wo_veg; // TP, FP, FN, TF order
            static std::vector<int> TPFNs_o; // TP, FP, FN, TF order

            calculate_precision_recall(*filtered_cloud_ptr, *post_ground_points, precision, recall, TPFNs);
            calculate_precision_recall_without_vegetation(*filtered_cloud_ptr, *post_ground_points, precision_wo_veg, recall_wo_veg, TPFNs_wo_veg);
            recall_arr.push_back(recall);
            prec_arr.push_back(precision);
            runtime.push_back(rt);

            double rt_mean =(double)accumulate(runtime.begin(), runtime.end(), 0.0)/runtime.size();
            double recall_mean =(double)accumulate(recall_arr.begin(), recall_arr.end(), 0)/recall_arr.size();
            double prec_mean =(double)accumulate(prec_arr.begin(), prec_arr.end(), 0)/prec_arr.size();
            double f1_score =  2 * (prec_mean * recall_mean) / (prec_mean + recall_mean);
            std::cout << "\033[1;32m Avg P: " << prec_mean << ", " << cal_stdev(prec_arr) <<  " | Avg R: " << recall_mean << ", " << cal_stdev(recall_arr) <<  " | Avg F1: " << f1_score << "\033[0m" << std::endl;
            std::cout << "Avg Time difference = " << rt_mean << "[ms]" << ", " << cal_stdev(runtime) << std::endl;

            calculate_precision_recall_origin(*filtered_cloud_ptr, *post_ground_points, precision_o, recall_o, TPFNs_o);
            recall_o_arr.push_back(recall_o);
            prec_o_arr.push_back(precision_o);

            double recall_o_mean =(double)accumulate(recall_o_arr.begin(), recall_o_arr.end(), 0)/recall_o_arr.size();
            double prec_o_mean =(double)accumulate(prec_o_arr.begin(), prec_o_arr.end(), 0)/prec_o_arr.size();

            // Publish msg
            pcl::PointCloud<PointType> TP;
            pcl::PointCloud<PointType> FP;
            pcl::PointCloud<PointType> FN;
            pcl::PointCloud<PointType> TN;
            //discern_ground(*post_ground_points, TP, FP);
            discern_ground_without_vegetation(*post_ground_points, TP, FP);
            //discern_ground(*final_non_ground_points, FN, TN);
            discern_ground_without_vegetation(*final_non_ground_points, FN, TN);

            pcl::toROSMsg(TP, *msg_TP);
            pcl::toROSMsg(FP, *msg_FP);
            pcl::toROSMsg(FN, *msg_FN);

            msg_TP->header.frame_id = target_frame;
            msg_TP->header.stamp = this->now();

            msg_FP->header.frame_id = target_frame;
            msg_FP->header.stamp = this->now();
            msg_FN->header.frame_id = target_frame;
            msg_FN->header.stamp = this->now();

            pub_tp->publish(*msg_TP);
            pub_fp->publish(*msg_FP);
            pub_fn->publish(*msg_FN);

        }

        if (show_seed_cells){

            auto grid_cells = post_processor->getGridCells();

            visualization_msgs::msg::MarkerArray markers;
            std::vector<Index3D> post_start_cells = post_processor->getSeedCells();

            int marker_count{0};
            for (const auto& start_cell_id : post_start_cells){
                auto start_cell = grid_cells[start_cell_id];

                geometry_msgs::msg::Pose pose;
                pose.position.x = start_cell.centroid.x();
                pose.position.y = start_cell.centroid.y();
                pose.position.z = start_cell.centroid.z();
                
                pose.orientation.x = 0;
                pose.orientation.y = 0;
                pose.orientation.z = 0;
                pose.orientation.w = 1;

                std_msgs::msg::ColorRGBA colour;
                colour.a = 1;
                colour.r = 0;
                colour.g = 0;
                colour.b = 1;

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = target_frame;
                marker.header.stamp = this->now();
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.pose = pose;
                marker.id = marker_count++;
                marker.scale.x = 1;
                marker.scale.y = 1;
                marker.scale.z = 1;
                marker.color = colour;       
                markers.markers.push_back(marker);     
            }
            publisher_start_cells->publish(markers);
        }

        ground_segmentation::msg::GridMap pre_grid_map_msg;
        pre_grid_map_msg.cell_size_x = pre_processor_config.cellSizeX;
        pre_grid_map_msg.cell_size_y = pre_processor_config.cellSizeY;
        pre_grid_map_msg.cell_size_z = pre_processor_config.cellSizeZ;
        pre_grid_map_msg.header.frame_id = this->get_parameter("target_frame").as_string();

        auto pre_grid_cells = pre_processor->getGridCells();

        for (auto& cellPair : pre_grid_cells){
            auto& cell = cellPair.second;

            if (cell.points->size() < 1){
                continue;
            }
            ground_segmentation::msg::GridCell cell_msg;

            cell_msg.position.x = (cell.x * pre_processor_config.cellSizeX) + pre_processor_config.cellSizeX/2;
            cell_msg.position.y = (cell.y * pre_processor_config.cellSizeY) + pre_processor_config.cellSizeY/2;
            cell_msg.position.z = (cell.z * pre_processor_config.cellSizeZ) + pre_processor_config.cellSizeZ/2;

            cell_msg.color.r = 0;
            cell_msg.color.g = 1;
            cell_msg.color.b = 0;
            cell_msg.color.a = 1;

            pre_grid_map_msg.cells.push_back(cell_msg);
        }
        pre_grid_map_publisher->publish(pre_grid_map_msg);

        ground_segmentation::msg::GridMap post_grid_map_msg;
        post_grid_map_msg.cell_size_x = post_processor_config.cellSizeX;
        post_grid_map_msg.cell_size_y = post_processor_config.cellSizeY;
        post_grid_map_msg.cell_size_z = post_processor_config.cellSizeZ;
        post_grid_map_msg.header.frame_id = this->get_parameter("target_frame").as_string();

        auto post_grid_cells = post_processor->getGridCells();

        for (auto& cellPair : post_grid_cells){
            auto& cell = cellPair.second;

            if (cell.points->size() < 1){
                continue;
            }

            ground_segmentation::msg::GridCell cell_msg;

            cell_msg.position.x = (cell.x * post_processor_config.cellSizeX) + post_processor_config.cellSizeX/2;
            cell_msg.position.y = (cell.y * post_processor_config.cellSizeY) + post_processor_config.cellSizeY/2;
            cell_msg.position.z = (cell.z * post_processor_config.cellSizeZ) + post_processor_config.cellSizeZ/2;

            cell_msg.color.r = 0;
            cell_msg.color.g = 1;
            cell_msg.color.b = 0;
            cell_msg.color.a = 1;
            
            post_grid_map_msg.cells.push_back(cell_msg);
        }

        post_grid_map_publisher->publish(post_grid_map_msg);

        post_ground_points->width = post_ground_points->points.size();
        post_ground_points->height = 1;
        post_ground_points->is_dense = true;  

        final_non_ground_points->width = final_non_ground_points->points.size();
        final_non_ground_points->height = 1;
        final_non_ground_points->is_dense = true;  

        pcl::toROSMsg(*filtered_cloud_ptr, *raw_ground_points);
        pcl::toROSMsg(*post_ground_points, *ground_points);
        pcl::toROSMsg(*final_non_ground_points, *obstacle_points);

        ground_points->header.frame_id = target_frame;
        obstacle_points->header.frame_id = target_frame;
        raw_ground_points->header.frame_id = target_frame;

        ground_points->header.stamp = this->now();
        obstacle_points->header.stamp = this->now();
        raw_ground_points->header.stamp = this->now();

        // Publish the message
        publisher_ground_points->publish(*ground_points);
        publisher_obstacle_points->publish(*obstacle_points);
        publisher_raw_ground_points->publish(*raw_ground_points);
   }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<GroundSegmentatioNode>(options));
    rclcpp::shutdown();
    return 0;
}
