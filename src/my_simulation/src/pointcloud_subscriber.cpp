#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <fstream>
#include <filesystem>
#include <string>

/** This subscriber has to be runed alongside the lidar simulation, so it can read data published to /lidar/out topic 
it will save the point cloud data for each timestamp inside a 'pcl' folder in the current directory.
If the folder doesn't exists, it will be created, also, if it exists and have files inside, the folder WILL BE CLEANED
before saving new files.

To run the subscriber:
Inside a ros2 workspace:

-- source the workspace:
$ source install/setup.bash

-- run the subscriber with the desired file format (obj, pcd, ply or xyz). If no format is specified, default is 'obj':
$ ros2 run my_simulation pointcloud_subscriber [file_format]

*/

// Default file format
std::string format = "obj";
bool pcl_dir_cleaned_once = false;

class PointCloudSubscriber : public rclcpp::Node
{
public:
    PointCloudSubscriber() : Node("PCL_sub")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_points", 10,
            std::bind(&PointCloudSubscriber::callback, this, std::placeholders::_1));
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::ofstream file;

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        RCLCPP_INFO(this->get_logger(), "Cloud: width = %d, height = %d", cloud.width, cloud.height);

        if (!std::filesystem::is_directory("cloud_data") || !std::filesystem::exists("cloud_data")) { // Check if cloud_data folder exists
            RCLCPP_INFO(this->get_logger(), "Creating cloud_data directory");
            std::filesystem::create_directory("cloud_data"); // create cloud_data folder
        }
        else if (!pcl_dir_cleaned_once) { // If it exists, clean it up only once
            pcl_dir_cleaned_once = true;
            for (const auto& entry : std::filesystem::directory_iterator("cloud_data"))
                std::filesystem::remove_all(entry.path());
        }

        file.open("cloud_data/pcl_out_time" + std::to_string(msg->header.stamp.sec) + "-" + std::to_string(msg->header.stamp.nanosec) + "." + format);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing.");
            return;
        }

        if (format == "pcd") {
            pcl::io::savePCDFileASCII("cloud_data/pcl_out_time" + std::to_string(msg->header.stamp.sec) + "-" + std::to_string(msg->header.stamp.nanosec) + ".pcd", cloud);
        }
        else if (format == "ply") {
            pcl::io::savePLYFileASCII("cloud_data/pcl_out_time" + std::to_string(msg->header.stamp.sec) + "-" + std::to_string(msg->header.stamp.nanosec) + ".ply", cloud);
        }
        else {
            // If format is 'obj' it will write the header of the obj file before writing the points
            // If it is 'xyz' then this part is skipped
            if (format == "obj") {
                file << "o obj1\n";
            }
            for (const auto& pt : cloud.points)
            {
                file << "v " << pt.x << " " << pt.y << " " << pt.z << "\n";
            }
        }

        std::cout << "Point cloud file created in " << 
            std::filesystem::current_path() / ("cloud_data/pcl_out_time" + std::to_string(msg->header.stamp.sec) + "-" + std::to_string(msg->header.stamp.nanosec) + "." + format) << std::endl;
        
        file.close();
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    if (argc > 2) {
        std::cout << "Too many arguments! The script only accepts 1 argument." << std::endl;
        return 0;
    }
    else if (argc < 2) {
        format = "obj";
        std::cout << "No argument provided. Using default file format: obj" << std::endl;
    }
    else {
        format = argv[1];
        if (format != "obj" && format != "pcd" && format != "xyz" && format != "ply") {
            std::cout << "Invalid file format! Supported formats are 'obj', 'pcd', 'xyz', and 'ply'." << std::endl;
            return 0;
        }
        std::cout << "File format set to: " << format << std::endl;
    }
    

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSubscriber>());
    rclcpp::shutdown();
    return 0;
}