// Move Semantics + Zero-Copy ROS2 Pipeline (PointCloud / Image)
// Goal: show how to design a move-enabled PointCloud object, hand it off efficiently and publish with minimal copying. 
// Demonstrates borrow_loaned_message() when supported (zero-copy) and intra-process transport.



// pointcloud_pipeline.hpp
// This code shows how to avoid expensive memory copying using move semantics.
// ROS loaned messages let us share memory instead of copying.

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <cstring>

struct Point {
    float x, y, z;
};

// Custom point cloud class that can be moved but NOT copied
class PointCloud {
public:
    PointCloud() = default;
    explicit PointCloud(size_t reserve_pts) { pts_.reserve(reserve_pts); }

    // Disable copy to prevent expensive buffer copying
    PointCloud(const PointCloud&) = delete;
    PointCloud& operator=(const PointCloud&) = delete;

    // Enable move (cheap / pointer only)
    PointCloud(PointCloud&& other) noexcept : pts_(std::move(other.pts_)) {}
    PointCloud& operator=(PointCloud&& other) noexcept {
        pts_ = std::move(other.pts_);
        return *this;
    }

    void push_back(Point p) { pts_.push_back(p); }
    size_t size() const { return pts_.size(); }

    sensor_msgs::msg::PointCloud2 toROS2() const {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.frame_id = "lidar";
        msg.height = 1;
        msg.width = pts_.size();
        msg.point_step = sizeof(Point);
        msg.row_step = msg.point_step * msg.width;

        // Allocate ROS msg buffer (copy fallback)
        msg.data.resize(msg.row_step);
        std::memcpy(msg.data.data(), pts_.data(), msg.row_step);
        return msg;
    }

private:
    std::vector<Point> pts_;
};
