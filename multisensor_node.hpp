// 边写边解释，所以要记住两样东西，一样是代码，一样是那句解释的话

// IMU数据frequenz是xxHz，相机和lidar是xxhz
// 传感器数据QoS都是best effort


// ROS2 Executor + CallbackGroups for Multi-Sensor Fusion
// Goal: isolate callbacks (camera heavy inference, lidar processing, control) to avoid contention; 
// use MutuallyExclusive for control to ensure single-threaded control code; 
// Reentrant or separate groups for sensor processing; 
// use weak_ptr capture in callbacks to avoid lifecycle leaks.



// multisensor_node.hpp
// This node subscribes to LiDAR, Camera, and IMU data, each in its own callback group.
// A timer runs the control loop. Uses multi-threaded executor for parallelism.

#pragma once // 只能引用一次，这个可以省略
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>

class MultiSensorNode : public rclcpp::Node {
public:
    MultiSensorNode()
    : Node("multisensor_node")
    {
        // Create callback groups controlling concurrency
        // - sensor_cb_group: LiDAR + IMU (exclusive = one at a time)
        // - heavy_cb_group: Camera processing (can run in parallel)
        // - control_cb_group: control loop must be exclusive
        sensor_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        heavy_cb_group_  = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        control_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // QoS settings — we want fast sensor streaming, so best effort
        rclcpp::QoS qos(10);
        qos.best_effort();

        // Subscription options to assign callback groups
        auto sensor_opts = rclcpp::SubscriptionOptions();
        sensor_opts.callback_group = sensor_cb_group_;

        auto heavy_opts = rclcpp::SubscriptionOptions();
        heavy_opts.callback_group = heavy_cb_group_;

        auto control_opts = rclcpp::SubscriptionOptions();
        control_opts.callback_group = control_cb_group_;

        auto weak_this = weak_from_this();

        // LiDAR subscriber
        lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points", qos,
            [weak_this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                if(auto self = weak_this.lock()) self->onLidar(std::move(msg));
            },   // At the system level, ROS2 nodes use **smart pointers** and **lambda callbacks** to connect everything.
            sensor_opts
        );

        // Camera subscriber (heavy processing)
        camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", qos,
            [weak_this](sensor_msgs::msg::Image::SharedPtr msg) {
                if(auto self = weak_this.lock()) self->onImage(std::move(msg));
            },
            heavy_opts
        );

        // IMU subscriber
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", qos,
            [weak_this](sensor_msgs::msg::Imu::SharedPtr msg) {
                if(auto self = weak_this.lock()) self->onImu(std::move(msg));
            },
            sensor_opts
        );

        // Control timer (100Hz)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            [weak_this]() {
                if(auto self = weak_this.lock()) self->controlLoop();
            },
            control_cb_group_
        );

        RCLCPP_INFO(get_logger(), "MultiSensorNode started");
    }

    // Minimal work inside sensor callbacks — don't block ROS threads
    void onLidar(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        (void)msg;
        // add to processing queue or immediately process
    }

    void onImage(sensor_msgs::msg::Image::SharedPtr msg) {
        (void)msg;
        // run inference or pass to GPU
    }

    void onImu(sensor_msgs::msg::Imu::SharedPtr msg) {
        (void)msg;
        // IMU buffer for estimation
    }

    void controlLoop() {
        // Safety-critical control logic, avoid long computation
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::CallbackGroup::SharedPtr sensor_cb_group_, heavy_cb_group_, control_cb_group_;
};

// Program entry — multi-thread executor runs callbacks in parallel
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiSensorNode>();

    // 4 worker threads — tune based on CPU
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
    exec.add_node(node);
    exec.spin();
}


/*
Notes & best practices

Create separate callback groups to control concurrency; heavy sensor processing uses Reentrant or its own thread pool.

Keep each callback minimal: push to a lock-free queue or hand off to worker thread/pool to avoid blocking executor threads.

Use weak_from_this() to avoid circular shared_ptr retention between node and callbacks.

Use QoS suited to sensor type (best-effort for high-bandwidth sensors).

*/
