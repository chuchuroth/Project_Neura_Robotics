// Publisher uses loaned messages if available:


class PCPublisher : public rclcpp::Node {
public:
    PCPublisher()
    : Node("pc_publisher")
    {
        rclcpp::QoS qos(10);
        qos.best_effort();
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/pc2", qos);

        // Start background thread to generate data
        prod_thread_ = std::thread([this](){ producerLoop(); });
    }

    ~PCPublisher() {
        running_ = false;
        if (prod_thread_.joinable()) prod_thread_.join();
    }

private:
    void producerLoop() {
        PointCloud cloud(200000); // reserve memory once

        while(rclcpp::ok() && running_) {
            cloud = PointCloud(200000);
            for(int i = 0; i < 1000; ++i)
                cloud.push_back({(float)i, 0, 0});

            // Try zero-copy loaned message
            if(auto loan = pub_->borrow_loaned_message()) {
                loan->header.stamp = now();
                loan->header.frame_id = "lidar";

                loan->width = cloud.size();
                loan->height = 1;
                loan->point_step = sizeof(Point);
                loan->row_step = loan->width * loan->point_step;

                loan->data.resize(loan->row_step);
                std::memcpy(loan->data.data(), cloud_pts_ptr(cloud), loan->row_step);

                pub_->publish(std::move(loan));
            } else {
                // Fallback copy
                pub_->publish(cloud.toROS2());
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(33));
        }
    }

    static const void* cloud_pts_ptr(const PointCloud& pc) {
        return pc.size() ? pc.pts_.data() : nullptr;
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::thread prod_thread_;
    std::atomic<bool> running_{true};
};


/*
Important notes about zero-copy and loaned messages

borrow_loaned_message() is part of rclcpp and will work only when the RMW and middleware support loaned messages (e.g., CycloneDDS/FastDDS with loaning enabled). 
If supported, you fill the borrowed message and publish(std::move(loaned_msg)) to avoid allocations/copies.

Intra-process communication (use_intra_process_comms(true) on NodeOptions) + loaned messages can achieve true zero-copy between publishers and subscribers in the same process.

If loaned messages are unavailable, you must copy into the outbound ROS2 message; minimize copies by organizing memory layout to allow single memcpy (or direct serialization into message buffer).

*/
