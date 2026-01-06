#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <mutex>
#include <chrono>
#include <set>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

using json = nlohmann::json;
using Server = websocketpp::server<websocketpp::config::asio>;

class WebSocketBridge : public rclcpp::Node {
public:
    WebSocketBridge() : Node("websocket_bridge") {
        // --- Parameters ---
        // 模拟一个序列号
        accid_ = "PF_TRON1_SIM_001";

        // --- Publishers ---
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // --- Subscribers ---
        // 订阅仿真器发布的 IMU 和 Odom 数据
        // 注意：需确认仿真器实际发布的话题名称，通常是 /imu 或 /imu/data
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&WebSocketBridge::imuCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&WebSocketBridge::odomCallback, this, std::placeholders::_1));

        // --- Timers ---
        // 30Hz Timer for publishing cmd_vel (Watchdog logic)
        cmd_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&WebSocketBridge::cmdTimerCallback, this)
        );

        // 1Hz Timer for robot info (模拟心跳)
        info_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&WebSocketBridge::infoTimerCallback, this)
        );

        last_cmd_time_ = this->now();

        // --- WebSocket Server ---
        server_thread_ = std::thread(&WebSocketBridge::runServer, this);

        RCLCPP_INFO(this->get_logger(), "WebSocket Bridge Full-Feature Node Started on port 5000");
    }

    ~WebSocketBridge() {
        if (server_running_) {
            server_.stop();
            if (server_thread_.joinable()) {
                server_thread_.join();
            }
        }
    }

private:
    // --- WebSocket Logic ---

    void runServer() {
        try {
            server_.clear_access_channels(websocketpp::log::alevel::all);
            server_.init_asio();
            server_.set_reuse_addr(true);
            server_.listen(5000);
            server_.start_accept();
            
            server_.set_open_handler(std::bind(&WebSocketBridge::onOpen, this, std::placeholders::_1));
            server_.set_close_handler(std::bind(&WebSocketBridge::onClose, this, std::placeholders::_1));
            server_.set_message_handler(std::bind(&WebSocketBridge::onMessage, this, std::placeholders::_1, std::placeholders::_2));
            
            server_running_ = true;
            RCLCPP_INFO(this->get_logger(), "WebSocket Server listening...");
            server_.run();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "WebSocket Server Error: %s", e.what());
        }
    }

    void onOpen(websocketpp::connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(conn_mutex_);
        connections_.insert(hdl);
        RCLCPP_INFO(this->get_logger(), "Client connected");
    }

    void onClose(websocketpp::connection_hdl hdl) {
        std::lock_guard<std::mutex> lock(conn_mutex_);
        connections_.erase(hdl);
        RCLCPP_INFO(this->get_logger(), "Client disconnected");
    }

    void onMessage(websocketpp::connection_hdl hdl, Server::message_ptr msg) {
        try {
            auto payload = msg->get_payload();
            auto data = json::parse(payload);
            std::string title = data.value("title", "");
            std::string guid = data.value("guid", "");

            if (title == "request_twist" && data.contains("data")) {
                handleTwist(data["data"]);
            } 
            else if (title == "request_stand_mode") {
                // 模拟处理：直接返回成功
                RCLCPP_INFO(this->get_logger(), "Received STAND request");
                sendResponse(hdl, "response_stand_mode", guid, {{"result", "success"}});
            }
            else if (title == "request_walk_mode") {
                RCLCPP_INFO(this->get_logger(), "Received WALK request");
                sendResponse(hdl, "response_walk_mode", guid, {{"result", "success"}});
            }
            else if (title == "request_sitdown") {
                RCLCPP_INFO(this->get_logger(), "Received SIT request");
                sendResponse(hdl, "response_sitdown", guid, {{"result", "success"}});
            }
            else if (title == "request_enable_imu") {
                bool enable = data["data"].value("enable", false);
                imu_enabled_ = enable;
                RCLCPP_INFO(this->get_logger(), "IMU Stream: %s", enable ? "ON" : "OFF");
                sendResponse(hdl, "response_enable_imu", guid, {{"result", "success"}});
            }
            else if (title == "request_enable_odom") {
                bool enable = data["data"].value("enable", false);
                odom_enabled_ = enable;
                RCLCPP_INFO(this->get_logger(), "Odom Stream: %s", enable ? "ON" : "OFF");
                sendResponse(hdl, "response_enable_odom", guid, {{"result", "success"}});
            }
            else {
                // Unknown command
                RCLCPP_WARN(this->get_logger(), "Unknown command: %s", title.c_str());
            }

        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "JSON Parse Error: %s", e.what());
        }
    }

    // --- Command Handlers ---

    void handleTwist(const json& data) {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        current_twist_.linear.x = data.value("x", 0.0);
        current_twist_.linear.y = data.value("y", 0.0);
        current_twist_.angular.z = data.value("z", 0.0);
        last_cmd_time_ = this->now();
    }

    // --- ROS Callbacks & Broadcasting ---

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (!imu_enabled_) return;

        json data;
        // Euler conversion is complex in C++, sending Quat directly as per protocol (optional euler calc omitted)
        // Protocol: quat: [w, x, y, z]
        data["quat"] = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
        data["acc"] = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
        data["gyro"] = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        
        // Mock euler for completeness (all zeros)
        data["euler"] = {0.0, 0.0, 0.0}; 

        broadcastNotification("notify_imu", data);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!odom_enabled_) return;

        json data;
        data["pose_position"] = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
        data["pose_orientation"] = {msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
        data["twist_linear"] = {msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
        data["twist_angular"] = {msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z};

        broadcastNotification("notify_odom", data);
    }

    void cmdTimerCallback() {
        auto now = this->now();
        double watchdog_timeout = 0.5;

        geometry_msgs::msg::Twist msg_to_pub;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            if ((now - last_cmd_time_).seconds() < watchdog_timeout) {
                msg_to_pub = current_twist_;
            }
            // else: msg_to_pub remains zero
        }
        cmd_vel_pub_->publish(msg_to_pub);
    }

    void infoTimerCallback() {
        // Broadcast robot status (heartbeat)
        json data;
        data["accid"] = accid_;
        data["sw_version"] = "sim-1.0.0";
        data["battery"] = 100;
        data["status"] = "WALK"; // Mock status
        data["imu"] = "OK";
        data["motor"] = "OK";
        
        broadcastNotification("notify_robot_info", data);
    }

    // --- Helpers ---

    std::string generate_guid() {
        boost::uuids::random_generator gen;
        boost::uuids::uuid u = gen();
        return boost::uuids::to_string(u);
    }

    void sendResponse(websocketpp::connection_hdl hdl, const std::string& title, const std::string& req_guid, const json& data) {
        json msg;
        msg["accid"] = accid_;
        msg["title"] = title;
        msg["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::system_clock::now().time_since_epoch()).count();
        msg["guid"] = req_guid; // Response must match request GUID
        msg["data"] = data;

        try {
            server_.send(hdl, msg.dump(), websocketpp::frame::opcode::text);
        } catch (...) {}
    }

    void broadcastNotification(const std::string& title, const json& data) {
        json msg;
        msg["accid"] = accid_;
        msg["title"] = title;
        msg["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::system_clock::now().time_since_epoch()).count();
        msg["guid"] = generate_guid();
        msg["data"] = data;

        std::string str = msg.dump();
        
        std::lock_guard<std::mutex> lock(conn_mutex_);
        for (auto hdl : connections_) {
            try {
                server_.send(hdl, str, websocketpp::frame::opcode::text);
            } catch (...) {
                // Handle dead connections lazily or ignore
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::TimerBase::SharedPtr cmd_timer_;
    rclcpp::TimerBase::SharedPtr info_timer_;
    
    Server server_;
    std::thread server_thread_;
    bool server_running_ = false;

    // Connection management
    typedef std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> con_list;
    con_list connections_;
    std::mutex conn_mutex_;

    std::mutex cmd_mutex_;
    geometry_msgs::msg::Twist current_twist_;
    rclcpp::Time last_cmd_time_;

    // State
    std::string accid_;
    std::atomic<bool> imu_enabled_{false};
    std::atomic<bool> odom_enabled_{false};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebSocketBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
