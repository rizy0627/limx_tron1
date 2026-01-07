#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "tron1_ctrl_ros2/msg/robot_status.hpp"
#include "tron1_ctrl_ros2/srv/set_base_height.hpp"
#include "tron1_ctrl_ros2/srv/set_light_effect.hpp"

#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <nlohmann/json.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <future>
#include <map>

using json = nlohmann::json;
using websocketpp::client;
using websocketpp::connection_hdl;

typedef websocketpp::client<websocketpp::config::asio_client> client_type;

class Tron1Node : public rclcpp::Node {
public:
    Tron1Node() : Node("tron1_node") {
        // Parameters
        this->declare_parameter<std::string>("server_uri", "ws://10.192.1.2:5000");
        this->get_parameter("server_uri", server_uri_);
        
        this->declare_parameter<std::string>("accid", "WF_TRON1A_519"); // Default empty, auto-discover
        this->get_parameter("accid", accid_);

        this->declare_parameter<double>("max_linear_x", 1.0); // m/s corresponding to ratio 1.0
        this->get_parameter("max_linear_x", max_linear_x_);

        this->declare_parameter<double>("max_linear_y", 1.0); // m/s corresponding to ratio 1.0
        this->get_parameter("max_linear_y", max_linear_y_);

        this->declare_parameter<double>("max_angular_z", 1.0); // rad/s corresponding to ratio 1.0
        this->get_parameter("max_angular_z", max_angular_z_);

        // Publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        status_pub_ = this->create_publisher<tron1_ctrl_ros2::msg::RobotStatus>("robot_status", 10);

        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&Tron1Node::cmdVelCallback, this, std::placeholders::_1));

        // Services
        srv_stand_ = this->create_service<std_srvs::srv::Trigger>("stand_mode", std::bind(&Tron1Node::cbStandMode, this, std::placeholders::_1, std::placeholders::_2));
        srv_walk_ = this->create_service<std_srvs::srv::Trigger>("walk_mode", std::bind(&Tron1Node::cbWalkMode, this, std::placeholders::_1, std::placeholders::_2));
        srv_sit_ = this->create_service<std_srvs::srv::Trigger>("sit_down", std::bind(&Tron1Node::cbSitDown, this, std::placeholders::_1, std::placeholders::_2));
        srv_recover_ = this->create_service<std_srvs::srv::Trigger>("recover", std::bind(&Tron1Node::cbRecover, this, std::placeholders::_1, std::placeholders::_2));
        srv_stop_ = this->create_service<std_srvs::srv::Trigger>("emergency_stop", std::bind(&Tron1Node::cbEmgyStop, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_stair_ = this->create_service<std_srvs::srv::SetBool>("stair_mode", std::bind(&Tron1Node::cbStairMode, this, std::placeholders::_1, std::placeholders::_2));
        srv_marktime_ = this->create_service<std_srvs::srv::SetBool>("marktime_mode", std::bind(&Tron1Node::cbMarktimeMode, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_odom_ = this->create_service<std_srvs::srv::SetBool>("enable_odom", std::bind(&Tron1Node::cbEnableOdom, this, std::placeholders::_1, std::placeholders::_2));
        srv_enable_imu_ = this->create_service<std_srvs::srv::SetBool>("enable_imu", std::bind(&Tron1Node::cbEnableImu, this, std::placeholders::_1, std::placeholders::_2));
        
        srv_base_height_ = this->create_service<tron1_ctrl_ros2::srv::SetBaseHeight>("set_base_height", std::bind(&Tron1Node::cbSetBaseHeight, this, std::placeholders::_1, std::placeholders::_2));
        srv_light_effect_ = this->create_service<tron1_ctrl_ros2::srv::SetLightEffect>("set_light_effect", std::bind(&Tron1Node::cbSetLightEffect, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Connecting to %s", server_uri_.c_str());

        // Start WebSocket thread
        ws_thread_ = std::thread(&Tron1Node::wsThread, this);
    }

    ~Tron1Node() {
        should_exit_ = true;
        if (connected_) {
             ws_client_.close(hdl_, websocketpp::close::status::normal, "Shutdown");
        }
        ws_client_.stop();
        if (ws_thread_.joinable()) {
            ws_thread_.join();
        }
    }

private:
    // --- Service Callbacks ---
    // 蹲起模式
    void cbStandMode(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handleTriggerService("request_stand_mode", response);
    }
    // 行走模式
    void cbWalkMode(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handleTriggerService("request_walk_mode", response);
    }
    // 蹲下
    void cbSitDown(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handleTriggerService("request_sitdown", response);
    }
    // 摔倒恢复
    void cbRecover(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handleTriggerService("request_recover", response);
    }
    // 紧急停止
    void cbEmgyStop(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handleTriggerService("request_emgy_stop", response);
    }
    // 楼梯模式
    void cbStairMode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        handleSetBoolService("request_stair_mode", request->data, response);
    }
    // 踏步模式（轮足是否可用，存疑？）
    void cbMarktimeMode(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        handleSetBoolService("request_marktime_mode", request->data, response);
    }
    // 启用里程计
    void cbEnableOdom(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        handleSetBoolService("request_enable_odom", request->data, response);
    }
    // 启用IMU
    void cbEnableImu(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        handleSetBoolService("request_enable_imu", request->data, response);
    }
    // 调整高度信息（1：上升5cm; -1：下降5cm --> -15cm -- 15cm）
    void cbSetBaseHeight(const std::shared_ptr<tron1_ctrl_ros2::srv::SetBaseHeight::Request> request, std::shared_ptr<tron1_ctrl_ros2::srv::SetBaseHeight::Response> response) {
        json resp = sendSyncRequest("request_base_height", {{"direction", request->direction}});
        fillResponse(resp, response->success, response->message);
    }
    // 设置灯效
    void cbSetLightEffect(const std::shared_ptr<tron1_ctrl_ros2::srv::SetLightEffect::Request> request, std::shared_ptr<tron1_ctrl_ros2::srv::SetLightEffect::Response> response) {
        json resp = sendSyncRequest("request_light_effect", {{"effect", request->effect_id}});
        fillResponse(resp, response->success, response->message);
    }

    // Helper for Trigger services
    void handleTriggerService(const std::string& title, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        json resp = sendSyncRequest(title);
        fillResponse(resp, response->success, response->message);
    }

    // Helper for SetBool services
    void handleSetBoolService(const std::string& title, bool enable, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        json resp = sendSyncRequest(title, {{"enable", enable}});
        fillResponse(resp, response->success, response->message);
    }

    // Helper to fill response fields from JSON
    void fillResponse(const json& resp, bool& success, std::string& message) {
        if (resp.is_null()) {
            success = false;
            message = "Timeout or Not Connected";
            return;
        }
        
        // Check "data" -> "result"
        if (resp.contains("data") && resp["data"].contains("result")) {
            std::string result = resp["data"]["result"].get<std::string>();
            if (result.find("success") != std::string::npos) {
                success = true;
                message = result;
            } else {
                success = false;
                message = result;
            }
        } else {
            success = false;
            message = "Invalid response format";
        }
    }

    // --- WebSocket Logic ---

    void wsThread() {
        try {
            ws_client_.clear_access_channels(websocketpp::log::alevel::all);
            ws_client_.set_access_channels(websocketpp::log::alevel::connect);
            ws_client_.set_access_channels(websocketpp::log::alevel::disconnect);
            ws_client_.set_access_channels(websocketpp::log::alevel::app);

            ws_client_.init_asio();
            ws_client_.set_open_handler(bind(&Tron1Node::onOpen, this, std::placeholders::_1));
            ws_client_.set_message_handler(bind(&Tron1Node::onMessage, this, std::placeholders::_1, std::placeholders::_2));
            ws_client_.set_close_handler(bind(&Tron1Node::onClose, this, std::placeholders::_1));
            ws_client_.set_fail_handler(bind(&Tron1Node::onFail, this, std::placeholders::_1));

            websocketpp::lib::error_code ec;
            client_type::connection_ptr con = ws_client_.get_connection(server_uri_, ec);
            if (ec) {
                RCLCPP_ERROR(this->get_logger(), "Could not create connection: %s", ec.message().c_str());
                return;
            }

            ws_client_.connect(con);
            ws_client_.run();
        } catch (websocketpp::exception const & e) {
             RCLCPP_ERROR(this->get_logger(), "WebSocket exception: %s", e.what());
        } catch (std::exception const & e) {
             RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    void onOpen(connection_hdl hdl) {
        RCLCPP_INFO(this->get_logger(), "WebSocket Connected");
        {
            std::lock_guard<std::mutex> lock(conn_mutex_);
            hdl_ = hdl;
            connected_ = true;
        }
        
        // Enable IMU
        sendRequest("request_enable_imu", {{"enable", true}});
        // Try enabling Odom (might fail if not supported, but harmless)
        sendRequest("request_enable_odom", {{"enable", true}});
    }

    void onClose(connection_hdl hdl) {
        (void)hdl;
        RCLCPP_INFO(this->get_logger(), "WebSocket Closed");
        {
            std::lock_guard<std::mutex> lock(conn_mutex_);
            connected_ = false;
        }
        // Fail all pending promises
        std::lock_guard<std::mutex> lock(pending_mutex_);
        for (auto& pair : pending_requests_) {
            try {
                pair.second->set_value(json(nullptr)); 
            } catch (...) {}
        }
        pending_requests_.clear();
    }

    void onFail(connection_hdl hdl) {
        (void)hdl;
        RCLCPP_ERROR(this->get_logger(), "WebSocket Connection Failed");
        {
            std::lock_guard<std::mutex> lock(conn_mutex_);
            connected_ = false;
        }
    }

    void onMessage(connection_hdl hdl, client_type::message_ptr msg) {
        (void)hdl;
        try {
            json data = json::parse(msg->get_payload());
            
            // Auto-discover ACCID
             if (data.contains("accid") && data["accid"].is_string()) {
                std::string received_accid = data["accid"].get<std::string>();
                if (accid_.empty()) {
                    accid_ = received_accid;
                    RCLCPP_INFO(this->get_logger(), "Auto-detected ACCID: %s", accid_.c_str());
                }
            }

            std::string title = data.value("title", "");
            std::string guid = data.value("guid", "");

            // Handle Response (Sync Request)
            if (title.rfind("response_", 0) == 0 && !guid.empty()) {
                std::lock_guard<std::mutex> lock(pending_mutex_);
                auto it = pending_requests_.find(guid);
                if (it != pending_requests_.end()) {
                    try {
                        it->second->set_value(data);
                    } catch (...) {}
                    pending_requests_.erase(it);
                    return; // Handled
                }
            }

            // Handle Notifications
            if (title == "notify_imu") {
                handleImu(data["data"]);
            } else if (title == "notify_odom") {
                handleOdom(data["data"]);
            } else if (title == "notify_robot_info") {
                handleRobotInfo(data["data"]);
            } else if (title == "notify_invalid_request") {
                RCLCPP_WARN(this->get_logger(), "Invalid Request Notification: %s", data["data"].dump().c_str());
            } else if (title.rfind("notify_", 0) == 0) {
                // Generic logging for other notifications
                RCLCPP_INFO(this->get_logger(), "Notification [%s]: %s", title.c_str(), data["data"].dump().c_str());
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
        }
    }

    void handleImu(const json& data) {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "imu_link";

        if (data.contains("quat") && data["quat"].size() == 4) {
            imu_msg.orientation.w = data["quat"][0];
            imu_msg.orientation.x = data["quat"][1];
            imu_msg.orientation.y = data["quat"][2];
            imu_msg.orientation.z = data["quat"][3];
        }
        
        if (data.contains("acc") && data["acc"].size() == 3) {
            imu_msg.linear_acceleration.x = data["acc"][0];
            imu_msg.linear_acceleration.y = data["acc"][1];
            imu_msg.linear_acceleration.z = data["acc"][2];
        }

        if (data.contains("gyro") && data["gyro"].size() == 3) {
            imu_msg.angular_velocity.x = data["gyro"][0];
            imu_msg.angular_velocity.y = data["gyro"][1];
            imu_msg.angular_velocity.z = data["gyro"][2];
        }

        imu_pub_->publish(imu_msg);
    }

    void handleOdom(const json& data) {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        if (data.contains("pose_position") && data["pose_position"].size() == 3) {
            odom_msg.pose.pose.position.x = data["pose_position"][0];
            odom_msg.pose.pose.position.y = data["pose_position"][1];
            odom_msg.pose.pose.position.z = data["pose_position"][2];
        }

        if (data.contains("pose_orientation") && data["pose_orientation"].size() == 4) {
            odom_msg.pose.pose.orientation.x = data["pose_orientation"][0];
            odom_msg.pose.pose.orientation.y = data["pose_orientation"][1];
            odom_msg.pose.pose.orientation.z = data["pose_orientation"][2];
            odom_msg.pose.pose.orientation.w = data["pose_orientation"][3];
        }

        if (data.contains("twist_linear") && data["twist_linear"].size() == 3) {
            odom_msg.twist.twist.linear.x = data["twist_linear"][0];
            odom_msg.twist.twist.linear.y = data["twist_linear"][1];
            odom_msg.twist.twist.linear.z = data["twist_linear"][2];
        }

        if (data.contains("twist_angular") && data["twist_angular"].size() == 3) {
            odom_msg.twist.twist.angular.x = data["twist_angular"][0];
            odom_msg.twist.twist.angular.y = data["twist_angular"][1];
            odom_msg.twist.twist.angular.z = data["twist_angular"][2];
        }

        odom_pub_->publish(odom_msg);
    }

    void handleRobotInfo(const json& data) {
        // "accid": "PF_TRON1A_075",
        // "sw_version": "robot-tron1-2.0.10.20241111103012",
        // "imu": "OK",    # 机器人IMU诊断信息
        // "camera": "OK", # 机器人相机诊断信息
        // "motor": "OK",  # 机器人电机诊断信息
        // "battery": 95,  # 机器人电量
        // "status": "WALK" # 机器人运行模式

        auto msg = tron1_ctrl_ros2::msg::RobotStatus();
        msg.header.stamp = this->now();
        
        if (data.contains("accid")) msg.accid = data["accid"].get<std::string>();
        if (data.contains("sw_version")) msg.sw_version = data["sw_version"].get<std::string>();
        if (data.contains("imu")) msg.imu_status = data["imu"].get<std::string>();
        if (data.contains("camera")) msg.camera_status = data["camera"].get<std::string>();
        if (data.contains("motor")) msg.motor_status = data["motor"].get<std::string>();
        if (data.contains("battery")) msg.battery = data["battery"].get<std::string>();
        if (data.contains("status")) msg.status = data["status"].get<std::string>();

        status_pub_->publish(msg);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!connected_ || accid_.empty()) return;

        // Convert cmd_vel to ratios [-1, 1]
        double x_ratio = msg->linear.x / max_linear_x_;
        double y_ratio = msg->linear.y / max_linear_y_;
        double z_ratio = msg->angular.z / max_angular_z_;

        // Clamp
        x_ratio = std::max(-1.0, std::min(1.0, x_ratio));
        y_ratio = std::max(-1.0, std::min(1.0, y_ratio));
        z_ratio = std::max(-1.0, std::min(1.0, z_ratio));

        sendRequest("request_twist", {
            {"x", x_ratio},
            {"y", y_ratio},
            {"z", z_ratio}
        });
    }

    // Fire and forget (used by cmd_vel and internal initial setup)
    void sendRequest(const std::string& title, const json& data = json::object()) {
        sendSyncRequest(title, data, 0); // 0 timeout means don't wait
    }

    // Send request and wait for response (used by Services)
    // timeout_ms = 0 means don't wait, return null immediately
    // default timeout 3000ms
    json sendSyncRequest(const std::string& title, const json& data = json::object(), int timeout_ms = 3000) {
        std::string guid = generate_guid();
        
        json message;
        message["accid"] = accid_;
        message["title"] = title;
        message["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    std::chrono::system_clock::now().time_since_epoch()).count();
        message["guid"] = guid;
        message["data"] = data;

        std::string message_str = message.dump();
        
        std::shared_ptr<std::promise<json>> promise;
        std::future<json> future;

        if (timeout_ms > 0) {
            promise = std::make_shared<std::promise<json>>();
            future = promise->get_future();
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending_requests_[guid] = promise;
        }

        {
            std::lock_guard<std::mutex> lock(conn_mutex_);
            if (!connected_) return json(nullptr);
            websocketpp::lib::error_code ec;
            ws_client_.send(hdl_, message_str, websocketpp::frame::opcode::text, ec);
            if (ec) {
                RCLCPP_ERROR(this->get_logger(), "Send failed: %s", ec.message().c_str());
                if (timeout_ms > 0) {
                    std::lock_guard<std::mutex> lock(pending_mutex_);
                    pending_requests_.erase(guid);
                }
                return json(nullptr);
            }
        }

        if (timeout_ms > 0) {
            if (future.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::ready) {
                return future.get();
            } else {
                RCLCPP_WARN(this->get_logger(), "Request %s timed out", title.c_str());
                std::lock_guard<std::mutex> lock(pending_mutex_);
                pending_requests_.erase(guid);
                return json(nullptr);
            }
        }

        return json(nullptr);
    }

    std::string generate_guid() {
        boost::uuids::random_generator gen;
        boost::uuids::uuid u = gen();
        return boost::uuids::to_string(u);
    }

    client_type ws_client_;
    std::thread ws_thread_;
    connection_hdl hdl_;
    bool connected_ = false;
    std::mutex conn_mutex_;
    
    std::mutex pending_mutex_;
    std::map<std::string, std::shared_ptr<std::promise<json>>> pending_requests_;

    std::string server_uri_;
    std::string accid_;
    double max_linear_x_;
    double max_linear_y_;
    double max_angular_z_;
    std::atomic<bool> should_exit_{false};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<tron1_ctrl_ros2::msg::RobotStatus>::SharedPtr status_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stand_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_walk_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_sit_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_recover_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stop_;
    
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_stair_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_marktime_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_odom_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_enable_imu_;

    rclcpp::Service<tron1_ctrl_ros2::srv::SetBaseHeight>::SharedPtr srv_base_height_;
    rclcpp::Service<tron1_ctrl_ros2::srv::SetLightEffect>::SharedPtr srv_light_effect_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tron1Node>());
    rclcpp::shutdown();
    return 0;
}
