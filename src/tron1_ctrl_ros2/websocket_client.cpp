#include <iostream>
#include <atomic>
#include <string>
#include <thread>
#include <chrono>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio.hpp> 
#include <nlohmann/json.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

using json = nlohmann::json;
using websocketpp::client;
using websocketpp::connection_hdl;

// Replace this ACCID value with your robot's actual serial number (SN)
static std::string ACCID = "";

// WebSocket client instance
static client<websocketpp::config::asio> ws_client;

// Atomic flag for graceful exit
static std::atomic<bool> should_exit(false);

// Connection handle for sending messages
static connection_hdl current_hdl;

// Generate dynamic GUID
static std::string generate_guid() {
    boost::uuids::random_generator gen;
    boost::uuids::uuid u = gen();
    return boost::uuids::to_string(u);
}

// Send WebSocket request with title and data
static void send_request(const std::string& title, const json& data = json::object()) {
    json message;
    
    // Adding necessary fields to the message
    message["accid"] = ACCID;
    message["title"] = title;
    message["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::system_clock::now().time_since_epoch()).count();
    message["guid"] = generate_guid();
    message["data"] = data;

    std::string message_str = message.dump();
    
    // Send the message through WebSocket
    ws_client.send(current_hdl, message_str, websocketpp::frame::opcode::text);
}

// Handle user commands
static void handle_commands() {
    while (!should_exit) {
        std::string command;
        std::cout << "Enter command ('stand', 'walk', 'twist', 'sit', 'stair', 'stop', 'imu') or 'exit' to quit:" << std::endl;
        std::getline(std::cin, command);  // Read user input

        if (command == "exit") {
            should_exit = true;  // Exit flag to stop the loop
            break;
        } else if (command == "stand") {
            send_request("request_stand_mode");  // Send stand mode request
        } else if (command == "walk") {
            send_request("request_walk_mode");  // Send walk mode request
        } else if (command == "twist") {
            float x, y, z;
            std::cout << "Enter x, y, z values:" << std::endl;
            std::cin >> x >> y >> z;  // Get twist values from user
            send_request("request_twist", {{"x", x}, {"y", y}, {"z", z}});
        } else if (command == "sit") {
            send_request("request_sitdown");  // Send sit down request
        } else if (command == "stair") {
            std::string enable;
            std::cout << "Enable stair mode (true/false):" << std::endl;
            std::cin >> enable;  // Get stair mode enable flag from user
            send_request("request_stair_mode", {{"enable", enable == "true" ? true : false}});
        } else if (command == "stop") {
            send_request("request_emgy_stop");  // Send emergency stop request
        } else if (command == "imu") {
            std::string enable;
            std::cout << "Enable IMU (true/false):" << std::endl;
            std::cin >> enable;  // Get IMU enable flag from user
            send_request("request_enable_imu", {{"enable", enable == "true" ? true : false}});
        }
    }
}

// WebSocket open callback
static void on_open(connection_hdl hdl) {
    std::cout << "Connected!" << std::endl;
    
    // Save connection handle for sending messages later
    current_hdl = hdl;

    // Start handling commands in a separate thread
    std::thread(handle_commands).detach();
}

// WebSocket message callback
static void on_message(connection_hdl hdl, client<websocketpp::config::asio>::message_ptr msg) {
    // Parse JSON data from message payload
    json data = json::parse(msg->get_payload());
          
    // Extract 'accid' field if present
    if (data.contains("accid") && data["accid"].is_string() && ACCID.empty()) {
        ACCID = data["accid"].get<std::string>();
    }
    
    std::cout << "Received: " << msg->get_payload() << std::endl;  // Print received message
}

// WebSocket close callback
static void on_close(connection_hdl hdl) {
    std::cout << "Connection closed." << std::endl;
}

// Close WebSocket connection
static void close_connection(connection_hdl hdl) {
    ws_client.close(hdl, websocketpp::close::status::normal, "Normal closure");  // Close connection normally
}

int main() {
    ws_client.init_asio();  // Initialize ASIO for WebSocket client
    
    // Set WebSocket event handlers
    ws_client.set_open_handler(&on_open);  // Set open handler
    ws_client.set_message_handler(&on_message);  // Set message handler
    ws_client.set_close_handler(&on_close);  // Set close handler

    std::string server_uri = "ws://127.0.0.1:5000";  // WebSocket server URI

    websocketpp::lib::error_code ec;
    client<websocketpp::config::asio>::connection_ptr con = ws_client.get_connection(server_uri, ec);  // Get connection pointer

    if (ec) {
        std::cout << "Error: " << ec.message() << std::endl;
        return 1;  // Exit if connection error occurs
    }

    connection_hdl hdl = con->get_handle();  // Get connection handle
    ws_client.connect(con);  // Connect to server
    std::cout << "Press Ctrl+C to exit." << std::endl;
    
    // Run the WebSocket client loop
    ws_client.run();

    return 0;
}
