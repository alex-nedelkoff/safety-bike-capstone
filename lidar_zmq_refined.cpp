#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <signal.h>
#include <zmq.hpp>
#include <sstream>
#include <map>
#include <cmath>
#include <jsoncpp/json/json.h> // or your preferred JSON library
#include "sl_lidar_driver.h"
#include <iomanip>  // Add at top with other includes
#include <ctime>    // Add at top with other includes

using namespace sl;
using namespace std;

// Set the LIDAR device serial port
#define SERIAL_PORT "/dev/ttyUSB0"
#define SERIAL_BAUDRATE 460800  // Standard baudrate for RPLidar C1
#define ZMQ_PORT_PUB "5556"     // Port for LIDAR publishing
#define ZMQ_PORT_SUB "5555"     // Port for object detection subscription
#define ZMQ_PORT_OBJ "5557"    // Port for object detections

// Maximum angle difference to consider a match (in degrees)
#define MAX_ANGLE_DIFF 5.0

// Debug mode for angle calculations
#define DEBUG_ANGLES true

// Global variables for cleanup
ILidarDriver* g_drv = nullptr;
IChannel* g_channel = nullptr;
zmq::context_t* g_context = nullptr;
zmq::socket_t* g_publisher = nullptr;
zmq::socket_t* g_subscriber = nullptr;
zmq::socket_t* g_corr_publisher = nullptr;  // New publisher for correlated data
bool g_running = true;  // Control flag for main loop

// JSON array to store correlated detections
Json::Value correlatedDets(Json::arrayValue);

// Signal handler for Ctrl+C
void signalHandler(int signum) {
    cout << "\nStopping LiDAR and cleaning up..." << endl;
    g_running = false;  // Signal the main loop to stop
}

// Function to convert LiDAR's raw angle to degrees
float convertRawAngleToDegrees(float raw_angle) {
    // RPLidar's angle is clockwise from the front
    // We want counterclockwise from the front to match camera
    // Also ensure the range is -180 to +180
    float angle = -raw_angle;  // Flip direction
    
    // Normalize to -180 to +180
    while (angle <= -180.0f) angle += 360.0f;
    while (angle > 180.0f) angle -= 360.0f;
    
    return angle;
}

// Add this helper function before main()
string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%H:%M:%S")
       << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

int main(int argc, const char *argv[]) {
    // Set up signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Initialize ZMQ
    try {
        cout << "Initializing ZMQ..." << endl;
        g_context = new zmq::context_t(1);

        // Publisher for LIDAR data
        g_publisher = new zmq::socket_t(*g_context, ZMQ_PUB);
        try {
            string address_pub = "tcp://*:" + string(ZMQ_PORT_PUB);
            cout << "Binding publisher to " << address_pub << endl;
            g_publisher->bind(address_pub);
            cout << "ZMQ publisher successfully bound to " << address_pub << endl;
        } catch (const zmq::error_t& e) {
            cerr << "Failed to bind LIDAR publisher: " << e.what() << endl;
            return -1;
        }

        // Publisher for object detections
        g_corr_publisher = new zmq::socket_t(*g_context, ZMQ_PUB);
        try {
            string address_obj = "tcp://*:" + string(ZMQ_PORT_OBJ);
            cout << "Binding object publisher to " << address_obj << endl;
            g_corr_publisher->bind(address_obj);
            cout << "ZMQ object publisher successfully bound to " << address_obj << endl;
        } catch (const zmq::error_t& e) {
            cerr << "Failed to bind object publisher: " << e.what() << endl;
            return -1;
        }

        // Subscriber for object detection data
        g_subscriber = new zmq::socket_t(*g_context, ZMQ_SUB);
        try {
            string address_sub = "tcp://localhost:" + string(ZMQ_PORT_SUB);
            cout << "Connecting subscriber to " << address_sub << endl;
            g_subscriber->connect(address_sub);
            g_subscriber->setsockopt(ZMQ_SUBSCRIBE, "", 0);
            cout << "Successfully connected subscriber to " << address_sub << endl;
        } catch (const zmq::error_t& e) {
            cerr << "Failed to connect subscriber: " << e.what() << endl;
            return -1;
        }

    } catch (const zmq::error_t& e) {
        cerr << "Failed to initialize ZMQ: " << e.what() << endl;
        return -1;
    }

    // Create a communication channel instance
    Result<IChannel*> channel = createSerialPortChannel(SERIAL_PORT, SERIAL_BAUDRATE);
    if (!channel) {
        cerr << "Failed to create serial port channel" << endl;
        return -1;
    }
    g_channel = *channel;

    // Create a LIDAR driver instance
    Result<ILidarDriver*> drv = createLidarDriver();
    if (!drv) {
        cerr << "Failed to create RPLidar driver" << endl;
        delete *channel;
        return -1;
    }
    g_drv = *drv;

    // Connect to RPLidar
    if (SL_IS_FAIL((*drv)->connect(*channel))) {
        cerr << "Failed to connect to RPLidar on " << SERIAL_PORT << endl;
        delete *drv;
        delete *channel;
        return -1;
    }

    // Get device info
    sl_lidar_response_device_info_t devinfo;
    if (SL_IS_FAIL((*drv)->getDeviceInfo(devinfo))) {
        cerr << "Failed to get device info" << endl;
        delete *drv;
        delete *channel;
        return -1;
    }

    cout << "RPLidar connected. Model: " << (int)devinfo.model
         << " Firmware: " << (int)devinfo.firmware_version
         << " Hardware: " << (int)devinfo.hardware_version << endl;

    // Start scanning
    (*drv)->startScan(0, 1);

    cout << "Scanning started. Press Ctrl+C to stop." << endl;
    cout << "Only front 180° (-90° to +90°) will be published." << endl;

    int message_count = 0;

    while (g_running) {
        //---------------------------------------------------------------------
        // 1) Grab latest LIDAR data
        //---------------------------------------------------------------------
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        if (SL_IS_FAIL((*drv)->grabScanDataHq(nodes, count))) {
            cerr << "Failed to get scan data" << endl;
            break;
        }
        (*drv)->ascendScanData(nodes, count);

        // Build a map of angle -> distance for the front 180
        map<float, float> lidarData;
        for (size_t i = 0; i < count; i++) {
            // Convert RPLidar's angle (in q14 fixed point) to degrees
            float rawAngle = (nodes[i].angle_z_q14 * 360.0f) / (1 << 14);
            float angle = convertRawAngleToDegrees(rawAngle);
            float distance = nodes[i].dist_mm_q2 / 4.0f; // distance in mm

            // We'll only store angles between -90 and +90
            if (angle >= -90.0f && angle <= 90.0f && distance > 0) {
                lidarData[angle] = distance;
            }
        }

        //---------------------------------------------------------------------
        // 2) Publish all LiDAR data (front 180) on port 5556 (as text)
        //---------------------------------------------------------------------
        {
            stringstream ss;
            ss << "LIDAR_DATA ";
            for (auto &kv : lidarData) {
                float angle = kv.first;
                float dist  = kv.second;
                ss << angle << "," << dist << ";";
            }
            try {
                string msg = ss.str();
                zmq::message_t message(msg.size());
                memcpy(message.data(), msg.c_str(), msg.size());
                g_publisher->send(message, zmq::send_flags::none);
                message_count++;
                
                // Print every 3 seconds (assuming ~10Hz scan rate)
                if (message_count % 30 == 0) {
                    cout << "\n\033[1;32m" << get_timestamp() << " [PORT 5556 - LIDAR]\033[0m" << endl;
                    cout << "┌──────────────────────────────────────────────────────┐" << endl;
                    cout << "│ Message count: " << std::setw(6) << message_count << "                                  │" << endl;
                    cout << "│ Points: " << std::setw(6) << lidarData.size() << "                                     │" << endl;
                    cout << "│ Sample points:                                        │" << endl;
                    int count = 0;
                    for (auto &kv : lidarData) {
                        if (count++ < 3) {
                            cout << "│   • " << std::fixed << std::setprecision(1) 
                                 << std::setw(6) << kv.first << "°, " 
                                 << std::setw(5) << kv.second/1000.0f << "m" 
                                 << "                               │" << endl;
                        }
                    }
                    cout << "└──────────────────────────────────────────────────────┘" << endl;
                    cout.flush();
                }
            } catch (const zmq::error_t& e) {
                cerr << "Failed to send ZMQ message: " << e.what() << endl;
            }
        }

        //---------------------------------------------------------------------
        // 3) Check (non-blocking) if there's a detection message to correlate
        //---------------------------------------------------------------------
        {
            zmq::message_t detectionMsg;
            auto result = g_subscriber->recv(detectionMsg, zmq::recv_flags::dontwait);
            if (result) {
                // We have some detection data
                string detStr(static_cast<char*>(detectionMsg.data()), detectionMsg.size());
                
                Json::Value root;
                Json::Reader reader;
                if (!reader.parse(detStr, root, false)) {
                    cerr << "JSON parse error for detection message" << endl;
                } else {
                    if (root.isMember("detections") && root["detections"].isArray()) {
                        const Json::Value& detArray = root["detections"];
                        for (unsigned int i = 0; i < detArray.size(); i++) {
                            float angleCam = 0.0f;
                            string label = "";
                            float confidence = 0.0f;
                            float area = 0.0f;  // Add area variable

                            if (detArray[i].isMember("angle_deg")) {
                                angleCam = detArray[i]["angle_deg"].asFloat();
                            }
                            if (detArray[i].isMember("label")) {
                                label = detArray[i]["label"].asString();
                            }
                            if (detArray[i].isMember("confidence")) {
                                confidence = detArray[i]["confidence"].asFloat();
                            }
                            if (detArray[i].isMember("area")) {
                                area = detArray[i]["area"].asFloat();
                            }

                            // Find the nearest angle in lidarData
                            float bestAngle = 0.0f;
                            float bestDist  = -1.0f;
                            float minDiff   = 9999.0f;

                            if (DEBUG_ANGLES) {
                                cout << "\nLooking for camera angle " << angleCam << "° in LiDAR data..." << endl;
                                cout << "Available LiDAR angles: ";
                                int count = 0;
                                for (auto &kv : lidarData) {
                                    if (count++ < 5) cout << kv.first << "° ";
                                }
                                cout << "... (" << lidarData.size() << " total)" << endl;
                            }

                            for (auto &kv : lidarData) {
                                float angleLidar = kv.first;
                                float distLidar  = kv.second;
                                float diff = fabs(angleCam - angleLidar);
                                if (diff < minDiff) {
                                    minDiff  = diff;
                                    bestAngle = angleLidar;
                                    bestDist  = distLidar;
                                }
                            }

                            // Only print if we found a good match
                            if (minDiff <= MAX_ANGLE_DIFF && bestDist > 0) {
                                cout << "\nObject Detected!" << endl;
                                cout << "- Label: " << label << endl;
                                cout << "- Confidence: " << confidence << endl;
                                cout << "- Camera Angle: " << angleCam << "°" << endl;
                                cout << "- LiDAR Angle: " << bestAngle << "°" << endl;
                                cout << "- Angle Diff: " << minDiff << "°" << endl;
                                cout << "- Distance: " << bestDist/1000.0f << " meters" << endl;
                                cout << "- Area: " << area << " px²" << endl;

                                // Send in format: "OBJECT,label,confidence,angle,distance,area"
                                stringstream ss;
                                ss << "OBJECT," << label << "," 
                                   << confidence << "," 
                                   << bestAngle << "," 
                                   << bestDist << "," 
                                   << area;
                                
                                try {
                                    string msg = ss.str();
                                    zmq::message_t message(msg.size());
                                    memcpy(message.data(), msg.c_str(), msg.size());
                                    g_corr_publisher->send(message, zmq::send_flags::none);
                                    
                                    cout << "\n\033[1;33m" << get_timestamp() << " [PORT 5557 - OBJECT]\033[0m" << endl;
                                    cout << "┌──────────────────────────────────────────────────────┐" << endl;
                                    cout << "│ Message: " << std::left << std::setw(45) << msg << " │" << endl;
                                    cout << "│                                                      │" << endl;
                                    cout << "│ Details:                                            │" << endl;
                                    cout << "│   • Label: " << std::setw(41) << label << " │" << endl;
                                    cout << "│   • Confidence: " << std::fixed << std::setprecision(2) 
                                         << std::setw(37) << confidence * 100 << "% │" << endl;
                                    cout << "│   • Camera Angle: " << std::setw(35) << angleCam << "° │" << endl;
                                    cout << "│   • LiDAR Angle: " << std::setw(36) << bestAngle << "° │" << endl;
                                    cout << "│   • Distance: " << std::setw(39) << bestDist/1000.0f << "m │" << endl;
                                    cout << "│   • Area: " << std::setw(42) << area << " px² │" << endl;
                                    cout << "└──────────────────────────────────────────────────────┘" << endl;
                                    cout.flush();
                                } catch (const zmq::error_t& e) {
                                    cerr << "Failed to send object detection: " << e.what() << endl;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Cleanup
    cout << "Cleaning up..." << endl;
    
    // Stop LiDAR
    if (g_drv) {
        g_drv->stop();
        delete g_drv;
        g_drv = nullptr;
    }
    
    // Close channel
    if (g_channel) {
        delete g_channel;
        g_channel = nullptr;
    }
    
    // Close ZMQ sockets
    if (g_publisher) {
        g_publisher->close();
        delete g_publisher;
        g_publisher = nullptr;
    }
    if (g_subscriber) {
        g_subscriber->close();
        delete g_subscriber;
        g_subscriber = nullptr;
    }
    if (g_corr_publisher) {
        g_corr_publisher->close();
        delete g_corr_publisher;
        g_corr_publisher = nullptr;
    }
    
    // Close ZMQ context
    if (g_context) {
        g_context->close();
        delete g_context;
        g_context = nullptr;
    }

    cout << "Cleanup complete. Exiting." << endl;
    return 0;
}
