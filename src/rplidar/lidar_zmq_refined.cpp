#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <signal.h>
#include <zmq.hpp>
#include <zmq.h>  // For ZMQ constants
#include <sstream>
#include <map>
#include <cmath>
#include <jsoncpp/json/json.h>
#include "sl_lidar_driver.h"

using namespace sl;
using namespace std;

#define SERIAL_PORT "/dev/ttyUSB0"
#define SERIAL_BAUDRATE 460800
#define ZMQ_PORT_PUB "5556"      // Raw LIDAR data
#define ZMQ_PORT_SUB "5555"      // Camera detections
#define ZMQ_PORT_OBJ "5557"      // Correlated objects
#define MAX_ANGLE_DIFF 10.0      // Maximum angle difference for correlation
#define ANGLE_RESOLUTION 1.0     // Only send points every 1 degree
#define MIN_DISTANCE_MM 100      // Ignore points closer than 10cm
#define MAX_DISTANCE_MM 3000     // Ignore points further than 3m
#define MAX_OBJECT_AGE_MS 500    // Keep objects for 500ms

// Global variables for cleanup
ILidarDriver* g_drv = nullptr;
IChannel* g_channel = nullptr;
zmq::context_t* g_context = nullptr;
zmq::socket_t* g_publisher = nullptr;
zmq::socket_t* g_subscriber = nullptr;
zmq::socket_t* g_corr_publisher = nullptr;
bool g_running = true;

// Structure to hold object data
struct DetectedObject {
    string label;
    float confidence;
    float angle_deg;
    float distance_mm;
    float area;
    uint64_t last_update_ms;
};

// Map to store detected objects
map<string, DetectedObject> g_objects;

void signalHandler(int signum) {
    g_running = false;
}

float convertRawAngleToDegrees(float raw_angle) {
    float angle = -raw_angle;
    while (angle <= -180.0f) angle += 360.0f;
    while (angle > 180.0f) angle -= 360.0f;
    return angle;
}

// Get current time in milliseconds
uint64_t getCurrentTimeMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

// Clean old objects from the map
void cleanOldObjects() {
    uint64_t current_time = getCurrentTimeMs();
    auto it = g_objects.begin();
    while (it != g_objects.end()) {
        if (current_time - it->second.last_update_ms > MAX_OBJECT_AGE_MS) {
            it = g_objects.erase(it);
        } else {
            ++it;
        }
    }
}

int main(int argc, const char *argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Initialize ZMQ with optimized settings
    try {
        g_context = new zmq::context_t(1);
        g_publisher = new zmq::socket_t(*g_context, ZMQ_PUB);
        g_corr_publisher = new zmq::socket_t(*g_context, ZMQ_PUB);
        g_subscriber = new zmq::socket_t(*g_context, ZMQ_SUB);

        // Set high water mark to 1 to prevent message queuing
        int hwm = 1;
        g_publisher->set(zmq::sockopt::sndhwm, hwm);
        g_corr_publisher->set(zmq::sockopt::sndhwm, hwm);
        g_subscriber->set(zmq::sockopt::rcvhwm, hwm);

        // Set socket options for performance
        int linger = 0;
        g_publisher->set(zmq::sockopt::linger, linger);
        g_corr_publisher->set(zmq::sockopt::linger, linger);
        g_subscriber->set(zmq::sockopt::linger, linger);

        string address_pub = "tcp://*:" + string(ZMQ_PORT_PUB);
        string address_obj = "tcp://*:" + string(ZMQ_PORT_OBJ);
        string address_sub = "tcp://localhost:" + string(ZMQ_PORT_SUB);

        g_publisher->bind(address_pub);
        g_corr_publisher->bind(address_obj);
        g_subscriber->connect(address_sub);
        g_subscriber->set(zmq::sockopt::subscribe, "");

        cout << "ZMQ initialized:" << endl
             << "- Publishing LIDAR data on port " << ZMQ_PORT_PUB << endl
             << "- Publishing correlated objects on port " << ZMQ_PORT_OBJ << endl
             << "- Subscribing to camera detections on port " << ZMQ_PORT_SUB << endl;

    } catch (const zmq::error_t& e) {
        cerr << "Failed to initialize ZMQ: " << e.what() << endl;
        return -1;
    }

    // Initialize LIDAR
    Result<IChannel*> channel = createSerialPortChannel(SERIAL_PORT, SERIAL_BAUDRATE);
    if (!channel) return -1;
    g_channel = *channel;

    Result<ILidarDriver*> drv = createLidarDriver();
    if (!drv) {
        delete *channel;
        return -1;
    }
    g_drv = *drv;

    if (SL_IS_FAIL((*drv)->connect(*channel))) {
        delete *drv;
        delete *channel;
        return -1;
    }

    sl_lidar_response_device_info_t devinfo;
    if (SL_IS_FAIL((*drv)->getDeviceInfo(devinfo))) {
        delete *drv;
        delete *channel;
        return -1;
    }

    // Start scanning with express mode
    (*drv)->startScan(0, 1);

    // Map to store downsampled points
    map<int, float> downsampledPoints;

    // ZMQ polling setup
    zmq::pollitem_t items[] = {
        { g_subscriber->handle(), 0, ZMQ_POLLIN, 0 }
    };

    while (g_running) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        if (SL_IS_FAIL((*drv)->grabScanDataHq(nodes, count))) break;
        (*drv)->ascendScanData(nodes, count);

        // Clear downsampled points
        downsampledPoints.clear();

        // Process LIDAR data with downsampling
        for (size_t i = 0; i < count; i++) {
            float rawAngle = (nodes[i].angle_z_q14 * 360.0f) / (1 << 14);
            float angle = convertRawAngleToDegrees(rawAngle);
            float distance = nodes[i].dist_mm_q2 / 4.0f;

            // Only process points in front 180° and within distance limits
            if (angle >= -90.0f && angle <= 90.0f && 
                distance >= MIN_DISTANCE_MM && distance <= MAX_DISTANCE_MM) {
                
                // Round angle to nearest resolution step
                int bucketAngle = static_cast<int>(round(angle / ANGLE_RESOLUTION) * ANGLE_RESOLUTION);
                
                // Keep the closest point for each angle bucket
                if (downsampledPoints.find(bucketAngle) == downsampledPoints.end() ||
                    distance < downsampledPoints[bucketAngle]) {
                    downsampledPoints[bucketAngle] = distance;
                }
            }
        }

        // Send downsampled LIDAR data
        if (!downsampledPoints.empty()) {
            stringstream ss;
            ss << "LIDAR_DATA ";
            for (const auto &kv : downsampledPoints) {
                ss << kv.first << "," << kv.second << ";";
            }
            try {
                string msg = ss.str();
                zmq::message_t message(msg.size());
                memcpy(message.data(), msg.c_str(), msg.size());
                g_publisher->send(message, zmq::send_flags::dontwait);
            } catch (const zmq::error_t&) {}
        }

        // Poll for camera detections (non-blocking)
        zmq::poll(items, 1, 1);  // 1ms timeout

        // Check for object detections
        if (items[0].revents & ZMQ_POLLIN) {
            zmq::message_t detectionMsg;
            if (g_subscriber->recv(detectionMsg, zmq::recv_flags::dontwait)) {
                string detStr(static_cast<char*>(detectionMsg.data()), detectionMsg.size());
                
                Json::Value root;
                Json::Reader reader;
                if (reader.parse(detStr, root, false) && root.isMember("detections") && root["detections"].isArray()) {
                    const Json::Value& detArray = root["detections"];
                    uint64_t current_time = getCurrentTimeMs();

                    // Create JSON array for correlated objects
                    Json::Value correlatedObjects(Json::arrayValue);

                    for (const auto& det : detArray) {
                        float angleCam = det.get("angle_deg", 0.0f).asFloat();
                        string label = det.get("label", "").asString();
                        float confidence = det.get("confidence", 0.0f).asFloat();
                        float area = det.get("area", 0.0f).asFloat();

                        // Find nearest LIDAR point
                        float bestDist = -1.0f;
                        float minDiff = 9999.0f;

                        for (const auto &kv : downsampledPoints) {
                            float diff = fabs(angleCam - kv.first);
                            if (diff < minDiff) {
                                minDiff = diff;
                                bestDist = kv.second;
                            }
                        }

                        // If we found a matching LIDAR point
                        if (minDiff <= MAX_ANGLE_DIFF && bestDist > 0) {
                            // Create unique ID for object
                            string objId = label + "_" + to_string(static_cast<int>(angleCam));

                            // Update or create object
                            DetectedObject& obj = g_objects[objId];
                            obj.label = label;
                            obj.confidence = confidence;
                            obj.angle_deg = angleCam;
                            obj.distance_mm = bestDist;
                            obj.area = area;
                            obj.last_update_ms = current_time;

                            // Add to JSON array
                            Json::Value objData;
                            objData["label"] = label;
                            objData["confidence"] = confidence;
                            objData["angle_deg"] = angleCam;
                            objData["distance_mm"] = bestDist;
                            objData["area"] = area;
                            objData["timestamp"] = static_cast<Json::UInt64>(current_time);
                            correlatedObjects.append(objData);
                        }
                    }

                    // Send all correlated objects in one message
                    if (correlatedObjects.size() > 0) {
                        Json::Value message;
                        message["type"] = "OBJECTS";
                        message["timestamp"] = static_cast<Json::UInt64>(current_time);
                        message["objects"] = correlatedObjects;

                        Json::FastWriter writer;
                        string jsonStr = writer.write(message);
                        
                        try {
                            zmq::message_t message(jsonStr.size());
                            memcpy(message.data(), jsonStr.c_str(), jsonStr.size());
                            g_corr_publisher->send(message, zmq::send_flags::dontwait);
                        } catch (const zmq::error_t&) {}
                    }
                }
            }
        }

        // Clean old objects periodically
        cleanOldObjects();
    }

    // Cleanup
    if (g_drv) {
        g_drv->stop();
        delete g_drv;
    }
    if (g_channel) delete g_channel;
    if (g_publisher) {
        g_publisher->close();
        delete g_publisher;
    }
    if (g_subscriber) {
        g_subscriber->close();
        delete g_subscriber;
    }
    if (g_corr_publisher) {
        g_corr_publisher->close();
        delete g_corr_publisher;
    }
    if (g_context) {
        g_context->close();
        delete g_context;
    }

    return 0;
}