#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <signal.h>
#include <zmq.hpp>
#include <zmq.h>  // For ZMQ constants
#include <sstream>
#include <map>
#include <unordered_map>  // Added for faster lookup
#include <cmath>
#include <jsoncpp/json/json.h>
#include "sl_lidar_driver.h"
#include <thread>
#include <chrono>  // For std::chrono

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
#define BATCH_SIZE 50            // Reduced batch size for more frequent updates
#define INIT_DELAY_MS 2000       // Initial delay for LiDAR operations (2 seconds)
#define SCAN_DELAY_MS 100        // Delay between scan attempts
#define VERBOSE_OUTPUT false      // Control terminal output
#define ANGLE_BUCKET_SIZE 5.0    // Size of angle buckets for faster correlation
#define PUBLISH_LIDAR_DATA true   // Toggle for publishing raw LIDAR data
#define FORCE_PUBLISH_MS 100     // Force object publishing every 100ms

// Global variables for cleanup
ILidarDriver* g_drv = nullptr;
IChannel* g_channel = nullptr;
zmq::context_t* g_context = nullptr;
zmq::socket_t* g_publisher = nullptr;
zmq::socket_t* g_subscriber = nullptr;
zmq::socket_t* g_corr_publisher = nullptr;
bool g_running = true;
uint64_t g_last_publish_time = 0;
uint64_t g_last_obj_publish_time = 0;  // Last time objects were published
int g_publish_count = 0;
bool g_publish_lidar_data = PUBLISH_LIDAR_DATA;  // Runtime toggle

// Optimized JSON writer settings
Json::StreamWriterBuilder g_writerBuilder;

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

// Bucketed angle map for faster correlation
unordered_map<int, float> g_angle_buckets;

void cleanup() {
    if (VERBOSE_OUTPUT) {
        cout << "\nCleaning up..." << endl;
    }
    
    // Stop LiDAR
    if (g_drv) {
        if (VERBOSE_OUTPUT) cout << "Stopping LiDAR..." << endl;
        g_drv->stop();
        delete g_drv;
        g_drv = nullptr;
    }
    
    // Close serial channel
    if (g_channel) {
        if (VERBOSE_OUTPUT) cout << "Closing serial channel..." << endl;
        delete g_channel;
        g_channel = nullptr;
    }
    
    // Close ZMQ sockets
    if (g_publisher) {
        if (VERBOSE_OUTPUT) cout << "Closing ZMQ publisher..." << endl;
        g_publisher->close();
        delete g_publisher;
        g_publisher = nullptr;
    }
    if (g_subscriber) {
        if (VERBOSE_OUTPUT) cout << "Closing ZMQ subscriber..." << endl;
        g_subscriber->close();
        delete g_subscriber;
        g_subscriber = nullptr;
    }
    if (g_corr_publisher) {
        if (VERBOSE_OUTPUT) cout << "Closing ZMQ correlation publisher..." << endl;
        g_corr_publisher->close();
        delete g_corr_publisher;
        g_corr_publisher = nullptr;
    }
    
    // Close ZMQ context
    if (g_context) {
        if (VERBOSE_OUTPUT) cout << "Closing ZMQ context..." << endl;
        g_context->close();
        delete g_context;
        g_context = nullptr;
    }
    
    if (VERBOSE_OUTPUT) cout << "Cleanup complete." << endl;
}

void toggleLidarPublishing() {
    g_publish_lidar_data = !g_publish_lidar_data;
    cout << "LIDAR data publishing " << (g_publish_lidar_data ? "enabled" : "disabled") << endl;
}

void signalHandler(int signum) {
    if (signum == SIGUSR1) {
        toggleLidarPublishing();
        return;
    }
    
    cout << "\nReceived signal " << signum << ", initiating cleanup..." << endl;
    g_running = false;
}

float convertRawAngleToDegrees(float raw_angle) {
    float angle = -raw_angle;
    while (angle <= -180.0f) angle += 360.0f;
    while (angle > 180.0f) angle -= 360.0f;
    return angle;
}

float roundToNearest(float value, float roundTo) {
    return roundTo * round(value / roundTo);
}

// Quantize angle to nearest bucket
int quantizeAngle(float angle) {
    return static_cast<int>(roundToNearest(angle, ANGLE_BUCKET_SIZE));
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

// Force publish current objects even if unchanged
void publishObjects(bool force = false) {
    uint64_t current_time = getCurrentTimeMs();
    
    // Check if we need to force publish based on timer
    bool should_publish = force || 
                         (current_time - g_last_obj_publish_time >= FORCE_PUBLISH_MS);
                         
    if (!should_publish || g_objects.empty()) {
        return;
    }
    
    // Reset the timer
    g_last_obj_publish_time = current_time;
    
    // Create JSON array for correlated objects
    Json::Value correlatedObjects(Json::arrayValue);
    
    // Add all current objects to the message
    for (const auto& obj_pair : g_objects) {
        const DetectedObject& obj = obj_pair.second;
        
        Json::Value objData;
        objData["label"] = obj.label;
        objData["confidence"] = obj.confidence;
        objData["angle_deg"] = obj.angle_deg;
        objData["distance_mm"] = obj.distance_mm;
        objData["area"] = obj.area;
        objData["timestamp"] = static_cast<Json::UInt64>(current_time);
        correlatedObjects.append(objData);
    }

    // Send all correlated objects in one message using optimized writer
    if (correlatedObjects.size() > 0) {
        Json::Value message;
        message["type"] = "OBJECTS";
        message["timestamp"] = static_cast<Json::UInt64>(current_time);
        message["objects"] = correlatedObjects;
        message["forced"] = force;  // Add a flag to indicate if this was a forced publish

        std::unique_ptr<Json::StreamWriter> writer(g_writerBuilder.newStreamWriter());
        std::stringstream ss;
        writer->write(message, &ss);
        std::string jsonStr = ss.str();
        
        try {
            zmq::message_t message(jsonStr.size());
            memcpy(message.data(), jsonStr.c_str(), jsonStr.size());
            g_corr_publisher->send(message, zmq::send_flags::dontwait);
            
            if (VERBOSE_OUTPUT && force) {
                std::cout << "Forced publish of " << correlatedObjects.size() << " objects" << std::endl;
            }
        } catch (const zmq::error_t&) {}
    }
}

// Find closest LIDAR point efficiently using the bucketed map
float findClosestLidarPoint(float targetAngle, float& minDiff) {
    int bucket = quantizeAngle(targetAngle);
    minDiff = 9999.0f;
    float bestDist = -1.0f;
    
    // Check target bucket and adjacent buckets
    for (int offset = -1; offset <= 1; offset++) {
        int checkBucket = bucket + (offset * ANGLE_BUCKET_SIZE);
        auto it = g_angle_buckets.find(checkBucket);
        if (it != g_angle_buckets.end()) {
            float diff = fabs(targetAngle - checkBucket);
            if (diff < minDiff) {
                minDiff = diff;
                bestDist = it->second;
            }
        }
    }
    
    return bestDist;
}

int main(int argc, const char *argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    signal(SIGUSR1, signalHandler);  // Add signal for toggling LIDAR publishing
    
    // Check command line arguments for initial state
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--no-lidar-publish") == 0) {
            g_publish_lidar_data = false;
            cout << "Starting with LIDAR data publishing disabled" << endl;
        }
    }

    // Initialize JSON writer settings for better performance
    g_writerBuilder["indentation"] = "";
    g_writerBuilder["commentStyle"] = "None";
    g_writerBuilder["precision"] = 1;  // Set to 1 decimal place precision
    g_writerBuilder["enableYAMLCompatibility"] = false;
    g_writerBuilder["dropNullPlaceholders"] = false;
    g_writerBuilder["omitEndingLineFeed"] = true;
    
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

        // Set CONFLATE option to only keep latest message
        int conflate = 1;
        g_publisher->set(zmq::sockopt::conflate, conflate);
        g_corr_publisher->set(zmq::sockopt::conflate, conflate);
        g_subscriber->set(zmq::sockopt::conflate, conflate);

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

        cout << "LiDAR system initialized:" << endl
             << "- Publishing LIDAR data on port " << ZMQ_PORT_PUB << (g_publish_lidar_data ? "" : " (disabled)") << endl
             << "- Publishing correlated objects on port " << ZMQ_PORT_OBJ << endl
             << "- Subscribing to camera detections on port " << ZMQ_PORT_SUB << endl
             << "- Send SIGUSR1 signal to toggle LIDAR data publishing" << endl;

    } catch (const zmq::error_t& e) {
        cerr << "Failed to initialize ZMQ: " << e.what() << endl;
        return -1;
    }

    // Initialize LIDAR
    Result<IChannel*> channel = createSerialPortChannel(SERIAL_PORT, SERIAL_BAUDRATE);
    if (!channel) {
        cerr << "Failed to create serial port channel" << endl;
        return -1;
    }
    g_channel = *channel;

    Result<ILidarDriver*> drv = createLidarDriver();
    if (!drv) {
        cerr << "Failed to create LiDAR driver" << endl;
        delete *channel;
        return -1;
    }
    g_drv = *drv;

    if (SL_IS_FAIL((*drv)->connect(*channel))) {
        cerr << "Failed to connect to LiDAR" << endl;
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

    // Check device health
    sl_lidar_response_device_health_t healthinfo;
    if (SL_IS_FAIL((*drv)->getHealth(healthinfo))) {
        cerr << "Failed to get health info" << endl;
        delete *drv;
        delete *channel;
        return -1;
    }

    if (healthinfo.status != SL_LIDAR_STATUS_OK) {
        cerr << "LiDAR health status: " << healthinfo.status << endl;
        delete *drv;
        delete *channel;
        return -1;
    }

    // Map to store downsampled points
    map<int, float> downsampledPoints;
    vector<pair<int, float>> batch;
    batch.reserve(BATCH_SIZE);

    // ZMQ polling setup
    zmq::pollitem_t items[] = {
        { g_subscriber->handle(), 0, ZMQ_POLLIN, 0 }
    };

    int consecutive_failures = 0;
    const int MAX_CONSECUTIVE_FAILURES = 3;
    uint64_t last_send_time = getCurrentTimeMs();

    // Stop any existing scan
    (*drv)->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(INIT_DELAY_MS));

    // Set motor speed to maximum
    if (SL_IS_FAIL((*drv)->setMotorSpeed(0))) {
        cerr << "Failed to set motor speed" << endl;
        delete *drv;
        delete *channel;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(INIT_DELAY_MS));

    // Check health status again before starting scan
    if (SL_IS_FAIL((*drv)->getHealth(healthinfo))) {
        cerr << "Failed to get health info" << endl;
        delete *drv;
        delete *channel;
        return -1;
    }
    cout << "LiDAR health status: " << healthinfo.status << endl;

    // Start scanning with express mode
    if (SL_IS_FAIL((*drv)->startScan(0, 1))) {
        cerr << "Failed to start scanning" << endl;
        delete *drv;
        delete *channel;
        return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(INIT_DELAY_MS));

    cout << "LiDAR initialized successfully" << endl;
    cout << "Device model: " << devinfo.model << endl;
    cout << "Firmware version: " << devinfo.firmware_version << endl;
    cout << "Hardware version: " << devinfo.hardware_version << endl;
    cout << "Serial number: " << devinfo.serialnum << endl;
    cout << "System running..." << endl;

    while (g_running) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        // Grab scan data with timeout
        if (SL_IS_FAIL((*drv)->grabScanDataHq(nodes, count))) {
            if (VERBOSE_OUTPUT) cerr << "Failed to grab scan data" << endl;
            consecutive_failures++;
            
            if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
                cerr << "Too many consecutive failures, stopping" << endl;
                break;
            }
            
            // Try to restart scanning with longer delays
            (*drv)->stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(SCAN_DELAY_MS));
            if (SL_IS_FAIL((*drv)->startScan(0, 1))) {
                cerr << "Failed to restart scanning" << endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(SCAN_DELAY_MS));
            continue;
        }

        // Reset failure counter on successful scan
        consecutive_failures = 0;

        // Check if we got any data
        if (count == 0) {
            if (VERBOSE_OUTPUT) cerr << "No scan data received" << endl;
            continue;
        }

        (*drv)->ascendScanData(nodes, count);

        // Clear downsampled points and buckets
        downsampledPoints.clear();
        g_angle_buckets.clear();

        // Process LIDAR data with downsampling
        for (size_t i = 0; i < count; i++) {
            float rawAngle = (nodes[i].angle_z_q14 * 360.0f) / (1 << 14);
            float angle = convertRawAngleToDegrees(rawAngle);
            float distance = nodes[i].dist_mm_q2 / 4.0f;

            // Only process points in front 180° and within distance limits
            if (angle >= -90.0f && angle <= 90.0f && 
                distance >= MIN_DISTANCE_MM && distance <= MAX_DISTANCE_MM) {
                
                // Removed distance rounding
                
                // Quantize angle to nearest bucket
                int bucketAngle = quantizeAngle(angle);
                
                // Keep the closest point for each angle bucket
                if (downsampledPoints.find(bucketAngle) == downsampledPoints.end() ||
                    distance < downsampledPoints[bucketAngle]) {
                    downsampledPoints[bucketAngle] = distance;
                    g_angle_buckets[bucketAngle] = distance;
                }
            }
        }

        // Send downsampled LIDAR data in batches
        if (!downsampledPoints.empty() && g_publish_lidar_data) {
            stringstream ss;
            ss << "LIDAR_DATA ";
            
            // Send all points immediately without batching
            for (const auto &kv : downsampledPoints) {
                ss << kv.first << "," << kv.second << ";";
            }
            
            try {
                string msg = ss.str();
                zmq::message_t message(msg.size());
                memcpy(message.data(), msg.c_str(), msg.size());
                g_publisher->send(message, zmq::send_flags::dontwait);
                
                // Update publish statistics
                g_publish_count++;
                if (getCurrentTimeMs() - g_last_publish_time >= 1000) {
                    if (VERBOSE_OUTPUT) cout << "Publishing " << g_publish_count << " messages/sec" << endl;
                    g_publish_count = 0;
                    g_last_publish_time = getCurrentTimeMs();
                }
            } catch (const zmq::error_t& e) {
                cerr << "Failed to send ZMQ message: " << e.what() << endl;
            }
        }

        // Poll for camera detections (non-blocking)
        zmq::poll(items, 1, std::chrono::milliseconds(1));  // 1ms timeout

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
                    bool new_detections = false;

                    // Create JSON array for correlated objects
                    Json::Value correlatedObjects(Json::arrayValue);

                    for (const auto& det : detArray) {
                        // Pre-quantize camera angle for faster lookup
                        float angleCam = det.get("angle_deg", 0.0f).asFloat();
                        int quantizedAngleCam = quantizeAngle(angleCam);
                        
                        string label = det.get("label", "").asString();
                        float confidence = det.get("confidence", 0.0f).asFloat();
                        float area = det.get("area", 0.0f).asFloat();

                        // Find nearest LIDAR point using efficient lookup
                        float minDiff;
                        float bestDist = -1.0f;
                        
                        // Check if the angle exists in our buckets
                        auto it = g_angle_buckets.find(quantizedAngleCam);
                        if (it != g_angle_buckets.end()) {
                            bestDist = it->second;
                            minDiff = 0;  // Exact match
                        } else {
                            // Look at adjacent buckets
                            bestDist = findClosestLidarPoint(angleCam, minDiff);
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
                            new_detections = true;

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

                    // Publish objects immediately if we had new detections
                    if (new_detections) {
                        publishObjects(true);  // Force publish
                    }
                }
            }
        }

        // Clean old objects periodically
        cleanOldObjects();
        
        // Force publish periodically regardless of changes
        uint64_t current_time = getCurrentTimeMs();
        if (current_time - g_last_obj_publish_time >= FORCE_PUBLISH_MS) {
            publishObjects(true);  // Force publish
        }
    }

    // Cleanup before exit
    cleanup();
    return 0;
}
