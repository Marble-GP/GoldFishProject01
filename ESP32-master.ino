/*
 * ESP32 Master Program for Fish Display with IMU Scrolling
 * Converted from fish_sender_with_imu_scroll.py
 * 
 * This program runs on ESP32 and acts as master, receiving IMU data 
 * from slave ESP32 and sending back fish data + scroll commands
 * 
 * Features:
 * - Receives IMU data from slave ESP32
 * - Processes attitude (roll/pitch) estimation
 * - Generates scroll commands based on tilt
 * - Sends realistic fish movement data
 * - Scroll area limited to 1440x1440 pixels
 */

#include <Arduino.h>
#include <WiFi.h>
#include <math.h>

// Display and scrolling constants
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 480
#define SCREEN_CENTER_X (SCREEN_WIDTH / 2)
#define SCREEN_CENTER_Y (SCREEN_HEIGHT / 2)
#define CIRCLE_RADIUS 240

// Scroll area limitation (1440x1440 = 3x screen size)
#define MAX_SCROLL_AREA 1440
#define SCROLL_LIMIT (MAX_SCROLL_AREA / 2)  // ±720 pixels

// IMU processing parameters
#define SAMPLE_RATE_HZ 100.0
#define STILLNESS_THRESHOLD 12.5  // degrees
#define DEAD_ZONE_THRESHOLD 15.0  // degrees
#define TILT_FORCE_SCALE 100.0
#define VELOCITY_DECAY 0.85
#define MAX_SCROLL_VELOCITY 50.0  // pixels/second

// Fish simulation parameters
#define MAX_FISH 4
#define UPDATE_RATE_HZ 30
#define UPDATE_INTERVAL_MS (1000 / UPDATE_RATE_HZ)

// Communication protocols
struct IMUDataPacket {
    uint8_t protocol_id;    // 0x01
    uint32_t timestamp;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_roll;
    int16_t gyro_pitch;
    int16_t gyro_yaw;
    uint8_t button_state;
} __attribute__((packed));

struct ScrollCommandPacket {
    uint8_t protocol_id;    // 0x01
    uint32_t timestamp;
    int32_t scroll_x;
    int32_t scroll_y;
    int16_t rotation_angle;
} __attribute__((packed));

struct FishData {
    uint8_t sprite_id;
    uint8_t reserved[2];
    int32_t x;
    int32_t y;
    int16_t direction;
} __attribute__((packed));

struct FishDataPacket {
    uint8_t protocol_id;    // 0x02
    uint32_t timestamp;
    uint8_t fish_count;
    FishData fish[MAX_FISH];
} __attribute__((packed));

// Simple bandpass filter for attitude detection
class SimpleBandpassFilter {
private:
    float prev_input;
    float prev_output;
    float alpha;  // Filter coefficient
    
public:
    SimpleBandpassFilter() {
        prev_input = 0.0;
        prev_output = 0.0;
        alpha = 0.1;  // Simplified filter coefficient
    }
    
    float apply(float input) {
        // Simple high-pass component
        float highpass = input - prev_input;
        prev_input = input;
        
        // Simple low-pass component
        prev_output = prev_output * (1.0 - alpha) + highpass * alpha;
        return prev_output;
    }
};

// IMU data processor
class IMUDataProcessor {
private:
    float roll_deg;
    float pitch_deg;
    float scroll_velocity_x;
    float scroll_velocity_y;
    float scroll_position_x;
    float scroll_position_y;
    unsigned long last_update_time;
    
    SimpleBandpassFilter roll_filter;
    SimpleBandpassFilter pitch_filter;
    float roll_filtered;
    float pitch_filtered;
    
public:
    IMUDataProcessor() {
        roll_deg = 0.0;
        pitch_deg = 0.0;
        scroll_velocity_x = 0.0;
        scroll_velocity_y = 0.0;
        scroll_position_x = 0.0;
        scroll_position_y = 0.0;
        roll_filtered = 0.0;
        pitch_filtered = 0.0;
        last_update_time = millis();
    }
    
    bool processIMUData(const IMUDataPacket* packet) {
        if (packet->protocol_id != 0x01) return false;
        
        // Convert raw accelerometer to G units
        float accel_x = packet->accel_y / 8191.75;  // Note: XY swapped for coordinate system
        float accel_y = packet->accel_x / 8191.75;
        float accel_z = packet->accel_z / 8191.75;
        
        // Calculate roll and pitch from accelerometer
        roll_deg = atan2(accel_y, sqrt(accel_x*accel_x + accel_z*accel_z)) * 180.0 / PI;
        pitch_deg = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * 180.0 / PI;
        
        // Apply bandpass filter for stillness detection
        roll_filtered = roll_filter.apply(roll_deg);
        pitch_filtered = pitch_filter.apply(pitch_deg);
        
        updateScrollFromAttitude();
        return true;
    }
    
private:
    void updateScrollFromAttitude() {
        unsigned long current_time = millis();
        float dt = (current_time - last_update_time) / 1000.0;
        last_update_time = current_time;
        
        if (dt > 0.1) return; // Skip large time deltas
        
        // Check if device is still
        float filtered_magnitude = sqrt(roll_filtered*roll_filtered + pitch_filtered*pitch_filtered);
        bool is_still = filtered_magnitude < STILLNESS_THRESHOLD;
        
        // Check dead zone
        float total_tilt = sqrt(roll_deg*roll_deg + pitch_deg*pitch_deg);
        bool in_dead_zone = total_tilt < DEAD_ZONE_THRESHOLD;
        
        // Apply forces or decay
        if (is_still && !in_dead_zone) {
            // Apply tilt forces
            float force_x = sin(pitch_deg * PI / 180.0) * TILT_FORCE_SCALE;
            float force_y = sin(roll_deg * PI / 180.0) * TILT_FORCE_SCALE;
            
            scroll_velocity_x = scroll_velocity_x * VELOCITY_DECAY + force_x * dt;
            scroll_velocity_y = scroll_velocity_y * VELOCITY_DECAY + force_y * dt;
        } else {
            // Apply stronger decay
            scroll_velocity_x *= 0.7;
            scroll_velocity_y *= 0.7;
        }
        
        // Limit velocity
        float velocity_magnitude = sqrt(scroll_velocity_x*scroll_velocity_x + scroll_velocity_y*scroll_velocity_y);
        if (velocity_magnitude > MAX_SCROLL_VELOCITY) {
            float scale = MAX_SCROLL_VELOCITY / velocity_magnitude;
            scroll_velocity_x *= scale;
            scroll_velocity_y *= scale;
        }
        
        // Update position
        scroll_position_x += scroll_velocity_x * dt;
        scroll_position_y += scroll_velocity_y * dt;
        
        // Apply scroll area limitation
        scroll_position_x = constrain(scroll_position_x, -SCROLL_LIMIT, SCROLL_LIMIT);
        scroll_position_y = constrain(scroll_position_y, -SCROLL_LIMIT, SCROLL_LIMIT);
    }
    
public:
    int32_t getScrollX() { return (int32_t)scroll_position_x; }
    int32_t getScrollY() { return (int32_t)scroll_position_y; }
    
    void printDebugInfo() {
        float total_tilt = sqrt(roll_deg*roll_deg + pitch_deg*pitch_deg);
        float filtered_magnitude = sqrt(roll_filtered*roll_filtered + pitch_filtered*pitch_filtered);
        bool is_still = filtered_magnitude < STILLNESS_THRESHOLD;
        bool in_dead_zone = total_tilt < DEAD_ZONE_THRESHOLD;
        
        Serial.printf("Attitude: Roll=%.1f° Pitch=%.1f° Total=%.1f° | Status: %s %s | Scroll: (%d,%d) | Vel: (%.1f,%.1f)\n",
                     roll_deg, pitch_deg, total_tilt,
                     is_still ? "STILL" : "MOVING", 
                     in_dead_zone ? "DEAD_ZONE" : "ACTIVE",
                     getScrollX(), getScrollY(),
                     scroll_velocity_x, scroll_velocity_y);
    }
};

// Simple fish model for movement simulation
class SimpleFishModel {
private:
    float pos_x, pos_y;
    float vel_x, vel_y;
    float angle;
    uint8_t fish_id;
    
public:
    SimpleFishModel(uint8_t id, float x, float y) {
        fish_id = id;
        pos_x = x;
        pos_y = y;
        vel_x = random(-10, 10) / 10.0;
        vel_y = random(-10, 10) / 10.0;
        angle = 0.0;
    }
    
    void update() {
        // Simple random walk with boundary reflection
        vel_x += random(-5, 5) / 100.0;
        vel_y += random(-5, 5) / 100.0;
        
        // Limit velocity
        vel_x = constrain(vel_x, -2.0, 2.0);
        vel_y = constrain(vel_y, -2.0, 2.0);
        
        // Update position
        pos_x += vel_x;
        pos_y += vel_y;
        
        // Boundary reflection (circular)
        float dx = pos_x - SCREEN_CENTER_X;
        float dy = pos_y - SCREEN_CENTER_Y;
        float distance = sqrt(dx*dx + dy*dy);
        
        if (distance > CIRCLE_RADIUS - 10) {
            float normal_x = dx / distance;
            float normal_y = dy / distance;
            
            // Reflect velocity
            float dot = vel_x * normal_x + vel_y * normal_y;
            vel_x -= 2 * dot * normal_x;
            vel_y -= 2 * dot * normal_y;
            
            // Move back inside
            pos_x = SCREEN_CENTER_X + normal_x * (CIRCLE_RADIUS - 15);
            pos_y = SCREEN_CENTER_Y + normal_y * (CIRCLE_RADIUS - 15);
        }
        
        // Update angle
        if (abs(vel_x) > 0.1 || abs(vel_y) > 0.1) {
            angle = atan2(vel_y, vel_x) * 180.0 / PI;
        }
    }
    
    FishData getFishData() {
        FishData fish;
        fish.sprite_id = fish_id % 3;
        fish.reserved[0] = 0;
        fish.reserved[1] = 0;
        fish.x = (int32_t)pos_x;
        fish.y = (int32_t)pos_y;
        fish.direction = (int16_t)angle;
        return fish;
    }
};

// Global objects
IMUDataProcessor imuProcessor;
SimpleFishModel* fishModels[MAX_FISH];
uint8_t receiveBuffer[64];
int bufferPos = 0;
unsigned long lastUpdateTime = 0;
unsigned long lastStatusTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Master - Fish Display with IMU Scrolling");
    Serial.println("Converted from Python fish_sender_with_imu_scroll.py");
    Serial.println("Features: IMU processing, scroll limitation, fish simulation");
    
    // Initialize fish models
    for (int i = 0; i < MAX_FISH; i++) {
        float angle = (2.0 * PI * i) / MAX_FISH;
        float radius = 50 + random(0, 100);
        float x = SCREEN_CENTER_X + radius * cos(angle);
        float y = SCREEN_CENTER_Y + radius * sin(angle);
        fishModels[i] = new SimpleFishModel(i, x, y);
    }
    
    Serial.printf("Initialized %d fish models\n", MAX_FISH);
    Serial.printf("Scroll area limited to %dx%d (±%d pixels)\n", MAX_SCROLL_AREA, MAX_SCROLL_AREA, SCROLL_LIMIT);
    Serial.println("Ready to receive IMU data and send fish/scroll commands");
    Serial.println();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Process incoming IMU data
    processIncomingIMUData();
    
    // Update and send fish data at specified rate
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL_MS) {
        updateFishPositions();
        sendFishData();
        sendScrollCommand();
        lastUpdateTime = currentTime;
    }
    
    // Print status every 2 seconds
    if (currentTime - lastStatusTime >= 2000) {
        imuProcessor.printDebugInfo();
        lastStatusTime = currentTime;
    }
    
    delay(5);
}

void processIncomingIMUData() {
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        
        if (bufferPos < sizeof(receiveBuffer)) {
            receiveBuffer[bufferPos++] = byte;
        }
        
        // Check for complete IMU packet (18 bytes)
        if (bufferPos >= 18) {
            if (receiveBuffer[0] == 0x01) {  // IMU packet
                IMUDataPacket* packet = (IMUDataPacket*)receiveBuffer;
                if (imuProcessor.processIMUData(packet)) {
                    // Successfully processed, reset buffer
                    bufferPos = 0;
                } else {
                    // Invalid packet, shift buffer
                    memmove(receiveBuffer, receiveBuffer + 1, bufferPos - 1);
                    bufferPos--;
                }
            } else {
                // Unknown packet, shift buffer
                memmove(receiveBuffer, receiveBuffer + 1, bufferPos - 1);
                bufferPos--;
            }
        }
    }
}

void updateFishPositions() {
    for (int i = 0; i < MAX_FISH; i++) {
        fishModels[i]->update();
    }
}

void sendFishData() {
    FishDataPacket packet;
    packet.protocol_id = 0x02;
    packet.timestamp = millis();
    packet.fish_count = MAX_FISH;
    
    for (int i = 0; i < MAX_FISH; i++) {
        packet.fish[i] = fishModels[i]->getFishData();
    }
    
    Serial.write((uint8_t*)&packet, sizeof(FishDataPacket) - sizeof(FishData) * (MAX_FISH - packet.fish_count));
}

void sendScrollCommand() {
    ScrollCommandPacket packet;
    packet.protocol_id = 0x01;
    packet.timestamp = millis();
    packet.scroll_x = imuProcessor.getScrollX();
    packet.scroll_y = imuProcessor.getScrollY();
    packet.rotation_angle = 0;
    
    Serial.write((uint8_t*)&packet, sizeof(ScrollCommandPacket));
}