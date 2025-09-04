#include <Arduino.h>
#include <lvgl.h>
#include "I2C_Driver.h"
#include "TCA9554PWR.h"
#include "Display_ST7701.h"
#include "LVGL_Driver.h"
#include "Gyro_QMI8658.h"
#include "../imdat/imdat_include.h"

// Fish display system configuration
#define MAX_FISH 10
#define CIRCLE_CENTER_X 240
#define CIRCLE_CENTER_Y 240
#define CIRCLE_RADIUS 240
#define FISH_DOT_SIZE 8

// IMU and scrolling configuration
#define IMU_SAMPLE_RATE_HZ 50
#define IMU_SAMPLE_INTERVAL_MS (1000 / IMU_SAMPLE_RATE_HZ)
int32_t scroll_offset_x = 0;
int32_t scroll_offset_y = 0;

// Protocol structures
struct FishData {
    uint8_t sprite_id;
    uint8_t reserved[2];
    int32_t x;
    int32_t y;
    int16_t direction;
} __attribute__((packed));

struct ProtocolPacket {
    uint8_t protocol_id;
    uint32_t timestamp;
    uint8_t fish_count;
    FishData fish[MAX_FISH];
} __attribute__((packed));

// Protocol 0x01 scroll command packet
struct ScrollCommandPacket {
    uint8_t protocol_id;          // 0x01
    // uint8_t padding1[2];          // bytes 1-2
    uint32_t timestamp;           // bytes 3-6
    int32_t scroll_x;             // bytes 7-10
    int32_t scroll_y;             // bytes 11-14
    int16_t rotation_angle;       // bytes 15-16
} __attribute__((packed));

// IMU data packet for slave→master communication
struct IMUDataPacket {
    uint8_t protocol_id;          // 0x01 for slave→master
    // uint8_t padding1[2];          // bytes 1-2 
    uint32_t timestamp;           // bytes 3-6
    int16_t accel_x;              // bytes 7-8
    int16_t accel_y;              // bytes 9-10
    int16_t accel_z;              // bytes 11-12
    int16_t gyro_roll;            // bytes 13-14
    int16_t gyro_pitch;           // bytes 15-16
    int16_t gyro_yaw;             // bytes 17-18
    uint8_t button_state;         // byte 17 (buttons A and B)
} __attribute__((packed));

// Global variables
lv_obj_t* fish_objects[MAX_FISH] = {nullptr};
lv_obj_t* background_obj = nullptr;
lv_obj_t* background_tiles[9] = {nullptr};  // 3x3 tile grid for seamless scrolling
FishData current_fish[MAX_FISH];
uint8_t active_fish_count = 0;
uint8_t serial_buffer[256];
size_t buffer_pos = 0;

// IMU and button state
uint32_t last_imu_sample_time = 0;
uint8_t button_a_state = 0;
uint8_t button_b_state = 0;

// Fish image arrays
const lv_image_dsc_t* fish_type_01_images[] = {
    &IMID0x01_0, &IMID0x01_1, &IMID0x01_2, &IMID0x01_3
};
const lv_image_dsc_t* fish_type_02_images[] = {
    &IMID0x02_0, &IMID0x02_1, &IMID0x02_2, &IMID0x02_3
};
const int NUM_FISH_FRAMES = 4;

// Function declarations
void create_fish_display();
void create_tiled_background();
void update_background_scroll();
void draw_fish_images();
bool is_point_in_circle(int32_t x, int32_t y);
void handle_serial_data();
bool process_fish_packet(const uint8_t* data, size_t length);
bool process_scroll_command(const uint8_t* data, size_t length);
void create_fish_objects();
const lv_image_dsc_t* get_fish_image(uint8_t sprite_id, int16_t direction);
void send_imu_data();
void process_imu_sampling();

// Circular boundary validation
bool is_point_in_circle(int32_t x, int32_t y) {
    int32_t dx = x - CIRCLE_CENTER_X;
    int32_t dy = y - CIRCLE_CENTER_Y;
    return (dx * dx + dy * dy) <= (CIRCLE_RADIUS * CIRCLE_RADIUS);
}

// Get fish image based on sprite ID and direction
const lv_image_dsc_t* get_fish_image(uint8_t sprite_id, int16_t direction) {
    // Determine fish type (0x01 or 0x02 based on sprite_id)
    const lv_image_dsc_t** fish_images;
    if (sprite_id < 128) {
        fish_images = fish_type_01_images;
    } else {
        fish_images = fish_type_02_images;
    }
    
    // Select frame based on direction (0-360 degrees)
    int frame = (direction + 45) / 90;  // 0=East, 1=South, 2=West, 3=North
    frame = frame % NUM_FISH_FRAMES;
    
    return fish_images[frame];
}

// Create fish objects as LVGL image objects
void create_fish_objects() {
    for (int i = 0; i < MAX_FISH; i++) {
        fish_objects[i] = lv_image_create(lv_screen_active());
        lv_image_set_src(fish_objects[i], &IMID0x01_0);  // Default image
        lv_obj_add_flag(fish_objects[i], LV_OBJ_FLAG_HIDDEN);  // Initially hidden
    }
}

// Draw fish using proper images based on sprite ID and direction
void draw_fish_images() {
    // Hide all fish objects first
    for (int i = 0; i < MAX_FISH; i++) {
        lv_obj_add_flag(fish_objects[i], LV_OBJ_FLAG_HIDDEN);
    }
    
    // Show and position active fish
    for (int i = 0; i < active_fish_count && i < MAX_FISH; i++) {
        // Validate fish position is within circular boundary
        if (!is_point_in_circle(current_fish[i].x, current_fish[i].y)) {
            continue;
        }
        
        // Get appropriate fish image based on sprite ID and direction
        const lv_image_dsc_t* fish_img = get_fish_image(current_fish[i].sprite_id, current_fish[i].direction);
        lv_image_set_src(fish_objects[i], fish_img);
        
        // Apply scroll offset and position fish (centered on coordinates)
        // Get image dimensions to center properly
        lv_coord_t img_w = fish_img->header.w;
        lv_coord_t img_h = fish_img->header.h;
        
        // Apply scroll offset to fish position
        int32_t display_x = current_fish[i].x - scroll_offset_x;
        int32_t display_y = current_fish[i].y - scroll_offset_y;
        
        // Check if scrolled position is still within circular bounds
        if (!is_point_in_circle(display_x, display_y)) {
            continue;  // Skip fish outside visible area after scrolling
        }
        
        lv_obj_set_pos(fish_objects[i], 
                       display_x - img_w / 2, 
                       display_y - img_h / 2);
        
        // Show fish
        lv_obj_clear_flag(fish_objects[i], LV_OBJ_FLAG_HIDDEN);
    }
}

// Process Protocol ID 0x02 fish data packet
bool process_fish_packet(const uint8_t* data, size_t length) {
    if (length < 6) return false;  // Minimum packet size
    
    ProtocolPacket* packet = (ProtocolPacket*)data;
    
    if (packet->protocol_id != 0x02) return false;
    if (packet->fish_count > MAX_FISH) return false;
    
    size_t expected_size = 6 + (packet->fish_count * sizeof(FishData));
    if (length < expected_size) return false;
    
    // Update fish data
    active_fish_count = packet->fish_count;
    memcpy(current_fish, packet->fish, packet->fish_count * sizeof(FishData));
    
    Serial.printf("Received %d fish positions\n", active_fish_count);
    
    // Redraw fish
    draw_fish_images();
    
    return true;
}

// Process Protocol ID 0x01 scroll command packet
bool process_scroll_command(const uint8_t* data, size_t length) {
    if (length < sizeof(ScrollCommandPacket)) return false;
    
    ScrollCommandPacket* packet = (ScrollCommandPacket*)data;
    
    if (packet->protocol_id != 0x01) return false;
    
    // Update scroll offsets
    scroll_offset_x = packet->scroll_x;
    scroll_offset_y = packet->scroll_y;
    
    Serial.printf("Scroll command received: offset=(%d,%d) rotation=%d°\n", 
                  scroll_offset_x, scroll_offset_y, packet->rotation_angle);
    
    // Update background tiles with new scroll offset
    update_background_scroll();
    
    // Redraw fish with new scroll offset
    draw_fish_images();
    
    return true;
}

// Send IMU data to master
void send_imu_data() {
    // Get current accelerometer and gyroscope data
    getAccelerometer();
    getGyroscope();
    
    IMUDataPacket packet;
    packet.protocol_id = 0x01;
    // packet.padding1[0] = 0;
    // packet.padding1[1] = 0;
    packet.timestamp = millis();
    
    // Convert float accelerometer values to 16-bit scaled integers
    // IMU range: ±4G, 16-bit range: ±32767, scaling: 32767/4 = 8191.75
    packet.accel_x = (int16_t)(Accel.x * 8191.75f);
    packet.accel_y = (int16_t)(Accel.y * 8191.75f); 
    packet.accel_z = (int16_t)(Accel.z * 8191.75f);
    
    // Convert float gyroscope values to 16-bit scaled integers  
    // IMU range: ±2000deg/s, 16-bit range: ±32767, scaling: 32767/2000 = 16.38
    packet.gyro_roll = (int16_t)(Gyro.x * 16.38f);
    packet.gyro_pitch = (int16_t)(Gyro.y * 16.38f);
    packet.gyro_yaw = (int16_t)(Gyro.z * 16.38f);
    
    // Set button state (bit 0 = button A, bit 1 = button B)
    packet.button_state = (button_b_state << 1) | button_a_state;
    
    // Send packet via serial
    Serial.write((uint8_t*)&packet, sizeof(packet));
}

// Process IMU sampling at specified rate
void process_imu_sampling() {
    uint32_t current_time = millis();
    
    if (current_time - last_imu_sample_time >= IMU_SAMPLE_INTERVAL_MS) {
        send_imu_data();
        last_imu_sample_time = current_time;
    }
}

// Handle incoming serial data
void handle_serial_data() {
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        
        if (buffer_pos < sizeof(serial_buffer)) {
            serial_buffer[buffer_pos++] = byte;
        }
        
        // Check if we have enough data for a packet
        if (buffer_pos >= 6) {  // Minimum packet size
            if (serial_buffer[0] == 0x01) {  // Protocol ID 0x01 - Scroll command
                if (buffer_pos >= sizeof(ScrollCommandPacket)) {
                    if (process_scroll_command(serial_buffer, buffer_pos)) {
                        // Packet processed successfully, reset buffer
                        buffer_pos = 0;
                    } else {
                        // Invalid packet, shift buffer
                        memmove(serial_buffer, serial_buffer + 1, buffer_pos - 1);
                        buffer_pos--;
                    }
                }
            } else if (serial_buffer[0] == 0x02) {  // Protocol ID 0x02 - Fish data
                uint8_t fish_count = serial_buffer[5];
                size_t expected_size = 6 + (fish_count * sizeof(FishData));
                
                if (buffer_pos >= expected_size) {
                    if (process_fish_packet(serial_buffer, buffer_pos)) {
                        // Packet processed successfully, reset buffer
                        buffer_pos = 0;
                    } else {
                        // Invalid packet, shift buffer
                        memmove(serial_buffer, serial_buffer + 1, buffer_pos - 1);
                        buffer_pos--;
                    }
                }
            } else {
                // Unknown protocol, shift buffer
                memmove(serial_buffer, serial_buffer + 1, buffer_pos - 1);
                buffer_pos--;
            }
        }
    }
}

// Create tiled scrollable background
void create_tiled_background() {
    // Get background image dimensions
    // Assume background2 is available and use its dimensions
    const lv_image_dsc_t* bg_img = &background2;
    lv_coord_t tile_width = bg_img->header.w;
    lv_coord_t tile_height = bg_img->header.h;
    
    // Create 3x3 grid of background tiles for seamless scrolling
    // This allows scrolling in any direction without gaps
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            int tile_index = row * 3 + col;
            
            // Create background tile
            background_tiles[tile_index] = lv_image_create(lv_screen_active());
            lv_image_set_src(background_tiles[tile_index], bg_img);
            
            // Position tiles in a 3x3 grid centered around screen
            // Center tile (index 4) is at screen center
            int pos_x = (col - 1) * tile_width + (ESP_PANEL_LCD_WIDTH - tile_width) / 2;
            int pos_y = (row - 1) * tile_height + (ESP_PANEL_LCD_HEIGHT - tile_height) / 2;
            
            lv_obj_set_pos(background_tiles[tile_index], pos_x, pos_y);
            
            // Ensure background tiles are behind everything else
            lv_obj_move_background(background_tiles[tile_index]);
        }
    }
    
    Serial.printf("Created 3x3 tiled background: %dx%d tiles\n", tile_width, tile_height);
}

// Update background tile positions based on scroll offset
void update_background_scroll() {
    if (!background_tiles[0]) return;  // Not initialized yet
    
    // Get background image dimensions
    const lv_image_dsc_t* bg_img = &background2;
    lv_coord_t tile_width = bg_img->header.w;
    lv_coord_t tile_height = bg_img->header.h;
    
    // Calculate the scroll offset modulo tile size to create seamless wrapping
    int wrapped_offset_x = scroll_offset_x % tile_width;
    int wrapped_offset_y = scroll_offset_y % tile_height;
    
    // Ensure positive modulo
    if (wrapped_offset_x < 0) wrapped_offset_x += tile_width;
    if (wrapped_offset_y < 0) wrapped_offset_y += tile_height;
    
    // Update positions for 3x3 grid of tiles
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            int tile_index = row * 3 + col;
            
            // Calculate base tile position (before scroll)
            int base_x = (col - 1) * tile_width + (ESP_PANEL_LCD_WIDTH - tile_width) / 2;
            int base_y = (row - 1) * tile_height + (ESP_PANEL_LCD_HEIGHT - tile_height) / 2;
            
            // Apply scroll offset (background moves opposite to scroll direction)
            int final_x = base_x - wrapped_offset_x;
            int final_y = base_y - wrapped_offset_y;
            
            lv_obj_set_pos(background_tiles[tile_index], final_x, final_y);
        }
    }
}

// Create fish display system
void create_fish_display() {
    // Create tiled scrollable background first
    create_tiled_background();
    
    // Create fish objects
    create_fish_objects();
    
    // Create title
    lv_obj_t* title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "Fish Display System");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
    
    // Create status label
    lv_obj_t* status = lv_label_create(lv_screen_active());
    lv_label_set_text(status, "Waiting for fish data...");
    lv_obj_set_style_text_color(status, lv_color_white(), 0);
    lv_obj_set_style_text_align(status, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(status, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    Serial.println("Fish display system initialized");
}

void Driver_Init() {
    Serial.println("Driver_Init開始");
    
    // I2C初期化
    I2C_Init();
    delay(120);
    
    // 拡張IOチップ初期化
    TCA9554PWR_Init(0x00);
    
    // EXIO_PIN8をLowに設定
    Set_EXIO(8, LOW);
    
    // バックライト初期化
    Backlight_Init();
    Set_Backlight(100);
    
    Serial.println("Driver_Init完了");
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32-S3 Fish Display System Starting...");
    
    Serial.printf("Total heap: %d\n", ESP.getHeapSize()); 
    Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
    Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
    
    // Initialize drivers in correct order
    Driver_Init();              
    LCD_Init();                 
    Lvgl_Init();
    
    // Initialize IMU
    Serial.println("Initializing IMU...");
    QMI8658_Init();
    delay(500);
    
    // Test IMU reading
    Serial.println("Testing IMU reading...");
    for (int i = 0; i < 3; i++) {
        getAccelerometer();
        Serial.printf("IMU Test %d: X=%.3f, Y=%.3f, Z=%.3f\n", i+1, Accel.x, Accel.y, Accel.z);
        delay(100);
    }
    
    // Create fish display UI
    create_fish_display();
    
    Serial.println("=== Fish Display System Ready ===");
    Serial.println("Protocol: 115200 baud, 8N1");
    Serial.println("Receiving: Protocol ID 0x01 (scroll) and 0x02 (fish) packets");
    Serial.printf("Transmitting: IMU data at %d Hz\n", IMU_SAMPLE_RATE_HZ);
    Serial.printf("Maximum fish: %d\n", MAX_FISH);
    Serial.printf("Display: %dx%d circular (center=%d,%d, radius=%d)\n", 
                   ESP_PANEL_LCD_WIDTH, ESP_PANEL_LCD_HEIGHT, 
                   CIRCLE_CENTER_X, CIRCLE_CENTER_Y, CIRCLE_RADIUS);
    
    Serial.printf("Free heap after init: %d\n", ESP.getFreeHeap());
    Serial.printf("Free PSRAM after init: %d\n", ESP.getFreePsram());
}

void loop() {
    // Handle LVGL tasks
    lv_timer_handler();
    
    // Process incoming serial data
    handle_serial_data();
    
    // Send IMU data to master at specified rate
    process_imu_sampling();
    
    delay(5);
}