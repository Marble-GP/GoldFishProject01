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

// Performance optimization settings
#define LVGL_UPDATE_RATE_HZ 30  // Reduce LVGL refresh rate
#define LVGL_UPDATE_INTERVAL_MS (1000 / LVGL_UPDATE_RATE_HZ)
#define SERIAL_BAUDRATE 500000   // Increase baudrate from 115200
#define SERIAL_BUFFER_SIZE 512   // Larger serial buffer

int32_t scroll_offset_x = 0;
int32_t scroll_offset_y = 0;

// Timing variables for separate update rates
uint32_t last_imu_time = 0;
uint32_t last_lvgl_time = 0;
uint32_t last_fish_update_time = 0;

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

struct ScrollCommandPacket {
    uint8_t protocol_id;
    uint32_t timestamp;
    int32_t scroll_x;
    int32_t scroll_y;
    int16_t rotation_angle;
} __attribute__((packed));

struct IMUDataPacket {
    uint8_t protocol_id;
    uint32_t timestamp;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_roll;
    int16_t gyro_pitch;
    int16_t gyro_yaw;
    uint8_t button_state;
} __attribute__((packed));

// Global variables
lv_obj_t* fish_objects[MAX_FISH] = {nullptr};
lv_obj_t* background_tiles[9] = {nullptr};
FishData current_fish[MAX_FISH];
uint8_t active_fish_count = 0;
uint8_t serial_buffer[SERIAL_BUFFER_SIZE];
size_t buffer_pos = 0;

// Flag to indicate if display needs update
volatile bool display_needs_update = false;
volatile bool scroll_needs_update = false;

// IMU and button state
uint8_t button_a_state = 0;
uint8_t button_b_state = 0;

// Pre-allocate IMU packet to avoid repeated allocations
IMUDataPacket imu_packet_buffer;

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
void send_imu_data_fast();
void IRAM_ATTR process_imu_sampling();

// Circular boundary validation
bool is_point_in_circle(int32_t x, int32_t y) {
    int32_t dx = x - CIRCLE_CENTER_X;
    int32_t dy = y - CIRCLE_CENTER_Y;
    return (dx * dx + dy * dy) <= (CIRCLE_RADIUS * CIRCLE_RADIUS);
}

// Get fish image based on sprite ID and direction
const lv_image_dsc_t* get_fish_image(uint8_t sprite_id, int16_t direction) {
    const lv_image_dsc_t** fish_images;
    if (sprite_id < 128) {
        fish_images = fish_type_01_images;
    } else {
        fish_images = fish_type_02_images;
    }
    
    int frame = (direction + 45) / 90;
    frame = frame % NUM_FISH_FRAMES;
    
    return fish_images[frame];
}

// Create fish objects as LVGL image objects
void create_fish_objects() {
    for (int i = 0; i < MAX_FISH; i++) {
        fish_objects[i] = lv_image_create(lv_screen_active());
        lv_image_set_src(fish_objects[i], &IMID0x01_0);
        lv_obj_add_flag(fish_objects[i], LV_OBJ_FLAG_HIDDEN);
    }
}

// Optimized fish drawing - only update when needed
void draw_fish_images() {
    static uint8_t prev_fish_count = 0;
    static int32_t prev_scroll_x = 0;
    static int32_t prev_scroll_y = 0;
    
    // Check if update is really needed
    if (active_fish_count == prev_fish_count && 
        scroll_offset_x == prev_scroll_x && 
        scroll_offset_y == prev_scroll_y &&
        !display_needs_update) {
        return;
    }
    
    prev_fish_count = active_fish_count;
    prev_scroll_x = scroll_offset_x;
    prev_scroll_y = scroll_offset_y;
    
    // Hide all fish objects first
    for (int i = 0; i < MAX_FISH; i++) {
        lv_obj_add_flag(fish_objects[i], LV_OBJ_FLAG_HIDDEN);
    }
    
    // Show and position active fish
    for (int i = 0; i < active_fish_count && i < MAX_FISH; i++) {
        if (!is_point_in_circle(current_fish[i].x, current_fish[i].y)) {
            continue;
        }
        
        const lv_image_dsc_t* fish_img = get_fish_image(current_fish[i].sprite_id, current_fish[i].direction);
        lv_image_set_src(fish_objects[i], fish_img);
        
        lv_coord_t img_w = fish_img->header.w;
        lv_coord_t img_h = fish_img->header.h;
        
        int32_t display_x = current_fish[i].x - scroll_offset_x;
        int32_t display_y = current_fish[i].y - scroll_offset_y;
        
        if (!is_point_in_circle(display_x, display_y)) {
            continue;
        }
        
        lv_obj_set_pos(fish_objects[i], 
                       display_x - img_w / 2, 
                       display_y - img_h / 2);
        
        lv_obj_clear_flag(fish_objects[i], LV_OBJ_FLAG_HIDDEN);
    }
    
    display_needs_update = false;
}

// Process Protocol ID 0x02 fish data packet
bool process_fish_packet(const uint8_t* data, size_t length) {
    if (length < 6) return false;
    
    ProtocolPacket* packet = (ProtocolPacket*)data;
    
    if (packet->protocol_id != 0x02) return false;
    if (packet->fish_count > MAX_FISH) return false;
    
    size_t expected_size = 6 + (packet->fish_count * sizeof(FishData));
    if (length < expected_size) return false;
    
    // Update fish data
    active_fish_count = packet->fish_count;
    memcpy(current_fish, packet->fish, packet->fish_count * sizeof(FishData));
    
    // Mark display for update
    display_needs_update = true;
    
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
    
    // Mark for update
    scroll_needs_update = true;
    display_needs_update = true;
    
    return true;
}

// Optimized IMU data sending
void IRAM_ATTR send_imu_data_fast() {
    // Pre-fill static parts of packet
    imu_packet_buffer.protocol_id = 0x01;
    imu_packet_buffer.timestamp = millis();
    
    // Get IMU data
    getAccelerometer();
    getGyroscope();
    
    // Convert to 16-bit integers
    imu_packet_buffer.accel_x = (int16_t)(Accel.x * 8191.75f);
    imu_packet_buffer.accel_y = (int16_t)(Accel.y * 8191.75f);
    imu_packet_buffer.accel_z = (int16_t)(Accel.z * 8191.75f);
    imu_packet_buffer.gyro_roll = (int16_t)(Gyro.x * 16.38f);
    imu_packet_buffer.gyro_pitch = (int16_t)(Gyro.y * 16.38f);
    imu_packet_buffer.gyro_yaw = (int16_t)(Gyro.z * 16.38f);
    imu_packet_buffer.button_state = (button_b_state << 1) | button_a_state;
    
    // Send packet directly
    Serial.write((uint8_t*)&imu_packet_buffer, sizeof(imu_packet_buffer));
}

// High-priority IMU sampling
void IRAM_ATTR process_imu_sampling() {
    uint32_t current_time = millis();
    
    if (current_time - last_imu_time >= IMU_SAMPLE_INTERVAL_MS) {
        send_imu_data_fast();
        last_imu_time = current_time;
    }
}

// Optimized serial data handler
void handle_serial_data() {
    // Process all available data at once
    size_t available = Serial.available();
    if (available == 0) return;
    
    // Read up to buffer capacity
    size_t to_read = min(available, size_t(sizeof(serial_buffer) - buffer_pos));
    size_t bytes_read = Serial.readBytes(serial_buffer + buffer_pos, to_read);
    buffer_pos += bytes_read;
    
    // Process complete packets
    size_t processed = 0;
    while (processed < buffer_pos) {
        if (buffer_pos - processed < 6) break; // Minimum packet size
        
        uint8_t* packet_start = serial_buffer + processed;
        
        if (packet_start[0] == 0x01) {  // Scroll command
            if (buffer_pos - processed >= sizeof(ScrollCommandPacket)) {
                if (process_scroll_command(packet_start, sizeof(ScrollCommandPacket))) {
                    processed += sizeof(ScrollCommandPacket);
                } else {
                    processed++; // Skip invalid byte
                }
            } else {
                break; // Need more data
            }
        } else if (packet_start[0] == 0x02) {  // Fish data
            if (buffer_pos - processed >= 6) {
                uint8_t fish_count = packet_start[5];
                size_t expected_size = 6 + (fish_count * sizeof(FishData));
                
                if (buffer_pos - processed >= expected_size) {
                    if (process_fish_packet(packet_start, expected_size)) {
                        processed += expected_size;
                    } else {
                        processed++; // Skip invalid byte
                    }
                } else {
                    break; // Need more data
                }
            } else {
                break; // Need more data
            }
        } else {
            processed++; // Skip unknown byte
        }
    }
    
    // Move unprocessed data to beginning of buffer
    if (processed > 0) {
        if (processed < buffer_pos) {
            memmove(serial_buffer, serial_buffer + processed, buffer_pos - processed);
        }
        buffer_pos -= processed;
    }
}

// Optimized background scrolling
void update_background_scroll() {
    if (!background_tiles[0] || !scroll_needs_update) return;
    
    const lv_image_dsc_t* bg_img = &background_480;
    lv_coord_t tile_width = bg_img->header.w;
    lv_coord_t tile_height = bg_img->header.h;
    
    int wrapped_offset_x = scroll_offset_x % tile_width;
    int wrapped_offset_y = scroll_offset_y % tile_height;
    
    if (wrapped_offset_x < 0) wrapped_offset_x += tile_width;
    if (wrapped_offset_y < 0) wrapped_offset_y += tile_height;
    
    // Update only visible tiles
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            int tile_index = row * 3 + col;
            
            int base_x = (col - 1) * tile_width + (ESP_PANEL_LCD_WIDTH - tile_width) / 2;
            int base_y = (row - 1) * tile_height + (ESP_PANEL_LCD_HEIGHT - tile_height) / 2;
            
            int final_x = base_x - wrapped_offset_x;
            int final_y = base_y - wrapped_offset_y;
            
            lv_obj_set_pos(background_tiles[tile_index], final_x, final_y);
        }
    }
    
    scroll_needs_update = false;
}

// Create tiled background
void create_tiled_background() {
    const lv_image_dsc_t* bg_img = &background_480;
    lv_coord_t tile_width = bg_img->header.w;
    lv_coord_t tile_height = bg_img->header.h;
    
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            int tile_index = row * 3 + col;
            
            background_tiles[tile_index] = lv_image_create(lv_screen_active());
            lv_image_set_src(background_tiles[tile_index], bg_img);
            
            int pos_x = (col - 1) * tile_width + (ESP_PANEL_LCD_WIDTH - tile_width) / 2;
            int pos_y = (row - 1) * tile_height + (ESP_PANEL_LCD_HEIGHT - tile_height) / 2;
            
            lv_obj_set_pos(background_tiles[tile_index], pos_x, pos_y);
            lv_obj_move_background(background_tiles[tile_index]);
        }
    }
}

// Create fish display system
void create_fish_display() {
    create_tiled_background();
    create_fish_objects();
    
    lv_obj_t* title = lv_label_create(lv_screen_active());
    // lv_label_set_text(title, "Fish Display System - Optimized");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
}

void Driver_Init() {
    Serial.println("Driver_Inité–‹å§‹");
    
    I2C_Init();
    delay(120);
    
    TCA9554PWR_Init(0x00);
    Set_EXIO(8, LOW);
    
    Backlight_Init();
    Set_Backlight(100);
    
    Serial.println("Driver_Initå®Œäº†");
}

void setup() {
    // Initialize serial with higher baudrate
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setRxBufferSize(SERIAL_BUFFER_SIZE);
    
    Serial.println("ESP32-S3 Fish Display System Starting (Optimized)...");
    
    Serial.printf("Total heap: %d\n", ESP.getHeapSize()); 
    Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
    Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
    Serial.printf("Free PSRAM: %d\n", ESP.getFreePsram());
    
    // Initialize drivers
    Driver_Init();              
    LCD_Init();                 
    Lvgl_Init();
    
    // Initialize IMU
    Serial.println("Initializing IMU...");
    QMI8658_Init();
    delay(100);
    
    // Create fish display UI
    create_fish_display();
    
    // Initialize timing
    last_imu_time = millis();
    last_lvgl_time = millis();
    last_fish_update_time = millis();
    
    Serial.println("=== Fish Display System Ready (Optimized) ===");
    Serial.printf("Serial: %d baud, 8N1\n", SERIAL_BAUDRATE);
    Serial.printf("IMU Rate: %d Hz\n", IMU_SAMPLE_RATE_HZ);
    Serial.printf("Display Rate: %d Hz\n", LVGL_UPDATE_RATE_HZ);
    Serial.printf("Display: %dx%d circular\n", ESP_PANEL_LCD_WIDTH, ESP_PANEL_LCD_HEIGHT);
    
    Serial.printf("Free heap after init: %d\n", ESP.getFreeHeap());
    Serial.printf("Free PSRAM after init: %d\n", ESP.getFreePsram());
}

void loop() {
    uint32_t current_time = millis();
    
    // High priority: IMU sampling (50Hz)
    process_imu_sampling();
    
    // Medium priority: Serial data processing
    handle_serial_data();
    
    // Low priority: LVGL updates (30Hz)
    if (current_time - last_lvgl_time >= LVGL_UPDATE_INTERVAL_MS) {
        // Update background if needed
        if (scroll_needs_update) {
            update_background_scroll();
        }
        
        // Update fish display if needed
        if (display_needs_update) {
            draw_fish_images();
        }
        
        // Handle LVGL tasks
        lv_timer_handler();
        
        last_lvgl_time = current_time;
    }
    
    // Minimal delay to prevent watchdog
    delayMicroseconds(100);
}