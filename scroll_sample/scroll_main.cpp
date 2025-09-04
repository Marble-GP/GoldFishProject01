#include <Arduino.h>
#include <lvgl.h>
#include <cmath>
#include "I2C_Driver.h"
#include "TCA9554PWR.h"
#include "Display_ST7701.h"
#include "LVGL_Driver.h"
#include "Gyro_QMI8658.h"

// IMU direction detection system configuration
#define SAMPLE_RATE_HZ 100
#define SAMPLE_INTERVAL_MS (1000 / SAMPLE_RATE_HZ)
// Filter order no longer needed with C++ class
#define PEAK_THRESHOLD 0.15f
#define MIN_PEAK_INTERVAL_MS 50
#define MAX_DETECTION_WINDOW_MS 2000

// Display configuration for circular LCD
#define ARROW_SIZE 80
#define ARROW_CENTER_X 240
#define ARROW_CENTER_Y 240
#define CIRCULAR_RADIUS 200
#define SAFE_ZONE_RADIUS 180

// Virtual scrolling configuration
#define RECT_SIZE 60
#define RECT_INITIAL_X 240
#define RECT_INITIAL_Y 240
// Tilt-based scrolling configuration
#define TILT_THRESHOLD 0.2f          // Minimum tilt angle (g) to start sliding
#define SLIDING_FORCE_SCALE 1000.0f   // Force proportional to tilt angle
#define VELOCITY_DECAY_FACTOR 0.95f  // 1st order LPF for inertia simulation
#define STILLNESS_THRESHOLD 0.05f    // Highpass filter threshold for stillness detection

// Improved C++ Bandpass Filter Class (1st order highpass + 1st order lowpass)
class BandpassFilter {
private:
    // サンプリング周期
    static constexpr float Ts = 0.01f;  // 100Hz sampling rate
    
    // フィルタ係数（1次ハイパス + 1次ローパス）
    // ハイパスフィルタ係数 (1Hz cutoff)
    float hp_a1;
    float hp_b0;
    float hp_b1;
    
    // ローパスフィルタ係数 (10Hz cutoff)  
    float lp_a1;
    float lp_b0;
    float lp_b1;
    
    // フィルタ状態変数
    float hp_x_prev;  // ハイパス入力履歴
    float hp_y_prev;  // ハイパス出力履歴
    float lp_x_prev;  // ローパス入力履歴
    float lp_y_prev;  // ローパス出力履歴
    
public:
    BandpassFilter() {
        init();
    }
    
    void init() {
        // カットオフ周波数 [rad/s]
        const float omega_hp = 2.0f * M_PI * 1.0f;   // 1Hz
        const float omega_lp = 2.0f * M_PI * 10.0f;  // 10Hz
        
        // オイラー法による離散化
        // ハイパスフィルタ H(s) = s / (s + omega_hp)
        float denominator_hp = 1.0f + omega_hp * Ts;
        hp_b0 = 1.0f / denominator_hp;
        hp_b1 = -1.0f / denominator_hp;
        hp_a1 = -1.0f / denominator_hp;
        
        // ローパスフィルタ H(s) = omega_lp / (s + omega_lp)  
        float denominator_lp = 1.0f + omega_lp * Ts;
        lp_b0 = omega_lp * Ts / denominator_lp;
        lp_b1 = 0.0f;
        lp_a1 = -1.0f / denominator_lp;
        
        // 状態変数の初期化
        reset();
    }
    
    void reset() {
        hp_x_prev = 0.0f;
        hp_y_prev = 0.0f;
        lp_x_prev = 0.0f;
        lp_y_prev = 0.0f;
    }
    
    float apply(float input) {
        // 1次ハイパスフィルタ
        float hp_output = hp_b0 * input + hp_b1 * hp_x_prev + hp_a1 * hp_y_prev;
        
        // 状態更新
        hp_x_prev = input;
        hp_y_prev = hp_output;
        
        // 1次ローパスフィルタ（ハイパス出力を入力として）
        float lp_output = lp_b0 * hp_output + lp_b1 * lp_x_prev + lp_a1 * lp_y_prev;
        
        // 状態更新
        lp_x_prev = hp_output;
        lp_y_prev = lp_output;
        
        return lp_output;
    }
};

// Peak detection structure
struct PeakDetector {
    float last_value;
    float peak_value;
    uint32_t last_peak_time;
    bool peak_detected;
    int peak_count;
    float first_peak_direction[2];  // x, y direction of first peak
};

// Tilt-based sliding physics simulation
class TiltScrollingFilter {
private:
    float velocity_x;
    float velocity_y;
    static constexpr float decay_factor = VELOCITY_DECAY_FACTOR;
    
public:
    TiltScrollingFilter() : velocity_x(0.0f), velocity_y(0.0f) {}
    
    void updateFromTilt(float tilt_x, float tilt_y, bool is_still) {
        if (is_still) {
            // Calculate sliding force from tilt (gravity component)
            float force_x = 0.0f;
            float force_y = 0.0f;
            
            // Device coordinate system: X=up, Y=right
            // LCD coordinate system: X=right, Y=down
            // Coordinate transformation: 
            // Device X (up) -> LCD Y (but inverted, up tilt = slide up on screen)
            // Device Y (right) -> LCD X (right tilt = slide right on screen)
            
            float tilt_magnitude = sqrt(tilt_x * tilt_x + tilt_y * tilt_y);
            if (tilt_magnitude > TILT_THRESHOLD) {
                // Apply sliding force proportional to tilt angle
                force_x = tilt_y * SLIDING_FORCE_SCALE;   // Device Y -> LCD X
                force_y = -tilt_x * SLIDING_FORCE_SCALE;  // Device X -> LCD Y (inverted)
            }
            
            // Apply forces to velocity with 1st order LPF (inertia simulation)
            velocity_x = velocity_x * decay_factor + force_x * (1.0f - decay_factor);
            velocity_y = velocity_y * decay_factor + force_y * (1.0f - decay_factor);
        } else {
            // Not still - just apply decay (no new forces)
            velocity_x *= decay_factor;
            velocity_y *= decay_factor;
        }
    }
    
    void update() {
        // This method is no longer used - updateFromTilt handles everything
    }
    
    float getVelX() const { return velocity_x; }
    float getVelY() const { return velocity_y; }
    
    void reset() {
        velocity_x = 0.0f;
        velocity_y = 0.0f;
    }
    
    float getCurrentMagnitude() const { return sqrt(velocity_x * velocity_x + velocity_y * velocity_y); }
    float getTiltForce(float tilt_x, float tilt_y) const {
        float tilt_magnitude = sqrt(tilt_x * tilt_x + tilt_y * tilt_y);
        return (tilt_magnitude > TILT_THRESHOLD) ? tilt_magnitude : 0.0f;
    }
};

// Global variables
BandpassFilter filter_x, filter_y;
PeakDetector peak_x, peak_y;
TiltScrollingFilter tilt_filter;
lv_obj_t* arrow_obj = nullptr;
lv_obj_t* status_label = nullptr;
lv_obj_t* debug_label = nullptr;
lv_obj_t* scroll_rect = nullptr;  // Virtual scrolling rectangle
uint32_t last_sample_time = 0;
float detected_direction[2] = {0.0f, 0.0f};  // x, y direction vector
bool direction_detected = false;
float rect_pos_x = RECT_INITIAL_X;
float rect_pos_y = RECT_INITIAL_Y;

// Function declarations
void init_peak_detector(PeakDetector* detector);
bool detect_peak(PeakDetector* detector, float filtered_value, uint32_t timestamp);
void process_imu_data();
void draw_arrow(float dir_x, float dir_y);
void update_virtual_scroll();
bool is_rect_in_bounds(float x, float y);
void create_ui();
void update_debug_display(float acc_x, float acc_y, float filt_x, float filt_y);

// Filter initialization and application now handled by C++ class

// Initialize peak detector
void init_peak_detector(PeakDetector* detector) {
    detector->last_value = 0.0f;
    detector->peak_value = 0.0f;
    detector->last_peak_time = 0;
    detector->peak_detected = false;
    detector->peak_count = 0;
    detector->first_peak_direction[0] = 0.0f;
    detector->first_peak_direction[1] = 0.0f;
}

// Detect peaks in filtered acceleration data
bool detect_peak(PeakDetector* detector, float filtered_value, uint32_t timestamp) {
    bool peak_found = false;
    
    // Check for peak (local maximum above threshold)
    if (fabs(filtered_value) > PEAK_THRESHOLD) {
        if (fabs(filtered_value) > fabs(detector->peak_value)) {
            detector->peak_value = filtered_value;
        }
    } else if (fabs(detector->peak_value) > PEAK_THRESHOLD) {
        // Peak ended, check if it's a valid peak
        uint32_t time_since_last = timestamp - detector->last_peak_time;
        if (time_since_last > MIN_PEAK_INTERVAL_MS) {
            detector->peak_detected = true;
            detector->last_peak_time = timestamp;
            detector->peak_count++;
            peak_found = true;
            Serial.print("Peak detected! Value: ");
            Serial.print(detector->peak_value, 3);
            Serial.print(", Count: ");
            Serial.println(detector->peak_count);
        }
        detector->peak_value = 0.0f;
    }
    
    detector->last_value = filtered_value;
    return peak_found;
}

// Process IMU data for direction detection
void process_imu_data() {
    uint32_t current_time = millis();
    
    // Sample at specified rate
    if (current_time - last_sample_time < SAMPLE_INTERVAL_MS) {
        return;
    }
    last_sample_time = current_time;
    
    // Get accelerometer data
    getAccelerometer();
    
    // Apply bandpass filtering to x and y acceleration using C++ class
    float filtered_x = filter_x.apply(Accel.x);
    float filtered_y = filter_y.apply(Accel.y);
    
    // Detect peaks in both axes
    bool peak_x_found = detect_peak(&peak_x, filtered_x, current_time);
    bool peak_y_found = detect_peak(&peak_y, filtered_y, current_time);
    
    // Tilt-based scrolling: Check for stillness using highpass filter magnitude
    float highpass_magnitude = sqrt(filtered_x * filtered_x + filtered_y * filtered_y);
    bool is_still = (highpass_magnitude < STILLNESS_THRESHOLD);
    
    // Get tilt from raw acceleration (gravity component when still)
    float tilt_x = Accel.x;  // Device X axis (up direction)
    float tilt_y = Accel.y;  // Device Y axis (right direction)
    
    // Update tilt-based scrolling
    tilt_filter.updateFromTilt(tilt_x, tilt_y, is_still);
    
    // Direction detection (for arrow display)
    if (highpass_magnitude > PEAK_THRESHOLD && !direction_detected) {
        detected_direction[0] = filtered_x;
        detected_direction[1] = filtered_y;
        
        direction_detected = true;
        draw_arrow(detected_direction[0], detected_direction[1]);
        
        Serial.print("Direction detected: X=");
        Serial.print(detected_direction[0], 3);
        Serial.print(", Y=");
        Serial.print(detected_direction[1], 3);
        Serial.print(", HP_Mag=");
        Serial.println(highpass_magnitude, 3);
    }
    
    // Reset detection after timeout
    static uint32_t detection_start_time = 0;
    if (peak_x.peak_count > 0 || peak_y.peak_count > 0) {
        if (detection_start_time == 0) {
            detection_start_time = current_time;
        } else if (current_time - detection_start_time > MAX_DETECTION_WINDOW_MS) {
            // Reset for next detection cycle
            init_peak_detector(&peak_x);
            init_peak_detector(&peak_y);
            direction_detected = false;
            detection_start_time = 0;
            
            // Hide arrow
            if (arrow_obj) {
                lv_obj_add_flag(arrow_obj, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
    
    // Update virtual scrolling
    update_virtual_scroll();
    
    // Update debug display
    update_debug_display(Accel.x, Accel.y, filtered_x, filtered_y);
    
    // Add serial debug output every 500ms
    static uint32_t last_debug_time = 0;
    if (current_time - last_debug_time > 500) {
        Serial.print("Raw Accel: X=");
        Serial.print(Accel.x, 3);
        Serial.print(", Y=");
        Serial.print(Accel.y, 3);
        Serial.print(", Z=");
        Serial.print(Accel.z, 3);
        Serial.print(" | Filtered: X=");
        Serial.print(filtered_x, 3);
        Serial.print(", Y=");
        Serial.print(filtered_y, 3);
        Serial.print(" | Tilt: X=");
        Serial.print(Accel.x, 3);
        Serial.print(", Y=");
        Serial.print(Accel.y, 3);
        Serial.print(" | Vel: X=");
        Serial.print(tilt_filter.getVelX(), 1);
        Serial.print(", Y=");
        Serial.print(tilt_filter.getVelY(), 1);
        Serial.print(" | Still: ");
        float hp_mag = sqrt(filtered_x*filtered_x + filtered_y*filtered_y);
        Serial.println((hp_mag < STILLNESS_THRESHOLD) ? "YES" : "NO");
        last_debug_time = current_time;
    }
}

// Draw arrow pointing in detected direction
void draw_arrow(float dir_x, float dir_y) {
    if (!arrow_obj) return;
    
    // Calculate angle from direction vector
    float angle_rad = atan2(dir_y, dir_x);
    int16_t angle_deg = (int16_t)(angle_rad * 180.0f / PI);
    
    // Show and rotate arrow
    lv_obj_clear_flag(arrow_obj, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_transform_angle(arrow_obj, angle_deg * 10, 0); // LVGL uses 0.1 degree units
    
    // Update status
    if (status_label) {
        lv_label_set_text_fmt(status_label, "Direction: %d°", angle_deg);
    }
}

// Update debug display with current values
void update_debug_display(float acc_x, float acc_y, float filt_x, float filt_y) {
    if (debug_label) {
        static char debug_text[120];
        snprintf(debug_text, sizeof(debug_text), "Raw: X=%.2f Y=%.2f\nFilt: X=%.2f Y=%.2f\nPos: X=%.0f Y=%.0f", 
                 acc_x, acc_y, filt_x, filt_y, rect_pos_x, rect_pos_y);
        lv_label_set_text(debug_label, debug_text);
    }
}

// Update virtual scrolling rectangle position
void update_virtual_scroll() {
    // Get current velocities from tilt filter
    float vel_x = tilt_filter.getVelX();
    float vel_y = tilt_filter.getVelY();
    
    // Update rectangle position based on velocity (integration with time delta)
    float dt = SAMPLE_INTERVAL_MS / 1000.0f;  // Convert ms to seconds
    rect_pos_x += vel_x * dt;
    rect_pos_y += vel_y * dt;
    
    // Check boundaries and bounce if necessary
    if (!is_rect_in_bounds(rect_pos_x, rect_pos_y)) {
        // Simple boundary reflection - reverse velocity components that go out of bounds
        float center_x = ARROW_CENTER_X;
        float center_y = ARROW_CENTER_Y;
        float dx = rect_pos_x - center_x;
        float dy = rect_pos_y - center_y;
        float distance = sqrt(dx*dx + dy*dy);
        
        if (distance > SAFE_ZONE_RADIUS - RECT_SIZE/2) {
            // Reflect position back into bounds
            float scale = (SAFE_ZONE_RADIUS - RECT_SIZE/2) / distance;
            rect_pos_x = center_x + dx * scale;
            rect_pos_y = center_y + dy * scale;
            
            // Reduce velocity significantly when hitting boundary
            tilt_filter.reset();
        }
    }
    
    // Update rectangle position on screen
    if (scroll_rect) {
        lv_obj_set_pos(scroll_rect, 
                      (int)(rect_pos_x - RECT_SIZE/2), 
                      (int)(rect_pos_y - RECT_SIZE/2));
    }
}

// Check if rectangle position is within circular bounds
bool is_rect_in_bounds(float x, float y) {
    float center_x = ARROW_CENTER_X;
    float center_y = ARROW_CENTER_Y;
    float dx = x - center_x;
    float dy = y - center_y;
    float distance = sqrt(dx*dx + dy*dy);
    return distance <= (SAFE_ZONE_RADIUS - RECT_SIZE/2);
}

// Create UI elements optimized for circular LCD
void create_ui() {
    // Create background
    lv_obj_t* background = lv_obj_create(lv_screen_active());
    lv_obj_set_size(background, ESP_PANEL_LCD_WIDTH, ESP_PANEL_LCD_HEIGHT);
    lv_obj_set_pos(background, 0, 0);
    lv_obj_set_style_bg_color(background, lv_color_black(), 0);
    lv_obj_clear_flag(background, LV_OBJ_FLAG_SCROLLABLE);
    
    // Create circular border to show display boundary
    lv_obj_t* circle = lv_obj_create(lv_screen_active());
    lv_obj_set_size(circle, CIRCULAR_RADIUS * 2, CIRCULAR_RADIUS * 2);
    lv_obj_center(circle);
    lv_obj_set_style_bg_opa(circle, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(circle, lv_color_hex(0x404040), 0);
    lv_obj_set_style_border_width(circle, 2, 0);
    lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);
    
    // Create title - positioned at top of circle
    lv_obj_t* title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "Tilt Scrolling");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_align(title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -120);
    
    // Create scrolling rectangle
    scroll_rect = lv_obj_create(lv_screen_active());
    lv_obj_set_size(scroll_rect, RECT_SIZE, RECT_SIZE);
    lv_obj_set_style_bg_color(scroll_rect, lv_color_hex(0x0088FF), 0); // Blue color
    lv_obj_set_style_border_color(scroll_rect, lv_color_white(), 0);
    lv_obj_set_style_border_width(scroll_rect, 2, 0);
    lv_obj_set_style_radius(scroll_rect, 8, 0); // Rounded corners
    lv_obj_set_pos(scroll_rect, RECT_INITIAL_X - RECT_SIZE/2, RECT_INITIAL_Y - RECT_SIZE/2);
    
    // Create arrow (initially hidden) - smaller now
    arrow_obj = lv_label_create(lv_screen_active());
    lv_label_set_text(arrow_obj, "→");
    lv_obj_set_style_text_color(arrow_obj, lv_color_hex(0x00FF00), 0); // Green color
    lv_obj_set_style_text_font(arrow_obj, LV_FONT_DEFAULT, 0);
    lv_obj_align(arrow_obj, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_add_flag(arrow_obj, LV_OBJ_FLAG_HIDDEN);
    
    // Create status label - positioned at bottom of circle
    status_label = lv_label_create(lv_screen_active());
    lv_label_set_text(status_label, "Tilt to slide");
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
    lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(status_label, LV_ALIGN_CENTER, 0, 100);
    
    // Create debug label - smaller font, positioned in safe zone
    debug_label = lv_label_create(lv_screen_active());
    lv_label_set_text(debug_label, "Ready");
    lv_obj_set_style_text_color(debug_label, lv_color_hex(0x808080), 0);
    lv_obj_set_style_text_font(debug_label, LV_FONT_DEFAULT, 0);
    lv_obj_align(debug_label, LV_ALIGN_CENTER, 0, 140);
    
    Serial.println("Virtual scrolling UI created for circular LCD");
}

// Driver initialization
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
    Serial.println("ESP32-S3 IMU Direction Detection Starting...");
    
    // Memory information
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
    delay(500);
    
    // Test IMU reading
    Serial.println("Testing IMU reading...");
    for (int i = 0; i < 5; i++) {
        getAccelerometer();
        Serial.print("IMU Test ");
        Serial.print(i+1);
        Serial.print(": X=");
        Serial.print(Accel.x, 3);
        Serial.print(", Y=");
        Serial.print(Accel.y, 3);
        Serial.print(", Z=");
        Serial.println(Accel.z, 3);
        delay(100);
    }
    
    // Initialize filters and detectors
    // C++ filter classes auto-initialize in constructor
    init_peak_detector(&peak_x);
    init_peak_detector(&peak_y);
    
    Serial.println("Bandpass filters initialized (1Hz-10Hz cascade)");
    
    // Create UI
    create_ui();
    
    Serial.println("=== IMU Direction Detection System Ready ===");
    Serial.println("Shake device horizontally to detect direction");
    Serial.printf("Sample rate: %d Hz\n", SAMPLE_RATE_HZ);
    Serial.printf("Peak threshold: %.2f\n", PEAK_THRESHOLD);
    
    Serial.printf("Free heap after init: %d\n", ESP.getFreeHeap());
    Serial.printf("Free PSRAM after init: %d\n", ESP.getFreePsram());
}

void loop() {
    // Handle LVGL tasks
    lv_timer_handler();
    
    // Process IMU data for direction detection
    process_imu_data();
    
    delay(5);
}