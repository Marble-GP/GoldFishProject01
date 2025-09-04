/*****************************************************************************
  | File        :   LVGL_Driver.c
  
  | help        : 
    The provided LVGL library file must be installed first
******************************************************************************/
#include "LVGL_Driver.h"

static lv_display_t* display;
void* buf1 = NULL;
void* buf2 = NULL;
// static lv_color_t buf1[ LVGL_BUF_LEN ];
// static lv_color_t buf2[ LVGL_BUF_LEN ];
// static lv_color_t* buf1 = (lv_color_t*) heap_caps_malloc(LVGL_BUF_LEN, MALLOC_CAP_SPIRAM);
// static lv_color_t* buf2 = (lv_color_t*) heap_caps_malloc(LVGL_BUF_LEN, MALLOC_CAP_SPIRAM);
    


/* Serial debugging */
void Lvgl_print(const char * buf)
{
    // Serial.printf(buf);
    // Serial.flush();
}

/*  Display flushing 
    Displays LVGL content on the LCD
    This function implements associating LVGL data to the LCD screen
*/
void Lvgl_Display_LCD(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  LCD_addWindow(area->x1, area->y1, area->x2, area->y2, px_map);
  lv_display_flush_ready(disp);
}
/*Read the touchpad*/
void Lvgl_Touchpad_Read(lv_indev_t *indev, lv_indev_data_t *data)
{
  if (Simulated_touch_data.points != 0x00) {
    data->point.x = Simulated_touch_data.x;
    data->point.y = Simulated_touch_data.y;
    data->state = LV_INDEV_STATE_PRESSED;
    // printf("LVGL : X=%u Y=%u points=%d\r\n",  Simulated_touch_data.x , Simulated_touch_data.y,Simulated_touch_data.points);
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
  Simulated_touch_data.x = 0;
  Simulated_touch_data.y = 0;
  Simulated_touch_data.points = 0;
}
void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}
void Lvgl_Init(void)
{
  lv_init();
  
  buf1 = heap_caps_malloc(LVGL_BUF_LEN, MALLOC_CAP_SPIRAM);
  buf2 = heap_caps_malloc(LVGL_BUF_LEN, MALLOC_CAP_SPIRAM);

  display = lv_display_create(LVGL_WIDTH, LVGL_HEIGHT);
  lv_display_set_buffers(display, buf1, buf2, LVGL_BUF_LEN, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(display, Lvgl_Display_LCD);

  /*Initialize the (dummy) input device driver*/
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, Lvgl_Touchpad_Read);

  /* Create simple label */
  // lv_obj_t *label = lv_label_create(lv_screen_active());
  // lv_label_set_text(label, "Hello Arduino and LVGL!");
  // lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);
}
void Lvgl_Loop(void)
{
  lv_timer_handler(); /* let the GUI do its work */
  // delay( 5 );
}
