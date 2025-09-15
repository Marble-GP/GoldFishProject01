# ifndef IMDAT_INCLUDE_H

#define IMDAT_INCLUDE_H

#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif


extern const lv_image_dsc_t IMID0x01_0;

extern const lv_image_dsc_t IMID0x01_1;

extern const lv_image_dsc_t IMID0x01_2;

extern const lv_image_dsc_t IMID0x01_3;

extern const lv_image_dsc_t IMID0x02_0;

extern const lv_image_dsc_t IMID0x02_1;

extern const lv_image_dsc_t IMID0x02_2;

extern const lv_image_dsc_t IMID0x02_3;

// extern const lv_image_dsc_t background2;

extern const lv_image_dsc_t background_480;


#endif