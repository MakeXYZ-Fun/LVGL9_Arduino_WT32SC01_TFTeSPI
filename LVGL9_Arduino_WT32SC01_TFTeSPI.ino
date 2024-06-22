/*Using LVGL with Arduino requires some extra steps:
 *Be sure to read the docs here: https://docs.lvgl.io/master/integration/framework/arduino.html  */

/*
  Libs:
    TFT_eSPI 2.5.43 (newest)
    lvgl 9.1.0 (newest)
*/


#include <lvgl.h>
#include "FT6336U.h"

#if LV_USE_TFT_ESPI
  #include <TFT_eSPI.h>
  TFT_eSPI tft = TFT_eSPI();
#endif

#define I2C_SDA 18
#define I2C_SCL 19

FT6336U ft6336u(I2C_SDA, I2C_SCL);
FT6336U_TouchPointType tp;

/*Set to your screen resolution and rotation*/
#define TFT_HOR_RES   480
#define TFT_VER_RES   320
#define TFT_ROTATION  LV_DISPLAY_ROTATION_0

/*LVGL draw into this buffer, 1/10 screen size usually works well. The size is in bytes*/
#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

#if LV_USE_LOG != 0
void my_print( lv_log_level_t level, const char * buf )
{
    LV_UNUSED(level);
    Serial.println(buf);
    Serial.flush();
}
#endif

/*use Arduinos millis() as tick source*/
static uint32_t my_tick(void)
{
    return millis();
}

/* LVGL calls it when a rendered image needs to copied to the display */
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint16_t * px_map);

/* Read the touchpad */
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data );

/* Button Demo */
void lv_button_demo(void);

/* Counter button event handler */
static void counter_event_handler(lv_event_t * e);

/* Toggle button event handler */
static void toggle_event_handler(lv_event_t * e);

////////////////////////////////////////////////////
void setup() {
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.begin( 115200 );
    Serial.println( LVGL_Arduino );

    // Enable TFT
    tft.begin();
    tft.setRotation(1);

    ft6336u.begin();

    Serial.print("FT6336U Firmware Version: ");
    Serial.println(ft6336u.read_firmware_id());
    Serial.print("FT6336U Device Mode: ");
    Serial.println(ft6336u.read_device_mode());

    lv_init();

    /*Set a tick source so that LVGL will know how much time elapsed. */
    lv_tick_set_cb(my_tick);

    /* register print function for debugging */
    #if LV_USE_LOG != 0
        lv_log_register_print_cb( my_print );
    #endif
    
        lv_display_t * disp;
    #if LV_USE_TFT_ESPI
        /*TFT_eSPI can be enabled lv_conf.h to initialize the display in a simple way*/
        disp = lv_tft_espi_create(TFT_HOR_RES, TFT_VER_RES, draw_buf, sizeof(draw_buf));
        lv_display_set_rotation(disp, TFT_ROTATION);
    
    #else
        /*Else create a display yourself*/
        disp = lv_display_create(TFT_HOR_RES, TFT_VER_RES);
        lv_display_set_flush_cb(disp, my_disp_flush);
        lv_display_set_buffers(disp, draw_buf, NULL, sizeof(draw_buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
    #endif

    /*Initialize the (dummy) input device driver*/
    lv_indev_t * indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    lv_indev_set_read_cb(indev, my_touchpad_read);

    Serial.println( "Setup done" );

    lv_button_demo();
}

////////////////////////////////////////////////////
void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    delay(5); /* let this time pass */
}

////////////////////////////////////////////////////
/* LVGL calls it when a rendered image needs to copied to the display */
void my_disp_flush( lv_display_t *disp, const lv_area_t *area, uint16_t * px_map)
{
    /*Copy `px map` to the `area`*/
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors(px_map, w * h, true);
    tft.endWrite();
    
    /*Call it to tell LVGL you are ready*/
    lv_display_flush_ready(disp);
}

/* Read the touchpad */
void my_touchpad_read( lv_indev_t * indev, lv_indev_data_t * data )
{
    tp = ft6336u.scan();

    if(!tp.touch_count) {
        data->state = LV_INDEV_STATE_RELEASED;
    } else {
        data->state = LV_INDEV_STATE_PRESSED;

        data->point.x = TFT_HOR_RES - tp.tp[0].y;
        data->point.y = tp.tp[0].x;

//        char tempString[128];
//        sprintf(tempString, "FT6336U TD Count %d / TD1 (%d, %4d, %4d) / TD2 (%d, %4d, %4d)\r", tp.touch_count, tp.tp[0].status, tp.tp[0].x, tp.tp[0].y, tp.tp[1].status, tp.tp[1].x, tp.tp[1].y);
//        Serial.print(tempString);
//        Serial.println("");

        Serial.printf("Touch (x,y): (%03d,%03d)\n",data->point.x,data->point.y );
    }
}

/* Counter button event handler */
static void counter_event_handler(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  // Safe casting using lv_event_get_target()
  lv_obj_t *btn = (lv_obj_t*)lv_event_get_target(e); 

  if (code == LV_EVENT_CLICKED) {
    static uint8_t cnt = 0;
    cnt++; 

    // Get the label directly (no need for get_child)
    lv_obj_t *label = lv_obj_get_child(btn, 0); 

    // Update the label text
    lv_label_set_text_fmt(label, "Button: %d", cnt);

    LV_LOG_USER("Clicked");
    Serial.println("Clicked");
  }
}

/* Toggle button event handler */
static void toggle_event_handler(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  // Safe casting using lv_event_get_target()
  lv_obj_t *btn = (lv_obj_t*)lv_event_get_target(e);

  if (code == LV_EVENT_VALUE_CHANGED) {
    if (lv_obj_has_state(btn, LV_STATE_CHECKED)) {
      LV_LOG_USER("Toggled ON");
      Serial.println("Toggled ON");
    } else {
      LV_LOG_USER("Toggled OFF");
      Serial.println("Toggled OFF");
    }
  }
}

void lv_button_demo(void) {
  lv_obj_t *label = lv_label_create( lv_screen_active() );
  lv_label_set_text( label, "Hello Arduino, I'm LVGL 9.1.0!" );
  lv_obj_align( label, LV_ALIGN_CENTER, 0, 20 );

  // Button with counter
  lv_obj_t *btn1 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn1, counter_event_handler, LV_EVENT_ALL, NULL);
  lv_obj_set_pos(btn1, 100, 100);  
  lv_obj_set_size(btn1, 120, 50);  

  label = lv_label_create(btn1);
  lv_label_set_text(label, "Button");
  lv_obj_center(label);

  // Toggle button
  lv_obj_t *btn2 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn2, toggle_event_handler, LV_EVENT_ALL, NULL);
  lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_pos(btn2, 250, 100); 
  lv_obj_set_size(btn2, 120, 50);

  label = lv_label_create(btn2);
  lv_label_set_text(label, "Toggle Button");
  lv_obj_center(label);
}
