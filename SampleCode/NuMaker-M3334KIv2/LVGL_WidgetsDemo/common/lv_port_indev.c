/**************************************************************************//**
 * @file     lv_port_disp.c
 * @brief    lvgl input device porting
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "lvgl.h"
#include "lv_glue.h"

static void input_read(lv_indev_t *indev, lv_indev_data_t *data)
{
    touchpad_device_read(data);
}

void lv_port_indev_init(void)
{
    static lv_indev_t *lv_indev_touch;

    LV_ASSERT(touchpad_device_initialize() == 0);
    LV_ASSERT(touchpad_device_open() == 0);

    lv_indev_touch = lv_indev_create();
    lv_indev_set_type(lv_indev_touch, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lv_indev_touch,  input_read);

#if defined(CONFIG_LV_USE_TOUCH_CURSOR) && (CONFIG_LV_USE_TOUCH_CURSOR == 1)
    /* Create a cursor object to visualize the touch position.
     * lv_indev_set_cursor() moves it to the sys layer and updates
     * its position automatically whenever the pointer moves. */
    lv_obj_t *cursor_obj = lv_label_create(lv_screen_active());
    lv_label_set_text(cursor_obj, LV_SYMBOL_PLUS);
    lv_obj_set_style_text_color(cursor_obj, lv_color_hex(0xFF0000), LV_PART_MAIN);
    lv_indev_set_cursor(lv_indev_touch, cursor_obj);
#endif
}
