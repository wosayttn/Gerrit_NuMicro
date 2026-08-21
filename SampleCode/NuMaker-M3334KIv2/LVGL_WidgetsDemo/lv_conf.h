/**************************************************************************//**
 * @file     lv_conf.h
 * @brief    lvgl configuration
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef LV_CONF_H
#define LV_CONF_H

// <<< Use Configuration Wizard in Context Menu >>>

// <o> LV_USE_DEMO_WIDGETS - Widgets Demo
// <i> Enable the LVGL built-in widgets demo.
// <0=> Disabled
// <1=> Enabled
#define LV_USE_DEMO_WIDGETS             1

// <o> LV_USE_DEMO_BENCHMARK - Benchmark Demo
// <i> Enable the LVGL benchmark demo.
// <0=> Disabled
// <1=> Enabled
#define LV_USE_DEMO_BENCHMARK           1


// <e> LV_USE_PERF_MONITOR - Performance Monitor
// <i> Enable the FPS / CPU usage performance monitor.
#define LV_USE_PERF_MONITOR             1

//   <o> LV_USE_PERF_MONITOR_LOG_MODE - Output Mode
//   <i> Select where the performance data is reported.
//   <0=> Show overlay on LCD after benchmark completes (default)
//   <1=> Print via LV_LOG (suppress LCD overlay)
#define LV_USE_PERF_MONITOR_LOG_MODE    1

// </e>

// <o> LV_USE_LOG - LVGL Logging
// <i> Enable LVGL logging. Required for benchmark CSV output via LV_LOG().
// <0=> Disabled
// <1=> Enabled
#define LV_USE_LOG                      1

// <o> CONFIG_LV_USE_TOUCH_CURSOR - Touch Position Cursor
// <i> Renders a "+" symbol on the sys layer that tracks the touch position.
// <i> Useful for verifying touch accuracy.
// <0=> Disabled
// <1=> Enabled
#define CONFIG_LV_USE_TOUCH_CURSOR      1

// <<< end of configuration section >>>

#if defined(__320x240__)
    #define LV_HOR_RES_MAX                  320
    #define LV_VER_RES_MAX                  240
#elif defined(__480x272__)
    #define LV_HOR_RES_MAX                  480
    #define LV_VER_RES_MAX                  272
#endif

#define LV_USE_OS   LV_OS_FREERTOS
#define LV_COLOR_DEPTH                  16

/*Display refresh period in milliseconds. Controls max FPS:
 *   33ms = ~30 FPS (default)
 *   20ms = ~50 FPS
 *   16ms = ~60 FPS */
#define LV_DEF_REFR_PERIOD              33

#define LV_FONT_MONTSERRAT_12           1
#define LV_FONT_MONTSERRAT_16           0

#if defined (LV_USE_DEMO_BENCHMARK)
    /* LV_FONT_MONTSERRAT_14 is the default in lv_conf_internal.h, but the
     * benchmark references it unconditionally so make it explicit. */
    #define LV_FONT_MONTSERRAT_14           1
    #define LV_FONT_MONTSERRAT_20           1
    #define LV_FONT_MONTSERRAT_24           1
    #define LV_FONT_MONTSERRAT_26           1
#endif

#define LV_USE_SYSMON                   1


#if 0

    //LV_CONF_MINIMAL

    #undef LV_USE_DEMO_WIDGETS
    #undef LV_USE_SYSMON
    #undef LV_USE_PERF_MONITOR
    #undef LV_USE_LOG

    #define LV_USE_THEME_DEFAULT 1
    #define LV_WIDGETS_HAS_DEFAULT_VALUE     0
    #define LV_USE_ANIMIMG                   0
    #define LV_USE_ARC                       1
    #define LV_USE_BAR                       0
    #define LV_USE_BUTTON                    1
    #define LV_USE_BUTTONMATRIX              1
    #define LV_USE_CALENDAR                  0
    #define LV_USE_CALENDAR_HEADER_ARROW     0
    #define LV_USE_CALENDAR_HEADER_DROPDOWN  0
    #define LV_USE_CALENDAR_CHINESE          0
    #define LV_USE_CANVAS                    0
    #define LV_USE_CHART                     0
    #define LV_USE_CHECKBOX                  0
    #define LV_USE_DROPDOWN                  0
    #define LV_USE_IMAGE                     1
    #define LV_USE_IMAGEBUTTON               0
    #define LV_USE_KEYBOARD                  0
    #define LV_USE_LABEL                     1
    #define LV_LABEL_TEXT_SELECTION          0
    #define LV_LABEL_LONG_TXT_HINT           0
    #define LV_LABEL_WAIT_CHAR_COUNT         0
    #define LV_USE_LED                       0
    #define LV_USE_LINE                      1
    #define LV_USE_LIST                      0
    #define LV_USE_MENU                      0
    #define LV_USE_MSGBOX                    0
    #define LV_USE_ROLLER                    0
    #define LV_USE_SCALE                     1
    #define LV_USE_SLIDER                    0
    #define LV_USE_SPAN                      0
    #define LV_USE_SPINBOX                   0
    #define LV_USE_SPINNER                   0
    #define LV_USE_SWITCH                    0
    #define LV_USE_TEXTAREA                  0
    #define LV_USE_TABLE                     0
    #define LV_USE_TABVIEW                   0
    #define LV_USE_TILEVIEW                  0
    #define LV_USE_WIN                       0
    #define LV_DRAW_SW_SUPPORT_RGB565        1
    #define LV_DRAW_SW_SUPPORT_RGB565A8      0
    #define LV_DRAW_SW_SUPPORT_RGB888          1
    #define LV_DRAW_SW_SUPPORT_XRGB8888      0
    #define LV_DRAW_SW_SUPPORT_ARGB8888      1
    #define LV_DRAW_SW_SUPPORT_L8            0
    #define LV_DRAW_SW_SUPPORT_AL88          0
    #define LV_DRAW_SW_SUPPORT_A8            0
    #define LV_DRAW_SW_SUPPORT_I1            0
    #define LV_DRAW_SW_COMPLEX               1

#endif

#define CONFIG_LV_MEM_SIZE            (128*1024U)

#if LV_USE_LOG == 1
    //#define LV_LOG_LEVEL                    LV_LOG_LEVEL_TRACE
    //#define LV_LOG_LEVEL                    LV_LOG_LEVEL_INFO
    //#define LV_LOG_LEVEL                    LV_LOG_LEVEL_WARN
    //#define LV_LOG_LEVEL                    LV_LOG_LEVEL_ERROR
    /* LV_LOG_LEVEL_USER is required for benchmark CSV output via LV_LOG() */
    #define LV_LOG_LEVEL                    LV_LOG_LEVEL_USER
    //#define LV_LOG_LEVEL                    LV_LOG_LEVEL_NONE
#endif

#endif
