set(LVGL_DEMO_DIR ${CMAKE_CURRENT_LIST_DIR})
set(FREERTOS_DIR  ${BSP_DIR}/ThirdParty/FreeRTOS)
set(LVGL_DIR      ${BSP_DIR}/ThirdParty/lvgl)

# ============================================================
# FreeRTOS library
# ============================================================
add_library(freertos_lib
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM33_NTZ/non_secure/port.c
    ${FREERTOS_DIR}/portable/GCC/ARM_CM33_NTZ/non_secure/portasm.c
)
target_include_directories(freertos_lib PUBLIC
    ${LVGL_DEMO_DIR}
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM33_NTZ/non_secure
)
target_compile_definitions(freertos_lib PUBLIC CPU_CORTEX_M33)

# ============================================================
# LVGL library (core src + demos)
# ============================================================
add_library(lvgl_lib
    # core
    ${LVGL_DIR}/src/lv_init.c
    ${LVGL_DIR}/src/core/lv_group.c
    ${LVGL_DIR}/src/core/lv_obj.c
    ${LVGL_DIR}/src/core/lv_obj_class.c
    ${LVGL_DIR}/src/core/lv_obj_draw.c
    ${LVGL_DIR}/src/core/lv_obj_event.c
    ${LVGL_DIR}/src/core/lv_obj_id_builtin.c
    ${LVGL_DIR}/src/core/lv_obj_pos.c
    ${LVGL_DIR}/src/core/lv_obj_property.c
    ${LVGL_DIR}/src/core/lv_obj_scroll.c
    ${LVGL_DIR}/src/core/lv_obj_style.c
    ${LVGL_DIR}/src/core/lv_obj_style_gen.c
    ${LVGL_DIR}/src/core/lv_obj_tree.c
    ${LVGL_DIR}/src/core/lv_refr.c
    # display
    ${LVGL_DIR}/src/display/lv_display.c
    # draw
    ${LVGL_DIR}/src/draw/lv_draw.c
    ${LVGL_DIR}/src/draw/lv_draw_3d.c
    ${LVGL_DIR}/src/draw/lv_draw_arc.c
    ${LVGL_DIR}/src/draw/lv_draw_buf.c
    ${LVGL_DIR}/src/draw/lv_draw_image.c
    ${LVGL_DIR}/src/draw/lv_draw_label.c
    ${LVGL_DIR}/src/draw/lv_draw_line.c
    ${LVGL_DIR}/src/draw/lv_draw_mask.c
    ${LVGL_DIR}/src/draw/lv_draw_rect.c
    ${LVGL_DIR}/src/draw/lv_draw_triangle.c
    ${LVGL_DIR}/src/draw/lv_draw_vector.c
    ${LVGL_DIR}/src/draw/lv_image_decoder.c
    ${LVGL_DIR}/src/draw/convert/lv_draw_buf_convert.c
    ${LVGL_DIR}/src/draw/convert/helium/lv_draw_buf_convert_helium.c
    ${LVGL_DIR}/src/draw/convert/neon/lv_draw_buf_convert_neon.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_arc.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_border.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_box_shadow.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_fill.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_grad.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_img.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_letter.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_line.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_mask.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_mask_rect.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_transform.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_triangle.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_utils.c
    ${LVGL_DIR}/src/draw/sw/lv_draw_sw_vector.c
    ${LVGL_DIR}/src/draw/sw/blend/lv_draw_sw_blend.c
    ${LVGL_DIR}/src/draw/sw/blend/lv_draw_sw_blend_to_al88.c
    ${LVGL_DIR}/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888.c
    ${LVGL_DIR}/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888_premultiplied.c
    ${LVGL_DIR}/src/draw/sw/blend/lv_draw_sw_blend_to_i1.c
    ${LVGL_DIR}/src/draw/sw/blend/lv_draw_sw_blend_to_l8.c
    ${LVGL_DIR}/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565.c
    ${LVGL_DIR}/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565_swapped.c
    ${LVGL_DIR}/src/draw/sw/blend/lv_draw_sw_blend_to_rgb888.c
    ${LVGL_DIR}/src/draw/sw/blend/neon/lv_draw_sw_blend_neon_to_rgb565.c
    ${LVGL_DIR}/src/draw/sw/blend/neon/lv_draw_sw_blend_neon_to_rgb888.c
    # font
    ${LVGL_DIR}/src/font/lv_binfont_loader.c
    ${LVGL_DIR}/src/font/lv_font.c
    ${LVGL_DIR}/src/font/lv_font_dejavu_16_persian_hebrew.c
    ${LVGL_DIR}/src/font/lv_font_fmt_txt.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_8.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_10.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_12.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_14.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_14_aligned.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_16.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_18.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_20.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_22.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_24.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_26.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_28.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_28_compressed.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_30.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_32.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_34.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_36.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_38.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_40.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_42.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_44.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_46.c
    ${LVGL_DIR}/src/font/lv_font_montserrat_48.c
    ${LVGL_DIR}/src/font/lv_font_source_han_sans_sc_14_cjk.c
    ${LVGL_DIR}/src/font/lv_font_source_han_sans_sc_16_cjk.c
    ${LVGL_DIR}/src/font/lv_font_unscii_8.c
    ${LVGL_DIR}/src/font/lv_font_unscii_16.c
    # indev
    ${LVGL_DIR}/src/indev/lv_indev.c
    ${LVGL_DIR}/src/indev/lv_indev_gesture.c
    ${LVGL_DIR}/src/indev/lv_indev_scroll.c
    # layouts
    ${LVGL_DIR}/src/layouts/lv_layout.c
    ${LVGL_DIR}/src/layouts/flex/lv_flex.c
    ${LVGL_DIR}/src/layouts/grid/lv_grid.c
    # libs
    ${LVGL_DIR}/src/libs/bin_decoder/lv_bin_decoder.c
    ${LVGL_DIR}/src/libs/bmp/lv_bmp.c
    ${LVGL_DIR}/src/libs/ffmpeg/lv_ffmpeg.c
    ${LVGL_DIR}/src/libs/fsdrv/lv_fs_cbfs.c
    ${LVGL_DIR}/src/libs/fsdrv/lv_fs_fatfs.c
    ${LVGL_DIR}/src/libs/fsdrv/lv_fs_frogfs.c
    ${LVGL_DIR}/src/libs/fsdrv/lv_fs_littlefs.c
    ${LVGL_DIR}/src/libs/fsdrv/lv_fs_memfs.c
    ${LVGL_DIR}/src/libs/fsdrv/lv_fs_posix.c
    ${LVGL_DIR}/src/libs/fsdrv/lv_fs_stdio.c
    ${LVGL_DIR}/src/libs/fsdrv/lv_fs_uefi.c
    ${LVGL_DIR}/src/libs/fsdrv/lv_fs_win32.c
    ${LVGL_DIR}/src/libs/gif/lv_gif.c
    ${LVGL_DIR}/src/libs/gif/AnimatedGIF/src/gif.c
    ${LVGL_DIR}/src/libs/libpng/lv_libpng.c
    ${LVGL_DIR}/src/libs/lodepng/lodepng.c
    ${LVGL_DIR}/src/libs/lodepng/lv_lodepng.c
    ${LVGL_DIR}/src/libs/lz4/lz4.c
    ${LVGL_DIR}/src/libs/tjpgd/lv_tjpgd.c
    ${LVGL_DIR}/src/libs/tjpgd/tjpgd.c
    # misc
    ${LVGL_DIR}/src/misc/lv_anim.c
    ${LVGL_DIR}/src/misc/lv_anim_timeline.c
    ${LVGL_DIR}/src/misc/lv_area.c
    ${LVGL_DIR}/src/misc/lv_array.c
    ${LVGL_DIR}/src/misc/lv_async.c
    ${LVGL_DIR}/src/misc/lv_bidi.c
    ${LVGL_DIR}/src/misc/lv_circle_buf.c
    ${LVGL_DIR}/src/misc/lv_color.c
    ${LVGL_DIR}/src/misc/lv_color_op.c
    ${LVGL_DIR}/src/misc/lv_event.c
    ${LVGL_DIR}/src/misc/lv_fs.c
    ${LVGL_DIR}/src/misc/lv_grad.c
    ${LVGL_DIR}/src/misc/lv_iter.c
    ${LVGL_DIR}/src/misc/lv_ll.c
    ${LVGL_DIR}/src/misc/lv_log.c
    ${LVGL_DIR}/src/misc/lv_lru.c
    ${LVGL_DIR}/src/misc/lv_math.c
    ${LVGL_DIR}/src/misc/lv_matrix.c
    ${LVGL_DIR}/src/misc/lv_palette.c
    ${LVGL_DIR}/src/misc/lv_profiler_builtin.c
    ${LVGL_DIR}/src/misc/lv_profiler_builtin_posix.c
    ${LVGL_DIR}/src/misc/lv_rb.c
    ${LVGL_DIR}/src/misc/lv_style.c
    ${LVGL_DIR}/src/misc/lv_style_gen.c
    ${LVGL_DIR}/src/misc/lv_templ.c
    ${LVGL_DIR}/src/misc/lv_text.c
    ${LVGL_DIR}/src/misc/lv_text_ap.c
    ${LVGL_DIR}/src/misc/lv_timer.c
    ${LVGL_DIR}/src/misc/lv_tree.c
    ${LVGL_DIR}/src/misc/lv_utils.c
    ${LVGL_DIR}/src/misc/cache/lv_cache.c
    ${LVGL_DIR}/src/misc/cache/lv_cache_entry.c
    ${LVGL_DIR}/src/misc/cache/class/lv_cache_lru_ll.c
    ${LVGL_DIR}/src/misc/cache/class/lv_cache_lru_rb.c
    ${LVGL_DIR}/src/misc/cache/class/lv_cache_sc_da.c
    ${LVGL_DIR}/src/misc/cache/instance/lv_image_cache.c
    ${LVGL_DIR}/src/misc/cache/instance/lv_image_header_cache.c
    # osal
    ${LVGL_DIR}/src/osal/lv_cmsis_rtos2.c
    ${LVGL_DIR}/src/osal/lv_freertos.c
    ${LVGL_DIR}/src/osal/lv_linux.c
    ${LVGL_DIR}/src/osal/lv_mqx.c
    ${LVGL_DIR}/src/osal/lv_os.c
    ${LVGL_DIR}/src/osal/lv_os_none.c
    ${LVGL_DIR}/src/osal/lv_pthread.c
    ${LVGL_DIR}/src/osal/lv_rtthread.c
    ${LVGL_DIR}/src/osal/lv_sdl2.c
    ${LVGL_DIR}/src/osal/lv_windows.c
    # others
    ${LVGL_DIR}/src/others/file_explorer/lv_file_explorer.c
    ${LVGL_DIR}/src/others/font_manager/lv_font_manager.c
    ${LVGL_DIR}/src/others/font_manager/lv_font_manager_recycle.c
    ${LVGL_DIR}/src/others/gridnav/lv_gridnav.c
    ${LVGL_DIR}/src/others/ime/lv_ime_pinyin.c
    ${LVGL_DIR}/src/others/imgfont/lv_imgfont.c
    ${LVGL_DIR}/src/others/monkey/lv_monkey.c
    ${LVGL_DIR}/src/others/observer/lv_observer.c
    ${LVGL_DIR}/src/others/snapshot/lv_snapshot.c
    ${LVGL_DIR}/src/others/sysmon/lv_sysmon.c
    ${LVGL_DIR}/src/others/translation/lv_translation.c
    ${LVGL_DIR}/src/others/vg_lite_tvg/vg_lite_matrix.c
    # stdlib
    ${LVGL_DIR}/src/stdlib/lv_mem.c
    ${LVGL_DIR}/src/stdlib/builtin/lv_mem_core_builtin.c
    ${LVGL_DIR}/src/stdlib/builtin/lv_sprintf_builtin.c
    ${LVGL_DIR}/src/stdlib/builtin/lv_string_builtin.c
    ${LVGL_DIR}/src/stdlib/builtin/lv_tlsf.c
    # themes
    ${LVGL_DIR}/src/themes/lv_theme.c
    ${LVGL_DIR}/src/themes/default/lv_theme_default.c
    ${LVGL_DIR}/src/themes/mono/lv_theme_mono.c
    ${LVGL_DIR}/src/themes/simple/lv_theme_simple.c
    # tick
    ${LVGL_DIR}/src/tick/lv_tick.c
    # widgets
    ${LVGL_DIR}/src/widgets/3dtexture/lv_3dtexture.c
    ${LVGL_DIR}/src/widgets/animimage/lv_animimage.c
    ${LVGL_DIR}/src/widgets/arc/lv_arc.c
    ${LVGL_DIR}/src/widgets/arclabel/lv_arclabel.c
    ${LVGL_DIR}/src/widgets/bar/lv_bar.c
    ${LVGL_DIR}/src/widgets/button/lv_button.c
    ${LVGL_DIR}/src/widgets/buttonmatrix/lv_buttonmatrix.c
    ${LVGL_DIR}/src/widgets/calendar/lv_calendar.c
    ${LVGL_DIR}/src/widgets/calendar/lv_calendar_chinese.c
    ${LVGL_DIR}/src/widgets/calendar/lv_calendar_header_arrow.c
    ${LVGL_DIR}/src/widgets/calendar/lv_calendar_header_dropdown.c
    ${LVGL_DIR}/src/widgets/canvas/lv_canvas.c
    ${LVGL_DIR}/src/widgets/chart/lv_chart.c
    ${LVGL_DIR}/src/widgets/checkbox/lv_checkbox.c
    ${LVGL_DIR}/src/widgets/dropdown/lv_dropdown.c
    ${LVGL_DIR}/src/widgets/image/lv_image.c
    ${LVGL_DIR}/src/widgets/imagebutton/lv_imagebutton.c
    ${LVGL_DIR}/src/widgets/keyboard/lv_keyboard.c
    ${LVGL_DIR}/src/widgets/label/lv_label.c
    ${LVGL_DIR}/src/widgets/led/lv_led.c
    ${LVGL_DIR}/src/widgets/line/lv_line.c
    ${LVGL_DIR}/src/widgets/list/lv_list.c
    ${LVGL_DIR}/src/widgets/lottie/lv_lottie.c
    ${LVGL_DIR}/src/widgets/menu/lv_menu.c
    ${LVGL_DIR}/src/widgets/msgbox/lv_msgbox.c
    ${LVGL_DIR}/src/widgets/objx_templ/lv_objx_templ.c
    ${LVGL_DIR}/src/widgets/property/lv_animimage_properties.c
    ${LVGL_DIR}/src/widgets/property/lv_dropdown_properties.c
    ${LVGL_DIR}/src/widgets/property/lv_image_properties.c
    ${LVGL_DIR}/src/widgets/property/lv_keyboard_properties.c
    ${LVGL_DIR}/src/widgets/property/lv_label_properties.c
    ${LVGL_DIR}/src/widgets/property/lv_obj_properties.c
    ${LVGL_DIR}/src/widgets/property/lv_roller_properties.c
    ${LVGL_DIR}/src/widgets/property/lv_slider_properties.c
    ${LVGL_DIR}/src/widgets/property/lv_style_properties.c
    ${LVGL_DIR}/src/widgets/property/lv_textarea_properties.c
    ${LVGL_DIR}/src/widgets/roller/lv_roller.c
    ${LVGL_DIR}/src/widgets/scale/lv_scale.c
    ${LVGL_DIR}/src/widgets/slider/lv_slider.c
    ${LVGL_DIR}/src/widgets/span/lv_span.c
    ${LVGL_DIR}/src/widgets/spinbox/lv_spinbox.c
    ${LVGL_DIR}/src/widgets/spinner/lv_spinner.c
    ${LVGL_DIR}/src/widgets/switch/lv_switch.c
    ${LVGL_DIR}/src/widgets/table/lv_table.c
    ${LVGL_DIR}/src/widgets/tabview/lv_tabview.c
    ${LVGL_DIR}/src/widgets/textarea/lv_textarea.c
    ${LVGL_DIR}/src/widgets/tileview/lv_tileview.c
    ${LVGL_DIR}/src/widgets/win/lv_win.c
    # demos
    ${LVGL_DIR}/demos/lv_demos.c
    ${LVGL_DIR}/demos/benchmark/lv_demo_benchmark.c
    ${LVGL_DIR}/demos/benchmark/assets/img_benchmark_avatar.c
    ${LVGL_DIR}/demos/benchmark/assets/img_benchmark_lvgl_logo_argb.c
    ${LVGL_DIR}/demos/benchmark/assets/img_benchmark_lvgl_logo_rgb.c
    ${LVGL_DIR}/demos/benchmark/assets/lv_font_benchmark_montserrat_12_aligned.c
    ${LVGL_DIR}/demos/benchmark/assets/lv_font_benchmark_montserrat_14_aligned.c
    ${LVGL_DIR}/demos/benchmark/assets/lv_font_benchmark_montserrat_16_aligned.c
    ${LVGL_DIR}/demos/benchmark/assets/lv_font_benchmark_montserrat_18_aligned.c
    ${LVGL_DIR}/demos/benchmark/assets/lv_font_benchmark_montserrat_20_aligned.c
    ${LVGL_DIR}/demos/benchmark/assets/lv_font_benchmark_montserrat_24_aligned.c
    ${LVGL_DIR}/demos/benchmark/assets/lv_font_benchmark_montserrat_26_aligned.c
    ${LVGL_DIR}/demos/widgets/lv_demo_widgets.c
    ${LVGL_DIR}/demos/widgets/lv_demo_widgets_analytics.c
    ${LVGL_DIR}/demos/widgets/lv_demo_widgets_components.c
    ${LVGL_DIR}/demos/widgets/lv_demo_widgets_profile.c
    ${LVGL_DIR}/demos/widgets/lv_demo_widgets_shop.c
    ${LVGL_DIR}/demos/widgets/assets/img_clothes.c
    ${LVGL_DIR}/demos/widgets/assets/img_demo_widgets_avatar.c
    ${LVGL_DIR}/demos/widgets/assets/img_demo_widgets_needle.c
    ${LVGL_DIR}/demos/widgets/assets/img_lvgl_logo.c
)
target_include_directories(lvgl_lib PUBLIC
    ${LVGL_DEMO_DIR}
    ${LVGL_DIR}
    ${LVGL_DIR}/src
    ${LVGL_DIR}/src/core
    ${LVGL_DIR}/src/draw
    ${LVGL_DIR}/src/draw/sw
    ${LVGL_DIR}/src/font
    ${LVGL_DIR}/src/misc
    ${LVGL_DIR}/src/widgets
    ${LVGL_DIR}/demos
    ${LVGL_DIR}/demos/benchmark
    ${LVGL_DIR}/demos/music
    ${LVGL_DIR}/demos/stress
    ${LVGL_DIR}/demos/widgets
    ${LVGL_DIR}/demos/music/assets
    ${LVGL_DIR}/demos/keypad_encoder
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/GCC/ARM_CM33_NTZ/non_secure
)
target_compile_definitions(lvgl_lib PUBLIC
    LV_CONF_INCLUDE_SIMPLE
    CPU_CORTEX_M33
)

# ============================================================
# LVGL_WidgetsDemo executable
# ============================================================
create_project(LVGL_WidgetsDemo
    ${LVGL_DEMO_DIR}/main.c
    ${LVGL_DEMO_DIR}/task_lv.c
    ${LVGL_DEMO_DIR}/common/lv_demo.c
    ${LVGL_DEMO_DIR}/common/lv_port_disp.c
    ${LVGL_DEMO_DIR}/common/lv_port_indev.c
    ${LVGL_DEMO_DIR}/lv_port/lv_glue.c
    ${LVGL_DEMO_DIR}/lv_port/drv_pdma.c
    ${LVGL_DEMO_DIR}/common/nu_misc.c
    ${LVGL_DEMO_DIR}/common/drv_disp/disp_ili9341.c
    ${LVGL_DEMO_DIR}/common/drv_disp/ili9341_spi.c
    ${LVGL_DEMO_DIR}/lv_port/drv_spi.c
    ${LVGL_DEMO_DIR}/common/drv_indev/touch_adc.c
    ${LVGL_DEMO_DIR}/common/drv_indev/touch_adc_calibration.c
)
target_link_libraries(LVGL_WidgetsDemo lvgl_lib freertos_lib)
target_compile_definitions(LVGL_WidgetsDemo PRIVATE
    __320x240__
    CPU_CORTEX_M33
    LV_CONF_INCLUDE_SIMPLE
    NUFUN=1
    M3334KI
)
target_include_directories(LVGL_WidgetsDemo PRIVATE
    ${LVGL_DEMO_DIR}
    ${LVGL_DEMO_DIR}/common
    ${LVGL_DEMO_DIR}/common/drv_disp
    ${LVGL_DEMO_DIR}/common/drv_indev
    ${LVGL_DEMO_DIR}/lv_port
    ${BSP_DIR}/SampleCode
)

# Override optimization to -Os for size reduction
target_compile_options(LVGL_WidgetsDemo PRIVATE -Os)
target_compile_options(lvgl_lib PRIVATE -Os)
target_compile_options(freertos_lib PRIVATE -Os)
bsp_set_thirdparty_warnings(lvgl_lib)
bsp_set_thirdparty_warnings(freertos_lib)
