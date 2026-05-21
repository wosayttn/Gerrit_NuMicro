/**************************************************************************//**
 * @file     DecodeGIF.c
 * @brief    To utilize emWin library to demonstrate GIF decode feature.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "GUI.h"

#define ANIMATED_GIF  //only work for chip series with sufficient RAM size

#ifdef ANIMATED_GIF
#include "nuvoton.c"
#define WIDTH   320
#define HEIGHT  240
#else
#include "testGIF.c"
#define WIDTH   221
#define HEIGHT  221
#endif

void MainTask(void);
extern const unsigned char my_gif_data[];
unsigned int my_gif_size;

/* For Display on ILI9341 */
#ifdef __DEMO_320x240__
#include <stdio.h>

void _Write0(U8 Cmd);
void _Write1(U8 Data);
void _InitController(void);
void NVT_SetPhysRAMAddr(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1);

/*
 * NVT_FillRect - 將 MEMDEV 的 16bpp RGB565 像素送至 ILI9341
 * pu16Data: 指向 MEMDEV pixel buffer 起始位址(NULL 則填藍色背景)
 */
void NVT_FillRect(int w, int h, U16 *pu16Data)
{
    int x, y;
    U16 pixelData;

    for (y = (240 - h) / 2; y < (240 + h) / 2; y++)
    {
        NVT_SetPhysRAMAddr((320 - w) / 2, y, (320 + w) / 2 - 1, y);
        for (x = (320 - w) / 2; x < (320 + w) / 2; x++)
        {
            if (pu16Data != NULL)
                pixelData = *pu16Data++;
            else
                pixelData = 0x001F;  /* 背景色：藍 */
            _Write1((U8)(pixelData >> 8));
            _Write1((U8)(pixelData & 0xFF));
        }
    }
}
#endif  /* __DEMO_320x240__ */

extern volatile GUI_TIMER_TIME OS_TimeMS;

void DecodeGIFToBuffer(void)
{
    GUI_MEMDEV_Handle  hGIF;
    GUI_GIF_INFO       GIF_Info;
    GUI_GIF_IMAGE_INFO frameInfo;
    U16               *pSrcData;
    GUI_TIMER_TIME     lastTime, curTime;
    int                r, i;
    static int         s_firstRun = 1;

    my_gif_size = sizeof(my_gif_data);

    r = GUI_GIF_GetInfo(my_gif_data, my_gif_size, &GIF_Info);
    if (r != 0)
    {
        printf("[GIF] GetInfo failed (r=%d)\n", r);
        return;
    }

    if (s_firstRun)
    {
        printf("[GIF] file=%u bytes  screen=%dx%d  frames=%d\n",
               my_gif_size, GIF_Info.xSize, GIF_Info.ySize, GIF_Info.NumImages);
        printf("[GIF] GUI_NUMBYTES=%u  MEMDEV_need=%u  free_before_alloc=%u\n",
               (unsigned)(185*1024),
               (unsigned)(GIF_Info.xSize * GIF_Info.ySize * 2),
               (unsigned)GUI_ALLOC_GetNumFreeBytes());
    }

    hGIF = GUI_MEMDEV_CreateFixed(0, 0, GIF_Info.xSize, GIF_Info.ySize,
                                  GUI_MEMDEV_NOTRANS,
                                  GUI_MEMDEV_APILIST_16, GUICC_M565);
    if (hGIF == 0)
    {
        printf("[GIF] MEMDEV alloc failed! free=%u need=%u\n",
               (unsigned)GUI_ALLOC_GetNumFreeBytes(),
               (unsigned)(GIF_Info.xSize * GIF_Info.ySize * 2));
        return;
    }

    GUI_MEMDEV_Select(hGIF);
    pSrcData = (U16 *)GUI_MEMDEV_GetDataPtr(hGIF);

    if (s_firstRun)
        printf("[GIF] MEMDEV OK  free_after_alloc=%u\n",
               (unsigned)GUI_ALLOC_GetNumFreeBytes());

    GUI_SetBkColor(GUI_BLACK);
    GUI_Clear();

#ifndef ANIMATED_GIF
    GUI_GIF_Draw(my_gif_data, my_gif_size, 0, 0);
#ifdef __DEMO_320x240__
    NVT_FillRect(WIDTH, HEIGHT, pSrcData);
#endif
#else
    for (i = 0; i < GIF_Info.NumImages; i++)
    {
        int gifRet;
        lastTime = OS_TimeMS;

        gifRet = GUI_GIF_DrawSub(my_gif_data, my_gif_size, 0, 0, i);

        if (gifRet != 0)
        {
            printf("[GIF] DrawSub frame=%d ret=%d  free=%u\n",
                   i, gifRet, (unsigned)GUI_ALLOC_GetNumFreeBytes());
        }

#ifdef __DEMO_320x240__
        GUI_GIF_GetImageInfo(my_gif_data, my_gif_size, &frameInfo, i);
        curTime = OS_TimeMS;
        NVT_FillRect(WIDTH, HEIGHT, pSrcData);
        GUI_X_Delay(frameInfo.Delay * 10 + lastTime - curTime);
#endif
    }
#endif

    GUI_MEMDEV_Delete(hGIF);
    s_firstRun = 0;
}

void MainTask(void)
{
    GUI_Init();

#ifdef __DEMO_320x240__
    _InitController();
    NVT_FillRect(320, 240, NULL);  /* 初始清除畫面(藍色背景) */
#endif

    while (1)
    {
        DecodeGIFToBuffer();
        GUI_Delay(100);
    }
}

/*************************** End of file ****************************/
