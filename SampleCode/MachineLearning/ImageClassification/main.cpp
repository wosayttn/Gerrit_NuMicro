/**************************************************************************//**
 * @file     main.cpp
 * @version  V1.00
 * @brief    MobileNetV2 network sample. Demonstrate image classification.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "BoardInit.hpp"      /* Board initialisation */
#include "log_macros.h"      /* Logging macros (optional) */

#include "BufAttributes.hpp" /* Buffer attributes to be applied */
#include "Classifier.hpp"    /* Classifier for the result */
#include "ClassificationResult.hpp"
#include "InputFiles.hpp"             /* Baked-in input (not needed for live data) */
#include "MobileNetModel.hpp"       /* Model API */
#include "Labels.hpp"
#include "ImgClassProcessing.hpp"

#include "imlib.h"          /* Image processing */
#include "framebuffer.h"

#undef PI /* PI macro conflict with CMSIS/DSP */
#include "NuMicro.h"

//#define __PROFILE__
#define __USE_CCAP__
#define __USE_DISPLAY__
//#define __USE_UVC__

#include "Profiler.hpp"

#if defined (__USE_CCAP__)
    #include "ImageSensor.h"
#endif
#if defined (__USE_DISPLAY__)
    #include "Display.h"
#endif

#if defined (__USE_UVC__)
    #include "UVC.h"
#endif

#define IMAGE_DISP_UPSCALE_FACTOR 1
#if defined(LT7381_LCD_PANEL)
    #define FONT_DISP_UPSCALE_FACTOR 2
#else
    #define FONT_DISP_UPSCALE_FACTOR 1
#endif

using ImageClassifier = arm::app::Classifier;

namespace arm
{
namespace app
{
/* Tensor arena buffer */
static uint8_t tensorArena[ACTIVATION_BUF_SZ] ACTIVATION_BUF_ATTRIBUTE;

/* Optional getter function for the model pointer and its size. */
namespace mobilenet
{
extern uint8_t *GetModelPointer();
extern size_t GetModelLen();
} /* namespace mobilenet */
} /* namespace app */
} /* namespace arm */

/* Image processing initiate function */
//Used by omv library
#if defined(__USE_UVC__)
    //UVC only support QVGA, QQVGA
    #define GLCD_WIDTH  320
    #define GLCD_HEIGHT 240
#else
    #define GLCD_WIDTH 320
    #define GLCD_HEIGHT 240
#endif

//RGB565
#define IMAGE_FB_SIZE   (GLCD_WIDTH * GLCD_HEIGHT * 2)

#undef OMV_FB_SIZE
#define OMV_FB_SIZE (IMAGE_FB_SIZE + 1024)

__attribute__((section(".bss.vram.data"), aligned(32))) static char fb_array[OMV_FB_SIZE + OMV_FB_ALLOC_SIZE];
__attribute__((section(".bss.vram.data"), aligned(32))) static char jpeg_array[OMV_JPEG_BUF_SIZE];

char *_fb_base = NULL;
char *_fb_end = NULL;
char *_jpeg_buf = NULL;
char *_fballoc = NULL;

static void omv_init()
{
    image_t frameBuffer;

    frameBuffer.w = GLCD_WIDTH;
    frameBuffer.h = GLCD_HEIGHT;
    frameBuffer.size = GLCD_WIDTH * GLCD_HEIGHT * 2;
    frameBuffer.pixfmt = PIXFORMAT_RGB565;

    _fb_base = fb_array;
    _fb_end =  fb_array + OMV_FB_SIZE - 1;
    _fballoc = _fb_base + OMV_FB_SIZE + OMV_FB_ALLOC_SIZE;
    _jpeg_buf = jpeg_array;

    fb_alloc_init0();

    framebuffer_init0();
    framebuffer_init_from_image(&frameBuffer);
}

int main()
{

    /* Initialise the UART module to allow printf related functions (if using retarget) */
    BoardInit();

    /* Model object creation and initialisation. */
    arm::app::MobileNetModel model;

    if (!model.Init(arm::app::tensorArena,
                    sizeof(arm::app::tensorArena),
                    arm::app::mobilenet::GetModelPointer(),
                    arm::app::mobilenet::GetModelLen()))
    {
        printf_err("Failed to initialise model\n");
        return 1;
    }

    /* Setup cache poicy of tensor arean buffer */
    info("Set tesnor arena cache policy to WTRA \n");
    const std::vector<ARM_MPU_Region_t> mpuConfig =
    {
        {
            // SRAM for tensor arena
            ARM_MPU_RBAR(((unsigned int)arm::app::tensorArena),        // Base
                         ARM_MPU_SH_NON,    // Non-shareable
                         0,                 // Read-only
                         1,                 // Non-Privileged
                         1),                // eXecute Never enabled
            ARM_MPU_RLAR((((unsigned int)arm::app::tensorArena) + ACTIVATION_BUF_SZ - 1),        // Limit
                         eMPU_ATTR_CACHEABLE_WTRA) // Attribute index - Write-Through, Read-allocate
        },
#if defined (__USE_CCAP__)
        {
            // Image data from CCAP DMA, so must set frame buffer to Non-cache attribute
            ARM_MPU_RBAR(((unsigned int)fb_array),        // Base
                         ARM_MPU_SH_NON,    // Non-shareable
                         0,                 // Read-only
                         1,                 // Non-Privileged
                         1),                // eXecute Never enabled
            ARM_MPU_RLAR((((unsigned int)fb_array) + OMV_FB_SIZE - 1),        // Limit
                         eMPU_ATTR_NON_CACHEABLE) // NonCache
        },
#endif
    };

    // Setup MPU configuration
    InitPreDefMPURegion(&mpuConfig[0], mpuConfig.size());

    ImageClassifier classifier;  /* Classifier object. */

    uint8_t u8ImgIdx = 0;

    TfLiteTensor *inputTensor   = model.GetInputTensor(0);

    if (!inputTensor->dims)
    {
        printf_err("Invalid input tensor dims\n");
        return 2;
    }
    else if (inputTensor->dims->size < 3)
    {
        printf_err("Input tensor dimension should be >= 3\n");
        return 3;
    }

    TfLiteIntArray *inputShape = model.GetInputShape(0);

    const int inputImgCols = inputShape->data[arm::app::MobileNetModel::ms_inputColsIdx];
    const int inputImgRows = inputShape->data[arm::app::MobileNetModel::ms_inputRowsIdx];
    const uint32_t inputChannels = inputShape->data[arm::app::MobileNetModel::ms_inputChannelsIdx];

    //label information
    std::vector <std::string> labels;
    GetLabelsVector(labels);

    /* Set up pre and post-processing. */
    arm::app::ImgClassPreProcess preProcess = arm::app::ImgClassPreProcess(&model);

    std::vector<arm::app::ClassificationResult> results;
    std::string predictLabelInfo;
    arm::app::ImgClassPostProcess postProcess = arm::app::ImgClassPostProcess(classifier, &model,
                                                                              labels, results);

    //display framebuffer
    image_t frameBuffer;
    rectangle_t roi;

    //omv library init
    omv_init();
    framebuffer_init_image(&frameBuffer);

#if defined(__PROFILE__)
    arm::app::Profiler profiler;
    uint64_t u64StartCycle;
    uint64_t u64EndCycle;
    uint64_t u64CCAPStartCycle;
    uint64_t u64CCAPEndCycle;
#else
    pmu_reset_counters();
#endif

#define EACH_PERF_SEC 5
    uint64_t u64PerfCycle;
    uint64_t u64PerfFrames = 0;

    u64PerfCycle = pmu_get_systick_Count();
    u64PerfCycle += (SystemCoreClock * EACH_PERF_SEC);

#if defined (__USE_CCAP__)
    //Setup image senosr
    ImageSensor_Init();
    ImageSensor_Config(eIMAGE_FMT_RGB565, frameBuffer.w, frameBuffer.h, true);
#endif

#if defined (__USE_DISPLAY__)
    char szDisplayText[160];

    Display_Init();
    Display_ClearLCD(C_WHITE);
#endif

#if defined (__USE_UVC__)
    UVC_Init();
    HSUSBD_Start();
#endif

#if !defined (__USE_CCAP__)
    char chStdIn;

    info("Press 'n' to run next image inference \n");
    info("Press 'q' to exit program \n");

    while ((chStdIn = getchar()))
    {
        if (chStdIn == 'q')
            break;
        else if (chStdIn != 'n')
            continue;

#else

    while (1)
    {
#endif

#if !defined (__USE_CCAP__)

        const uint8_t *pu8ImgSrc = get_img_array(u8ImgIdx);

        if (nullptr == pu8ImgSrc)
        {
            printf_err("Failed to get image index %" PRIu32 " (max: %u)\n", u8ImgIdx,
                       NUMBER_OF_FILES - 1);
            return 4;
        }

        //resize source image to framebuffer
        image_t srcImg;

        srcImg.w = IMAGE_WIDTH;
        srcImg.h = IMAGE_HEIGHT;
        srcImg.data = (uint8_t *)pu8ImgSrc;
        srcImg.pixfmt = PIXFORMAT_RGB888;

        roi.x = 0;
        roi.y = 0;
        roi.w = IMAGE_WIDTH;
        roi.h = IMAGE_HEIGHT;
        imlib_nvt_scale(&srcImg, &frameBuffer, &roi);
#endif

#if defined (__USE_DISPLAY__)
        //Display image on LCD
        S_DISP_RECT sDispRect;

        sDispRect.u32TopLeftX = 0;
        sDispRect.u32TopLeftY = 0;
        sDispRect.u32BottonRightX = ((frameBuffer.w * IMAGE_DISP_UPSCALE_FACTOR) - 1);
        sDispRect.u32BottonRightY = ((frameBuffer.h * IMAGE_DISP_UPSCALE_FACTOR) - 1);

#if defined(__PROFILE__)
        u64StartCycle = pmu_get_systick_Count();
#endif

        Display_FillRect((uint16_t *)frameBuffer.data, &sDispRect, IMAGE_DISP_UPSCALE_FACTOR);

#if defined(__PROFILE__)
        u64EndCycle = pmu_get_systick_Count();
        info("display image cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif

#endif

        //resize framebuffer image to model input
        image_t resizeImg;

        roi.x = 0;
        roi.y = 0;
        roi.w = frameBuffer.w;
        roi.h = frameBuffer.h;

        resizeImg.w = inputImgCols;
        resizeImg.h = inputImgRows;
        resizeImg.data = (uint8_t *)inputTensor->data.data; //direct resize to input tensor buffer
        resizeImg.pixfmt = PIXFORMAT_RGB888;

#if defined(__PROFILE__)
        u64StartCycle = pmu_get_systick_Count();
#endif
        imlib_nvt_scale(&frameBuffer, &resizeImg, &roi);
#if defined(__PROFILE__)
        u64EndCycle = pmu_get_systick_Count();
        info("resize cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif

#if defined (__USE_UVC__)

        if (UVC_IsConnect())
        {
#if (UVC_Color_Format == UVC_Format_YUY2)
            image_t RGB565Img;
            image_t YUV422Img;

            RGB565Img.w = frameBuffer.w;
            RGB565Img.h = frameBuffer.h;
            RGB565Img.data = (uint8_t *)frameBuffer.data;
            RGB565Img.pixfmt = PIXFORMAT_RGB565;

            YUV422Img.w = RGB565Img.w;
            YUV422Img.h = RGB565Img.h;
            YUV422Img.data = (uint8_t *)frameBuffer.data;
            YUV422Img.pixfmt = PIXFORMAT_YUV422;

            roi.x = 0;
            roi.y = 0;
            roi.w = RGB565Img.w;
            roi.h = RGB565Img.h;
            imlib_nvt_scale(&RGB565Img, &YUV422Img, &roi);

#else
            image_t origImg;
            image_t vflipImg;

            origImg.w = frameBuffer.w;
            origImg.h = frameBuffer.h;
            origImg.data = (uint8_t *)frameBuffer.data;
            origImg.pixfmt = PIXFORMAT_RGB565;

            vflipImg.w = origImg.w;
            vflipImg.h = origImg.h;
            vflipImg.data = (uint8_t *)frameBuffer.data;
            vflipImg.pixfmt = PIXFORMAT_RGB565;

            imlib_nvt_vflip(&origImg, &vflipImg);
#endif
            UVC_SendImage((uint32_t)frameBuffer.data, IMAGE_FB_SIZE, uvcStatus.StillImage);
        }

#endif


#if defined (__USE_CCAP__)
        //Capture new image
#if defined(__PROFILE__)
        u64CCAPStartCycle = pmu_get_systick_Count();
#endif
        ImageSensor_TriggerCapture((uint32_t)frameBuffer.data);
#endif

#if defined(__PROFILE__)
        u64StartCycle = pmu_get_systick_Count();
#endif

        /* Run the pre-processing, inference and post-processing. */
        if (!preProcess.DoPreProcess(resizeImg.data, (resizeImg.w * resizeImg.h * inputChannels)))
        {
            goto round_done;
        }

#if defined(__PROFILE__)
        u64EndCycle = pmu_get_systick_Count();
        info("quantize cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif

#if !defined (__USE_CCAP__)
        /* Run inference over this image. */
        info("Running inference on image %" PRIu32 " => %s\n", u8ImgIdx, get_filename(u8ImgIdx));
#endif

#if defined(__PROFILE__)
        profiler.StartProfiling("Inference");
#endif

        if (!model.RunInference())
        {
            printf_err("Inference failed.");
            return 5;
        }

#if defined(__PROFILE__)
        profiler.StopProfiling();
#endif

#if defined (__USE_CCAP__)
        //Capture new image

        ImageSensor_WaitCaptureDone();
#if defined(__PROFILE__)
        u64CCAPEndCycle = pmu_get_systick_Count();
        info("ccap capture cycles %llu \n", (u64CCAPEndCycle - u64CCAPStartCycle));
#endif
#endif

        //results.clear(); not necessary ???
        predictLabelInfo.clear();

#if defined(__PROFILE__)
        u64StartCycle = pmu_get_systick_Count();
#endif

        if (postProcess.DoPostProcess())
        {
            predictLabelInfo =  results[0].m_label + std::string(":") + std::to_string(results[0].m_normalisedVal);
        }
        else
        {
            predictLabelInfo = std::string("???") + std::string(":") + std::to_string(results[0].m_normalisedVal);
        }

#if defined(__PROFILE__)
        u64EndCycle = pmu_get_systick_Count();
        info("post processing cycles %llu \n", (u64EndCycle - u64StartCycle));
#endif

        //show result
        info("Final results:\n");
        info("%s\n", predictLabelInfo.c_str());

#if defined (__USE_DISPLAY__)
        sprintf(szDisplayText, "%s", predictLabelInfo.c_str());

        sDispRect.u32TopLeftX = 0;
        sDispRect.u32TopLeftY = frameBuffer.h * IMAGE_DISP_UPSCALE_FACTOR;
        sDispRect.u32BottonRightX = (Disaplay_GetLCDWidth() - 1);
        sDispRect.u32BottonRightY = ((frameBuffer.h * IMAGE_DISP_UPSCALE_FACTOR) + (FONT_DISP_UPSCALE_FACTOR * FONT_HTIGHT) - 1);

        Display_ClearRect(C_WHITE, &sDispRect);
        Display_PutText(
            szDisplayText,
            strlen(szDisplayText),
            0,
            frameBuffer.h * IMAGE_DISP_UPSCALE_FACTOR,
            C_BLUE,
            C_WHITE,
            false,
            FONT_DISP_UPSCALE_FACTOR
        );
#endif

#if defined(__PROFILE__)
        profiler.PrintProfilingResult();
#endif

        u64PerfFrames ++;

        if (pmu_get_systick_Count() > u64PerfCycle)
        {
            info("Total inference rate: %llu\n", u64PerfFrames / EACH_PERF_SEC);

#if defined (__USE_DISPLAY__)
            sprintf(szDisplayText, "Frame Rate %llu", u64PerfFrames / EACH_PERF_SEC);

            sDispRect.u32TopLeftX = 0;
            sDispRect.u32TopLeftY = frameBuffer.h + (FONT_HTIGHT * FONT_DISP_UPSCALE_FACTOR);
            sDispRect.u32BottonRightX = (frameBuffer.w);
            sDispRect.u32BottonRightY = (frameBuffer.h + (2 * FONT_HTIGHT * FONT_DISP_UPSCALE_FACTOR) - 1);

            Display_ClearRect(C_WHITE, &sDispRect);
            Display_PutText(
                szDisplayText,
                strlen(szDisplayText),
                0,
                (frameBuffer.h * IMAGE_DISP_UPSCALE_FACTOR) + FONT_HTIGHT * FONT_DISP_UPSCALE_FACTOR,
                C_BLUE,
                C_WHITE,
                false,
                FONT_DISP_UPSCALE_FACTOR
            );
#endif

            u64PerfCycle = pmu_get_systick_Count();
            u64PerfCycle += (SystemCoreClock * EACH_PERF_SEC);
            u64PerfFrames = 0;
        }

round_done:
        u8ImgIdx ++;

        if (u8ImgIdx >= NUMBER_OF_FILES)
            u8ImgIdx = 0;

    }
}
