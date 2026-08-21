/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2023  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V6.46 - Graphical user interface for embedded applications **
emWin is protected by international copyright laws. Knowledge of the
source code may not be used to write a similar product.
This file may
only be used in accordance with the following terms:

The  software has  been licensed by SEGGER Software GmbH to Nuvoton Technology Corporation
at the address: No. 4, Creation Rd. III, Hsinchu Science Park, Taiwan
for the purposes  of  creating  libraries  for its
Arm Cortex-A, Arm Cortex-M and  Arm9 32-bit microcontrollers, commercialized and distributed by Nuvoton Technology Corporation
under  the terms and conditions  of  an  End  User
License  Agreement  supplied  with  the libraries.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Software GmbH
Licensed to:              Nuvoton Technology Corporation, No. 4, Creation Rd. III, Hsinchu Science Park, 30077 Hsinchu City, Taiwan
Licensed SEGGER software: emWin
License number:           GUI-00735
License model:            emWin License Agreement, signed February 27, 2018
Licensed platform:        Cortex-M, Cortex-A, and ARM9 32-bit series microcontroller designed and manufactured by Nuvoton Technology Corporation.
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2018-03-26 - 2025-09-04
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

#include <stddef.h>
#include <stdio.h>

#include "GUI.h"
#include "GUIDRV_Null.h"


/*********************************************************************
*
*       Layer configuration
*
**********************************************************************
*/
//
// Physical display size
//
#ifdef __DEMO_160x128__
    #define XSIZE_PHYS 128
    #define YSIZE_PHYS 160
#elif defined(__DEMO_320x240__)
    #define XSIZE_PHYS 320   /* Landscape 320x240: ILI9341 orientation set via MADCTL hardware */
    #define YSIZE_PHYS 240
#else
    #define XSIZE_PHYS 240
    #define YSIZE_PHYS 320
#endif

//
// Color conversion
//
#ifdef __DEMO_160x128__
    #define COLOR_CONVERSION GUICC_M565
#else
    #define COLOR_CONVERSION GUICC_565
#endif

//
// Display driver
//
#define DISPLAY_DRIVER GUIDRV_NULL

//
// Orientation
//
//#define DISPLAY_ORIENTATION (0)
//#define DISPLAY_ORIENTATION (GUI_MIRROR_X)
//#define DISPLAY_ORIENTATION (GUI_MIRROR_Y)
//#define DISPLAY_ORIENTATION (GUI_MIRROR_X | GUI_MIRROR_Y)
//#define DISPLAY_ORIENTATION (GUI_SWAP_XY)
#ifdef __DEMO_160x128__
    #define DISPLAY_ORIENTATION (GUI_MIRROR_X | GUI_SWAP_XY)
#elif defined(__DEMO_320x240__)
    #define DISPLAY_ORIENTATION (0)  /* ILI9341 handles landscape via MADCTL=0xE8; no emWin transform needed */
#else
    #define DISPLAY_ORIENTATION (GUI_MIRROR_Y | GUI_SWAP_XY)
#endif
//#define DISPLAY_ORIENTATION (GUI_MIRROR_X | GUI_MIRROR_Y | GUI_SWAP_XY)

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
    #define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
    #define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
    #error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
    #error Physical Y size of display is not defined!
#endif
#ifndef   COLOR_CONVERSION
    #error Color conversion not defined!
#endif
#ifndef   DISPLAY_DRIVER
    #error No display driver defined!
#endif
#ifndef   DISPLAY_ORIENTATION
    #define DISPLAY_ORIENTATION 0
#endif

/*********************************************************************
*
*       Static code
*
**********************************************************************
*/
/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/


void LCD_X_Config(void)
{



    //
    // Set display driver and color conversion for 1st layer
    //
    GUI_DEVICE_CreateAndLink(GUIDRV_NULL, COLOR_CONVERSION, 0, 0);
    //
    // Orientation
    //

    //
    // Display driver configuration
    //
    if(DISPLAY_ORIENTATION & GUI_SWAP_XY)
    {
        LCD_SetSizeEx(0, YSIZE_PHYS,   XSIZE_PHYS);
        LCD_SetVSizeEx(0, VYSIZE_PHYS,  VXSIZE_PHYS);
    }
    else
    {
        LCD_SetSizeEx(0, XSIZE_PHYS,   YSIZE_PHYS);
        LCD_SetVSizeEx(0, VXSIZE_PHYS,  VYSIZE_PHYS);
    }
    //
    // Function selection, hardware routines (PortAPI) and operation mode (bus, bpp and cache)
    //

}

/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Purpose:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void *pData)
{
    int r;

    GUI_USE_PARA(LayerIndex);
    GUI_USE_PARA(pData);
    switch(Cmd)
    {
        //
        // Required
        //
        case LCD_X_INITCONTROLLER:
        {
            //
            // Called during the initialization process in order to set up the
            // display controller and put it into operation. If the display
            // controller is not initialized by any external routine this needs
            // to be adapted by the customer...
            //

            return 0;
        }
        default:
            r = -1;
    }
    return r;
}

/*************************** End of file ****************************/
