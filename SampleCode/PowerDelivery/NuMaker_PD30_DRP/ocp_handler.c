#include "NuMicro.h"
#include "utcpdlib.h"
#include "stdlib.h"

#if 1
#define DBG_PRINTF(...)
#else
#define DBG_PRINTF printf
#endif

int16_t ina219_ReadShuntVoltage_uV(void);

int16_t gi16VbusOcpThreshold;
static const int16_t i16VbusOcpThresholdArray[4] =
{
    1000,         /* Default 5V PDO */
    3100, 	      /* Fixed 5V PDO OCP Threshold */
    1600, 	      /* Fixed 9V PDO OCP Threshold */
    1100,         /* Fixed 15V PDO OCP Threshold */
};

#define E_MOVING_AVE_SAMPLE  4


int16_t gi16Array[E_MOVING_AVE_SAMPLE] = {0};
void vbus_set_overcurrent_threshold(int port, uint32_t u32PdoIdx)
{
    int i;
    gi16VbusOcpThreshold = i16VbusOcpThresholdArray[u32PdoIdx];
    DBG_PRINTF("Set OCP Threshold = %d\n", gi16VbusOcpThreshold);
    for(i = 0; i < E_MOVING_AVE_SAMPLE; i = i + 1)
        gi16Array[i] = 0;
}
int16_t vbus_get_overcurrent_threshold(int port)
{
    return gi16VbusOcpThreshold;
}

void vbus_overcurrent_moving_average(int16_t i16Curr)
{
    static uint8_t u8Idx = 0;
    gi16Array[u8Idx] = i16Curr;
    u8Idx = u8Idx + 1;
    if(u8Idx >= E_MOVING_AVE_SAMPLE)
        u8Idx = 0;
}


int16_t vbus_get_current(int port)
{
    int i, ave = 0;
    for(i = 0; i < E_MOVING_AVE_SAMPLE; i = i + 1)
        ave =  ave + gi16Array[i];
    ave = ave / E_MOVING_AVE_SAMPLE;

    return  (int16_t)abs(ave);
}


void vbus_ocp_polling(int port)
{
    int i32Curr;
    int16_t i16uV;
    int i32mV;
    int16_t i16Curr, i16Threshold;


    i16uV = ina219_ReadShuntVoltage_uV();
    DBG_PRINTF("Shunt Volt = %d uV\n", i16uV);
    i16Curr = i16uV / 10; 	//uV/10 mOhm = mA
    DBG_PRINTF("VBUS Curr = %d mA\n", i16Curr);

    i16Threshold = vbus_get_overcurrent_threshold(port);
    if ( i16Curr > i16Threshold)
    {
        printf("i16Curr = %d, i16Threshold = %d\n", i16Curr, i16Threshold);
        pd_dpm_request(port, DPM_REQUEST_HARD_RESET_SEND);
        printf("OCP Send Hard Reset\n");
    }
}