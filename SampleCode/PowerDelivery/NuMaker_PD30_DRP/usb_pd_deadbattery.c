#include "NuMicro.h"
#include "utcpdlib.h"
#include "stdlib.h"

#define CONFIG_MAX_CHANGER_NUM 	(1)
int32_t gi32deadbatThreshold[CONFIG_MAX_CHANGER_NUM];
void pd_set_deadbattry_threshold(int chgnum, int32_t i32threshold)
{
    gi32deadbatThreshold[chgnum] = i32threshold;
}

int32_t pd_get_deadbattry_threshold(int chgnum)
{
    return gi32deadbatThreshold[chgnum];
}