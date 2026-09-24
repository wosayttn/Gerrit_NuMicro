#include "NuMicro.h"
#include "utcpdlib.h"
#include "rt9490.h"

void Charger_init(int chgnum);
void Charger_get_current(int chgnum, int* curr);
void Charger_get_voltage(int chgnum, int* volt);
void Charger_enable_charge_mode(int chgnum);
void Charger_disable_charge_mode(int chgnum);
void Charger_enable_otg_mode(int chgnum);
void Charger_disable_otg_mode(int chgnum);
//void Charger_dump_register(int chgnum, int begin, int end);
void Charger_dump_register(int chgnum);
void Charger_set_otg_current_voltage(int chgnum, int output_current, int output_voltage);

void Charger_vbus_highz_mode(int chgnum, bool en);