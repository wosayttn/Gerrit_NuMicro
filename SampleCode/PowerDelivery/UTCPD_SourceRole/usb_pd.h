#define  CODE_LDROM  __attribute__((section("LDROM")))


CODE_LDROM void pd_protection_handler(int port, uint32_t u32Alert);
void pd_backup_src_pdo(int port);
void pd_modify_src_pdo(int port, uint32_t u32CblMaxVbus, uint32_t u32CblMaxCurr);
void pd_recovery_src_pdo(int port);
void tc_vbus_discharge_start(int port);
CODE_LDROM void vbus_set_overcurrent_threshold(int port, uint32_t u32PdoIdx, uint16_t u16PdoCurrent);
CODE_LDROM void pd_protection_run(int port);
CODE_LDROM uint32_t vbus_current_read(int port);
extern uint32_t pd_get_tick(void);
CODE_LDROM uint32_t NPD48_ADC_GET_CONVERSION_DATA(uint8_t ch);
void UFCS_Init(void);
CODE_LDROM void NPD48_ADC_Init(int port);
CODE_LDROM void pd_protection_event_threshold(int port, int target , int cmplvl_mill_unit);
CODE_LDROM void pd_protection_enable_int(int port, int data);
CODE_LDROM void pd_protection_disable_int(int port, int data);
int32_t VinDacFormula(int port, uint16_t mv);
CODE_LDROM int pd_protect_temperature(int port, int type, int value);
void vbus_force_discharge(int port, uint32_t u32IsEnable);
CODE_LDROM void setVinVoltage(int port, uint32_t vbus_volt);

void NPD48_ChipInit(int port);
void NPD48_CertificationFineTune(int port);
CODE_LDROM void vbus_current_clear(int port);

void VBUS_CMD_Disable_Source_VBus(int port);
void VBUS_CMD_Enable_Source_VBus(int port);