#ifndef ENS_STRUCT
#define ENS_STRUCT

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct data_to_obc {

    float vbat;
    uint8_t tanWaState;
    float thrust_val;
    float tankWeight_val;
    float temperature_postFill;
    float temperature_Wall;
    float postFillN2O_pres;
    float cutoffN2O_pres;
    float droidN2O_press;
    float preRegulatorN2_pres;
    float postRegulatorN2_pres;
    float postFillN2_pres;
    float droidN2_press;
    float combChamber_pres;
    //TANWA POWER DATA
    float TANWA_24V_SYS_VOLTAGE;
    float TANWA_24V_SYS_CURRENT;
    float TANWA_24V_SOL_VOLTAGE;
    float TANWA_24V_SOL_CURRENT;
    //BOOLEANS
    bool soft_arm : 1;
    bool canWeighta_con : 1;
    bool canSensor_con : 1;
    bool canSolenoid_con : 1;
    bool canUtility_con : 1;
    bool canPower_con : 1;
    bool igniterContinouity_1 : 1;
    bool igniterContinouity_2 : 1;
    bool fillN2OState : 1;
    bool deprN2OState : 1;
    bool fillN2State : 1;
    bool deprN2State : 1;
    bool droidN2OState : 1;
    bool droidN2State : 1;
    bool heatingTankState : 1;
    bool heatingValveState : 1;
    bool abortButton : 1;

} data_to_obc_t;


#endif