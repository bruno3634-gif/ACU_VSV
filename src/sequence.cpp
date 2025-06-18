#include "sequence.h"

#include "Watchdog_t4.h"
#define pressureA 0



// Declare external watchdog object
extern WDT_T4<WDT1> wdt_software;

extern void median_pressures(); 


int initial_sequence()
{
    volatile int ebs_error = 0;
    extern float EBS_TANK_PRESSURE_B_value;
    if (EBS_TANK_PRESSURE_B_value < TANK_PRESSURE_THRESHOLD || EBS_TANK_PRESSURE_B_value > 10)
    {
        // EBS ERROR
        ebs_error = 1;
    }
    else
    {
        #if pressureA
            extern float EBS_TANK_PRESSURE_A_value;
            if (EBS_TANK_PRESSURE_A_value < TANK_PRESSURE_THRESHOLD || EBS_TANK_PRESSURE_B_value > 10)
            {
                // EBS ERROR
                ebs_error = 1;
            }
        #endif
    }
    if (!ebs_error)
    {
        CAN_message_t CAN_MSG;
        do
        {
            wdt_software.feed();
            CAN_MSG = CAN_MSG_RECEIVE();
        } while (CAN_MSG.id != AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID);
        if (CAN_MSG.buf[1] < 11.5 * EBS_TANK_PRESSURE_B)
        {
            // EBS ERROR
            ebs_error = 1;
        }
        else
        {
            #if pressureA
                if (CAN_MSG.buf[2] < 11.5 * EBS_TANK_PRESSURE_A)
                {
                    // EBS ERROR
                    ebs_error = 1;
                }
            #endif
        }
    }
    return ebs_error;
}


int initial_sequence_after_ign(int ebs_error){
    CAN_message_t CAN_MSG;
    digitalWrite(SOLENOID1, HIGH);
    digitalWrite(SOLENOID2, LOW);
    do{
        wdt_software.feed();
        CAN_MSG = CAN_MSG_RECEIVE();
    }while(CAN_MSG.id != AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID);

}