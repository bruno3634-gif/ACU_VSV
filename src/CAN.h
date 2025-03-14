#include <Arduino.h>
#include "definitions.h"
#include "FlexCAN_T4_.h" 




//extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;


void CAN_init();
void CAN_MSG_SEND(int id, uint8_t lenght, uint8_t data[]);
CAN_message_t CAN_MSG_RECEIVE();