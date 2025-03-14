#include "CAN.h"


 FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;



 void CAN_init() {
  can2.begin();
  can2.setBaudRate(1000000);
}

void CAN_MSG_SEND(int id, uint8_t lenght, uint8_t data[]){
CAN_message_t msg;
  msg.id = id;
  msg.len = lenght;
  for (int i = 0; i < lenght; i++)
  {
    msg.buf[i] = data[i];
  }
  can2.write(msg);
}


CAN_message_t CAN_MSG_RECEIVE(){
  CAN_message_t msg;
  can2.read(msg);
  return msg;
  
}