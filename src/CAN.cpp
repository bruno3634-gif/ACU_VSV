#include "CAN.h"


 FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> can2;



 void CAN_init() {
  can2.begin();
  can2.setBaudRate(1000000);

  can2.setMBFilter(MB0, RES_ID);              // Only accept RES_ID on mailbox 0
  can2.setMBFilter(MB1, IGN_FROM_VCU);        // Only accept IGN_FROM_VCU on mailbox 1
  can2.setMBFilter(MB2, JETSON_AMS);          // Only accept JETSON_AMS on mailbox 2
  can2.setMBFilter(MB3, JETSON_MS);           // Only accept JETSON_MS on mailbox 3

  // Set remaining mailboxes to reject all
  for (uint8_t i = 4; i < 16; i++) {
    //can2.setMBFilter(i, 0x000);               // Set other mailboxes to reject all
  }

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