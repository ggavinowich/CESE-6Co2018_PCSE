
/*
 * mainCan.c
 *
 *  Created on: Aug 16, 2018
 *      Author: gabriel
 */


#include "sapi.h"               // <= sAPI header
#include "sapi_can.h"
#include "can_cfg_board_2.h"	/* This already includes sapi_can.h */

DEBUG_PRINT_ENABLE
CONSOLE_PRINT_ENABLE

#define BOARD_ID	2

/* Create de initialization array with the ids we are going to listen */
CAN_CreateDataObjects(can1_buffer, MAX_NUMBER_OF_CAN1_SIGNALS);


/* Local variables */
uint8_t bufferRx[8];
uint32_t dlcRx;
uint8_t bufferTx[8];
uint32_t dlcTx;

int main(void)
{
   uint32_t i;


//   Board_Init();
   boardConfig();

   printf("Board number %d\r\n", BOARD_ID);

   sapi_can_interface_t interface;

   sapi_can_canInit(&interface, CCAN_1, 125000, can1_buffer, MAX_NUMBER_OF_CAN1_SIGNALS, can1_initialArray);

   while(1) {

	   /* Poll for the different ids and execute the command */
	   if (true == sapi_can_read(bufferRx, &dlcRx, &interface, SIG1) ){
		   /* Led delay message received */
		   uint32_t delayMs;
		   if (4 == dlcRx ) {
		       delayMs = bufferRx[0];
		       delayMs = (delayMs | (bufferRx[1]<<8));
			   delayMs = (delayMs | (bufferRx[2]<<16));
			   delayMs = (delayMs | (bufferRx[3]<<24));
			   printf("Received command 0x%X, with delay %d ms\r\n", SIG1, delayMs);
			   /* Send the ACK */
 		       bufferTx[0] = BOARD_ID;
			   bufferTx[1] = 0x01;
			   dlcTx = 2;
			   delay(300);
			   sapi_can_write(&interface, 0x304, bufferTx, dlcTx);
			   gpioWrite(LEDR, 1);
			   delay(delayMs);
			   gpioWrite(LEDR, 0);
		   } else {
			   /* dlc different that the one expected, send NACK */
			   printf("Error receiving command 0x%X\r\n", SIG1);
 			   bufferTx[0] = BOARD_ID;
   			   bufferTx[1] = 0x00;
			   dlcTx = 2;
			   delay(300);
			   sapi_can_write(&interface, 0x304, bufferTx, dlcTx);
		   }
	   }
	   if (true == sapi_can_read(bufferRx, &dlcRx, &interface, SIG2) ){
		   /* Fail on purpose!!! */
		   if (1 == dlcRx ) {
			   printf("Received command 0x%X\r\n", SIG2);
 			   bufferTx[0] = BOARD_ID;
			   bufferTx[1] = 0x01;
			   dlcTx = 2;
			   delay(300);
			   sapi_can_write(&interface, 0x305, bufferTx, dlcTx);
		   } else {
			   /* dlc different that the one expected, send NACK */
			   printf("Error receiving command 0x%X\r\n", SIG2);
  			   bufferTx[0] = BOARD_ID;
 			   bufferTx[1] = 0x00;
			   dlcTx = 2;
			   delay(300);
			   sapi_can_write(&interface, 0x305, bufferTx, dlcTx);
		   }
	   }
	   if (true == sapi_can_read(bufferRx, &dlcRx, &interface, SIG3) ) {
		   /* Systick command received */
		   if (8 == dlcRx ) {
		      uint32_t j;
		      tick_t currentTick;
		      uint8_t * pCurrentTick = (uint8_t *) &currentTick;
		      for( j = 0; j < 8; j++) {
		    	  pCurrentTick[j] = bufferRx[j];
		      }
		      printf("Command 0x%X received, current tick is %d\r\n", SIG3, currentTick);

		      /* Send the ACK */
		      bufferTx[0] = BOARD_ID;
		      bufferTx[1] = 0x01;
		      dlcTx = 2;
		      delay(300);
		      sapi_can_write(&interface, 0x306, bufferTx, dlcTx);
		   } else {
			   printf("Error receiving command 0x%X\r\n", SIG3);
			   /* dlc different that the one expected, send NACK */
			   bufferTx[0] = BOARD_ID;
			   bufferTx[1] = 0x00;
			   dlcTx = 2;
			   delay(300);
			   sapi_can_write(&interface, 0x306, bufferTx, dlcTx);
		   }
	   }

	   delay(1000);
   }
}
