/*
 * mainCan.c
 *
 *  Created on: Aug 16, 2018
 *      Author: gabriel
 */

#include "sapi.h"               // <= sAPI header
#include "sapi_can.h"
#include "can_cfg_board_1.h"	/* This already includes sapi_can.h */

DEBUG_PRINT_ENABLE
CONSOLE_PRINT_ENABLE

/* Custom define to identify the boards */
#define BOARD_ID  1

/* Create de initialization array with the ids we are going to listen */
CAN_CreateDataObjects(can1_buffer, MAX_NUMBER_OF_CAN1_SIGNALS);


/* Local variables */
static uint8_t bufferRx[8];
static uint32_t dlcRx;
static uint8_t bufferTx[8];
static uint32_t dlcTx;

/* Send the led delay */
static void sendLed(sapi_can_interface_t * interface, uint32_t delayMs)
{
   bufferTx[0] = delayMs & 0xFF;
   bufferTx[1] = (delayMs>>8) & 0xFF;
   bufferTx[2] = (delayMs>>16) & 0xFF;
   bufferTx[3] = (delayMs>>24) & 0xFF;
   dlcTx = 4; /* Four data */
   sapi_can_write(interface, SIG1, bufferTx, dlcTx);
}

int main(void)
{
   uint32_t i;
   uint32_t ledOnTimeMs = 500;

   boardConfig();

   printf("Board number %d\r\n", BOARD_ID);

   sapi_can_interface_t interface;

   sapi_can_canInit(&interface, CCAN_1, 125000, can1_buffer, MAX_NUMBER_OF_CAN1_SIGNALS, can1_initialArray);

   while(1) {

      /* If TEC1 is pressed, then send the command SIG1 to the bus, the first
       * argument will be the time off the led
       */
      if( !gpioRead( TEC1 ) ) {
         if (ledOnTimeMs > 500) {
            ledOnTimeMs-= 500;
         }
         sendLed(&interface, ledOnTimeMs);
         delay(200);
      }

     /* If TEC2 is pressed, then send the command SIG1 to the bus, the first
      * argument will be the time off the led
      */
      if( !gpioRead( TEC2 ) ) {
         ledOnTimeMs+= 500;
         sendLed(&interface, ledOnTimeMs);
         delay(200);
      }

      /* If TEC3 is pressed, then send the COMMAND SIG2 with no data */
      if( !gpioRead( TEC3 ) ) {
         dlcTx = 0; /* No data */
         sapi_can_write(&interface, SIG2, bufferTx, dlcTx);
         delay(200);
      }
      /* If TEC4 is pressed, then send the COMMAND SIG3 with the tickcounter as data */
      if( !gpioRead( TEC4 ) ) {
         uint32_t j;
         tick_t currentTick = tickRead();
         uint8_t * pCurrentTick = (uint8_t *) &currentTick;
         for( j = 0; j < 8; j++) {
            bufferTx[j] = pCurrentTick[j];
         }
         dlcTx = 8; /* No data */
         sapi_can_write(&interface, SIG3, bufferTx, dlcTx);
         delay(500);
      }

       /* Poll for the signals and print the result sent by the other board */
	   if (true == sapi_can_read(bufferRx, &dlcRx, &interface, SIG4) ){
	      uint8_t board = bufferRx[0];
	      bool_t result = bufferRx[1];
		   printf("Command 0x%X %s from board %d\r\n", SIG4, (result)?"ACK":"NACK", board);
	   }
	   if (true == sapi_can_read(bufferRx, &dlcRx, &interface, SIG5) ){
	      uint8_t board = bufferRx[0];
	      bool_t result = bufferRx[1];
	      printf("Command 0x%X %s from board %d\r\n", SIG5, (result)?"ACK":"NACK", board);
	   }
	   if (true == sapi_can_read(bufferRx, &dlcRx, &interface, SIG6) ){
	      uint8_t board = bufferRx[0];
	      bool_t result = bufferRx[1];
	      printf("Command 0x%X %s from board %d\r\n", SIG6, (result)?"ACK":"NACK", board);
	   }
	   if (true == sapi_can_read(bufferRx, &dlcRx, &interface, SIG7) ){
	   	      uint8_t board = bufferRx[0];
	   	      bool_t result = bufferRx[1];
	   	      printf("Command 0x%X %s from board %d\r\n", SIG7, (result)?"ACK":"NACK", board);
	   }
	   if (true == sapi_can_read(bufferRx, &dlcRx, &interface, SIG8) ){
	   	      uint8_t board = bufferRx[0];
	   	      bool_t result = bufferRx[1];
	   	      printf("Command 0x%X %s from board %d\r\n", SIG8, (result)?"ACK":"NACK", board);
	   }
	   if (true == sapi_can_read(bufferRx, &dlcRx, &interface, SIG9) ){
	   	      uint8_t board = bufferRx[0];
	   	      bool_t result = bufferRx[1];
	   	      printf("Command 0x%X %s from board %d\r\n", SIG9, (result)?"ACK":"NACK", board);
	   }

	   delay(100);
   }
}
