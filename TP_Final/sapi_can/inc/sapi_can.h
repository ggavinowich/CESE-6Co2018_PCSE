/*
 * sapi_can.h
 *
 *  Created on: Aug 16, 2018
 *      Author: gabriel
 */

#ifndef _SAPI_CAN_H_
#define _SAPI_CAN_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"

/*
 * Function Pointer definition for the can callbacks
 * --------------------------------------------------
 * param:  A can object
 * return: none
 */
typedef void (*callBackCanFuncPtr_t)(CCAN_MSG_OBJ_T * dataObject);

/*
 * Available physical interfaces
 */
typedef enum {
   CCAN_1 = 0,
   CCAN_2 = 1,
   CCAN_MAX_NUMBER_OF_INTERFACES,
}ccan_interfaces_t;

/*
 * Structure of the basic type of the initialization array
 * created by the user.
 *
 */
typedef struct {
	uint32_t id;
	callBackCanFuncPtr_t pCallback;
}sapi_can_configValues_t;

/*
 * Structure of the object that will hold the information
 * received in the interrupt
 *
 */
typedef struct {
   CCAN_MSG_OBJ_T signalDataBuffer;
   bool_t newDataReceived;
   callBackCanFuncPtr_t pCallback;
}sapi_canObject_t;

/* Interface structure.
 *
 *  There should be one for each can interface used
 *
 *
 */
typedef struct {
	ccan_interfaces_t	canNumber;
	LPC_CCAN_T *pCCAN;
	CCAN_MSG_IF_T canIfRegister;
	sapi_canObject_t * can_signal_buffer;
	uint32_t numberOfSignals;
}sapi_can_interface_t;

/*
 * Creates an array of sapi_canObject one for each signal we are going
 * to read.
 * This must be given to the sapi_can_canInit and will be used to create the
 * local buffers.
 *
 */
#define CAN_CreateDataObjects(name, numberOfSignals) \
      sapi_canObject_t name[numberOfSignals]


/* initialize a can interface.
 *
 * @param:  interface 		- An empty pointer to the interface, it will be filled and returned to
 * 							  the user to be used in the subsequent calls.
 * 			interfaceNumber - The number of the interface to initialize, CAN_1, CAN_2, etc.
 * 			baudrate 		- The desired baudrate
 * @return: true if the initialization was successful
 *
 * NOTE: Multiple calls to this function can lead to undesirable behavior.
 * TODO: Add a function to deinit the interface
 *
*/
bool_t sapi_can_canInit(sapi_can_interface_t *interface, ccan_interfaces_t interfaceNumber, uint32_t baudrate, sapi_canObject_t * pDataObject, uint32_t numberOfSignals, sapi_can_configValues_t * pIdArray);

/* Write data to the CANbus
 *
 * @param:  interface 		- The interface to write to
 * 			id				- The id of the message to send
 * 			pBuffer 		- Pointer to the data to send, can be null if dlc is zero
 * 			dlc 			- Number of bytes to send
 * @return: true if the data was sent successful
*/
bool_t sapi_can_write(sapi_can_interface_t *interface, uint32_t id, uint8_t * pBuffer, size_t dlc );

/* Get a message a message already received from the data to the CANbus
 *
 * This function gets the message from the local buffer, it will not wait
 * for the message to appear in the bus
 *
 * @param:  interface 		- The interface to get from
 * 			id				- The id of the message to receive
 * 			pBuffer 		- Buffer to put the reived data, can be null if dlc is zero
 * 			dlc 			- Pointer where the number of received data will be placed
 * @return: true if data was received successful
*/
bool sapi_can_read(uint8_t *pData, uint32_t *dlc, sapi_can_interface_t *interface, uint32_t id);

/* Loops over all the messages. If there is a new message present and a callback is defined
 * then it calls to the callback. In this case, the newMEssage flag es erased.
 * If no callback is defined the newMessage flag keeps the current value
 * so this signal can be queried by polling.
 *
 * We create a local object before calling the callback, so we can receive a new one
 * while we are executing the callback.
 *
 * @param:  interface 		- The interface to get from
 *
 * @return: none
*/
void sapi_can_loop(sapi_can_interface_t *interface);

/* We need this function so the config X-MACRO works when no callback is defined.
 * This will never be called by code.
 *
 * @param:  dataObject a pointer to the received information
 * @return: no
*/
void SAPI_CAN_NO_CALLBACK(CCAN_MSG_OBJ_T * dataObject);


#endif /* EJERCICIOS_PERIPH_CCAN_INC_SAPI_CAN_H_ */
