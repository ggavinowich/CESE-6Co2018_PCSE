/*
 * sapi_can.c
 *
 *  Created on: Aug 13, 2018
 *      Author: gabriel
 */

/*==================[inclusions]=============================================*/

#include <string.h>
#include "sapi_can.h"
#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/


/*==================[internal functions declaration]=========================*/

static void sapi_can_set_pinmux(sapi_can_interface_t *interface);
static uint32_t sapi_can_get_power_of_2(uint32_t n);
static void sapi_can_configure_clock(sapi_can_interface_t *interface);
static void sapi_can_module_init(sapi_can_interface_t *interface);
static void sapi_can_set_baudrate(sapi_can_interface_t *interface, uint32_t baudrate);
static void sapi_can_set_baudrate(sapi_can_interface_t *interface, uint32_t baudrate);

/*==================[internal data definition]===============================*/

/*
 * We need to have this global variables so they can be called within the interrupt, since we don't
 * have the "interface" context there.
 */
static sapi_canObject_t * sapi_can1_pData;
static uint32_t sapi_can1_numberOfSignals;

static sapi_canObject_t * sapi_can2_pData;
static uint32_t sapi_can2_numberOfSignals;



/*==================[Interrupts]=============================================*/


/* CAN Interrupt
 *
 * Receives a command and puts it to the buffer from the same id.
 *
*/
void CAN0_IRQHandler(void)
{
   CCAN_MSG_OBJ_T msg_buf;
   uint32_t can_int, can_stat, i;
   while ( (can_int = Chip_CCAN_GetIntID(LPC_C_CAN0)) != 0 ) {
      if (can_int & CCAN_INT_STATUS) {
         can_stat = Chip_CCAN_GetStatus(LPC_C_CAN0);
         // TODO with error or TXOK, RXOK
         if (can_stat & CCAN_STAT_EPASS) {
            DEBUGOUT("Passive error\r\n");
            return;
         }
         if (can_stat & CCAN_STAT_EWARN) {
            DEBUGOUT("Warning!!!\r\n");
            return;
         }
         if (can_stat & CCAN_STAT_BOFF) {
            DEBUGOUT("CAN bus is off\r\n");
            return;
         }
         Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_TXOK);
         Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_RXOK);
      }
      else if ((1 <= CCAN_INT_MSG_NUM(can_int)) && (CCAN_INT_MSG_NUM(can_int) <= 0x20)) {
         // Process msg num canint
         Chip_CCAN_GetMsgObject(LPC_C_CAN0, CCAN_MSG_IF1, can_int, &msg_buf);
         if( (msg_buf.dlc <= 8) && (0 != msg_buf.id) ) {
            for (i = 0; i < sapi_can1_numberOfSignals; i++) {
               if (sapi_can1_pData[i].signalDataBuffer.id == msg_buf.id) {
                  sapi_can1_pData[i].signalDataBuffer.dlc = msg_buf.dlc;
                  memcpy(sapi_can1_pData[i].signalDataBuffer.data, msg_buf.data, msg_buf.dlc);
                  sapi_can1_pData[i].newDataReceived = true;
                  break;
               }
            }
         }
      }
   }
}

/* CAN Interrupt
 *
 * Receives a command and puts it to the buffer from the same id.
 *
*/
void CAN1_IRQHandler(void)
{
   CCAN_MSG_OBJ_T msg_buf;
   uint32_t can_int, can_stat, i;
   while ( (can_int = Chip_CCAN_GetIntID(LPC_C_CAN0)) != 0 ) {
      if (can_int & CCAN_INT_STATUS) {
         can_stat = Chip_CCAN_GetStatus(LPC_C_CAN0);
         // TODO with error or TXOK, RXOK
         if (can_stat & CCAN_STAT_EPASS) {
            DEBUGOUT("Passive error\r\n");
            return;
         }
         if (can_stat & CCAN_STAT_EWARN) {
            DEBUGOUT("Warning!!!\r\n");
            return;
         }
         if (can_stat & CCAN_STAT_BOFF) {
            DEBUGOUT("CAN bus is off\r\n");
            return;
         }
         Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_TXOK);
         Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_RXOK);
      }
      else if ((1 <= CCAN_INT_MSG_NUM(can_int)) && (CCAN_INT_MSG_NUM(can_int) <= 0x20)) {
         // Process msg num canint
         Chip_CCAN_GetMsgObject(LPC_C_CAN0, CCAN_MSG_IF1, can_int, &msg_buf);
         if( (msg_buf.dlc <= 8) && (0 != msg_buf.id) ) {
            for (i = 0; i < sapi_can1_numberOfSignals; i++) {
               if (sapi_can2_pData[i].signalDataBuffer.id == msg_buf.id) {
                  sapi_can2_pData[i].signalDataBuffer.dlc = msg_buf.dlc;
                  memcpy(sapi_can2_pData[i].signalDataBuffer.data, msg_buf.data, msg_buf.dlc);
                  sapi_can2_pData[i].newDataReceived = true;
                  break;
               }
            }
         }
      }
   }
}

/*==================[internal functions definition]==========================*/

/*
 * @brief:  Enable the CAN pin-mux
 * @param:  interface - The interface to set
 * @return: none
*/
static void sapi_can_set_pinmux(sapi_can_interface_t *interface)
{
   /* EDU-CIAA-NXP */
   if (CCAN_1 == interface->canNumber) {
      Chip_SCU_PinMuxSet(0x3, 1, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC2)); /* CAN RD */
      Chip_SCU_PinMuxSet(0x3, 2, (SCU_MODE_INACT | SCU_MODE_FUNC2)); /* CAN TD */
   } else if (CCAN_2 == interface->canNumber) {
      Chip_SCU_PinMuxSet(0x1, 17, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC2)); /* CAN RD */
      Chip_SCU_PinMuxSet(0x1, 18, (SCU_MODE_INACT | SCU_MODE_FUNC2)); /* CAN TD */
   }
}

/*
 * @brief:  Get the next power of 2 of a number
 * @param:  The entry number
 * @return: The next power of 2
*/
static uint32_t sapi_can_get_power_of_2(uint32_t n)
{
  uint32_t k = 1;
  while (k < n)
    k *= 2;
  return k;
}

/* El clock del modulo del can debe ser menor a 50MHz
 *
 * Obtener el clock configurado y agregar los divisores
 *
 */
/*
 * @brief:  Configure the clock of the can interface
 *
 * 			According to the speck, the clock must be a less than 50MHz for CAN
 * 			to function.
 *
 * @param:  The interface to configure
 * @return: none
*/
static void sapi_can_configure_clock(sapi_can_interface_t *interface)
{
   uint32_t divisor, divisorPotencia2;
   SystemCoreClockUpdate();
   divisor = (SystemCoreClock / 50000000);
   divisorPotencia2 = sapi_can_get_power_of_2(divisor);

   if (CCAN_1 == interface->canNumber) {
      Chip_Clock_SetBaseClock(CLK_BASE_APB3, CLKIN_IDIVC, true, false);
      Chip_Clock_SetDivider(CLK_IDIV_C, CLK_BASE_APB3, divisorPotencia2);
   } else if (CCAN_2 == interface->canNumber) {
      //TODO: Test this
      Chip_Clock_SetBaseClock(CLK_BASE_APB1, CLKIN_IDIVC, true, false);
      Chip_Clock_SetDivider(CLK_IDIV_B, CLK_BASE_APB1, divisorPotencia2);
   }
}

/* Init the can interface
 *
 * @param:  The interface to configure
 * @return: none
*/
static void sapi_can_module_init(sapi_can_interface_t *interface)
{
	Chip_CCAN_Init(interface->pCCAN);
}

/* Set the baudrate. Max is 500kbps
 *
 * @param:  The interface to configure
 * 			baudrate - The desired baudrate
 * @return: none
*/
static void sapi_can_set_baudrate(sapi_can_interface_t *interface, uint32_t baudrate)
{
	Chip_CCAN_SetBitRate(interface->pCCAN, baudrate);
}

/* Enable the interrupts
 *
 * @param:  The interface to configure
 *
 * @return: none
*/
static void sapi_can_configure_interrupt(sapi_can_interface_t *interface)
{
   if (CCAN_1 == interface->canNumber) {
      /* Any problem in the bus or any message received will trigger an interrupt */
      Chip_CCAN_EnableInt(LPC_C_CAN0, (CCAN_CTRL_IE | CCAN_CTRL_SIE | CCAN_CTRL_EIE));
      Chip_CCAN_ClearStatus(LPC_C_CAN0, CCAN_STAT_TXOK);
      NVIC_EnableIRQ(C_CAN0_IRQn);
   } else if (CCAN_2 == interface->canNumber) {
      /* Any problem in the bus or any message received will trigger an interrupt */
      Chip_CCAN_EnableInt(LPC_C_CAN1, (CCAN_CTRL_IE | CCAN_CTRL_SIE | CCAN_CTRL_EIE));
      Chip_CCAN_ClearStatus(LPC_C_CAN1, CCAN_STAT_TXOK);
      NVIC_EnableIRQ(C_CAN1_IRQn);
   }
}

/*==================[external functions definition]==========================*/

/* initialize a can interface.
 *
 * @param:  interface 		- An empty pointer to the interface, it will be filled and returned to
 * 							  the user to be used in the subsequent calls.
 * 			interfaceNumber - The number of the interface to initialize, CAN_1, CAN_2, etc.
 * 			baudrate 		- The desired baudrate
 * @return: true if the initialization was successful
*/
bool_t sapi_can_canInit(sapi_can_interface_t *interface, ccan_interfaces_t interfaceNumber, uint32_t baudrate, sapi_canObject_t * pDataObject, uint32_t numberOfSignals, sapi_can_configValues_t * pInitialValuesArray)
{
	uint32_t i;

   if (interfaceNumber >= CCAN_MAX_NUMBER_OF_INTERFACES) {
      return false;
   }

   if (CCAN_1 == interfaceNumber) {
	   interface->canIfRegister = CCAN_MSG_IF1;
	   interface->canNumber = interfaceNumber;
	   interface->pCCAN = LPC_C_CAN0;
   } else if (CCAN_2 == interface->canNumber) {
      interface->canIfRegister = CCAN_MSG_IF2;
      interface->canNumber = interfaceNumber;
      interface->pCCAN = LPC_C_CAN1;
   }

   sapi_can_set_pinmux(interface);

   sapi_can_configure_clock(interface);

   sapi_can_module_init(interface);

   sapi_can_set_baudrate(interface, baudrate);

   /* TODO: Add check to ensure that MAX_NUMBER_OF_CAN1_SIGNALS is lower than CCAN_MSG_MAX_NUM
   	*  	    otherwise there is not going to enought filters to all the signals
    */

   if (CCAN_1 == interfaceNumber) {
      /* Point the global object to the data, this will be used in the interrupt */
      sapi_can1_pData = pDataObject;
      sapi_can1_numberOfSignals = numberOfSignals;
   } else if (CCAN_2 == interface->canNumber) {
      /* Point the global object to the data, this will be used in the interrupt */
      sapi_can2_pData = pDataObject;
      sapi_can2_numberOfSignals = numberOfSignals;
   }

   /* Point to the interface object to the data, this will be used in the regular calls */
   interface->can_signal_buffer = pDataObject;
   interface->numberOfSignals = numberOfSignals;

   /* Start listening for the requested signals, also reset the newData signal */
   for(i = 0; i < numberOfSignals; i++) {
	   interface->can_signal_buffer[i].signalDataBuffer.id = pInitialValuesArray[i].id;
	   interface->can_signal_buffer[i].pCallback = pInitialValuesArray[i].pCallback;
	   Chip_CCAN_AddReceiveID(interface->pCCAN, interface->canIfRegister, interface->can_signal_buffer[i].signalDataBuffer.id);
	   interface->can_signal_buffer[i].newDataReceived = false;
   }

   sapi_can_configure_interrupt(interface);

   return true;
}

/* Write data to the CANbus
 *
 * @param:  interface 		- The interface to write to
 * 			id				- The id of the message to send
 * 			pBuffer 		- Pointer to the data to send, can be null if dlc is zero
 * 			dlc 			- Number of bytes to send
 * @return: true if the data was sent successful
*/
bool_t sapi_can_write(sapi_can_interface_t *interface, uint32_t id, uint8_t * pBuffer, size_t dlc )
{
   CCAN_MSG_OBJ_T send_obj;

   /* verificar que es una interfaz valida */
   if (interface->canNumber >= CCAN_MAX_NUMBER_OF_INTERFACES) {
      return false;
   }

   /* Verificar que no se quieren enviar mas de 8 datos */
   if (8 < dlc) {
      return false;
   } else if ( (0 < dlc) && (NULL == pBuffer) ){
      /* Si quiere enviar datos, el buffer de entrada no puede ser nulo */
      return false;
   }

   send_obj.id = id;
   send_obj.dlc = dlc;

   if (0 < dlc) {
      uint32_t i;
      /* Son pocos datos, se copian con un for */
      for(i = 0; i < dlc; i++) {
         send_obj.data[i] = pBuffer[i];
      }
   }

   /* Enviar los datos */
   Chip_CCAN_Send(interface->pCCAN, interface->canIfRegister, false, &send_obj);

   return true;
}

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
bool sapi_can_read(uint8_t *pData, uint32_t *dlc, sapi_can_interface_t *interface, uint32_t id)
{
	uint32_t i;
	bool_t result = false;
    for (i = 0; i < interface->numberOfSignals; i++) {
    	if ( (interface->can_signal_buffer[i].signalDataBuffer.id == id) && (true == interface->can_signal_buffer[i].newDataReceived)) {
    		memcpy(pData, interface->can_signal_buffer[i].signalDataBuffer.data, interface->can_signal_buffer[i].signalDataBuffer.dlc);
    		*dlc = interface->can_signal_buffer[i].signalDataBuffer.dlc;
    		interface->can_signal_buffer[i].newDataReceived = false;
    		result = true;
    		break;
    	}
    }
    return result;
}

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
void sapi_can_loop(sapi_can_interface_t *interface)
{
	uint32_t i;
	for (i = 0; i < interface->numberOfSignals; i++) {
		/* Check if new data has arrived */
		if ( true == interface->can_signal_buffer[i].newDataReceived) {
			if (SAPI_CAN_NO_CALLBACK != interface->can_signal_buffer[i].pCallback) {
				CCAN_MSG_OBJ_T  localCanObject;
			   /* First we copy the received object to a local container */
			   memcpy(&localCanObject, &interface->can_signal_buffer[i].signalDataBuffer, sizeof(CCAN_MSG_OBJ_T));
				/* Then we clear the flag if the signal was proccessed */
				interface->can_signal_buffer[i].newDataReceived = false;
				/* Execute the callback */
				interface->can_signal_buffer[i].pCallback(&localCanObject);
			}
		}
	}
}

/* We need this function so the config X-MACRO works when no callback is defined.
 * This will never be called by code.
 *
 * @param:  dataObject a pointer to the received information
 * @return: no
*/
void SAPI_CAN_NO_CALLBACK(CCAN_MSG_OBJ_T * dataObject)
{

}
