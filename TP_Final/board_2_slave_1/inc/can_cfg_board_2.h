/*
 * can_cfg.h
 *
 *  Created on: Aug 21, 2018
 *      Author: ggabriel
 */

#ifndef EJERCICIOS_PERIPH_CCAN_SRC_CAN_CFG_H_
#define EJERCICIOS_PERIPH_CCAN_SRC_CAN_CFG_H_


#include "sapi_datatypes.h"
#include "sapi_can.h"

/* X-Macro to configure the signals */
#define CAN1_RECEIVED_SIGNALS\
   CAN1_SIGNAL(SIG1, 0x301, SAPI_CAN_NO_CALLBACK) \
   CAN1_SIGNAL(SIG2, 0x302, SAPI_CAN_NO_CALLBACK) \
   CAN1_SIGNAL(SIG3, 0x303, SAPI_CAN_NO_CALLBACK)

#define CAN1_TRANSMITTED_SIGNALS \
   CAN1_SIGNAL(SIG4, 0x304) \
   CAN1_SIGNAL(SIG5, 0x305) \
   CAN1_SIGNAL(SIG6, 0x306)

/**************** Under the hood work with the x-macros **************************/

/* X-MACRO enum to asociate the receive signal name with the signal id */
#undef CAN1_SIGNAL
#define  CAN1_SIGNAL(name, id, callback) \
				name = id,

typedef enum {
	CAN1_RECEIVED_SIGNALS
}can_rx_signal_by_name_t;


/* X-MACRO enum to asociate the transmit signal name with the signal id */
#undef CAN1_SIGNAL
#define  CAN1_SIGNAL(name, id) \
				name = id,

typedef enum {
	CAN1_TRANSMITTED_SIGNALS
}can_tx_signal_by_name_t;


/* X-MACRO Enum to get the number of configured signals */
#undef CAN1_SIGNAL
#define CAN1_SIGNAL(name, id, callback) \
            CAN1_SIGNAL_##name,

typedef enum {
   CAN1_RECEIVED_SIGNALS
   MAX_NUMBER_OF_CAN1_SIGNALS,
}presentSignals_t;

/* X-MACRO array to declare the external callback functions */
#undef CAN1_SIGNAL
#define CAN1_SIGNAL(name, id, callback)\
		extern void callback(CCAN_MSG_OBJ_T * dataObject);

CAN1_RECEIVED_SIGNALS

/* X-MACRO array to initialize the can objects */
#undef CAN1_SIGNAL
#define  CAN1_SIGNAL(name, id, callback) \
            { name, callback },

sapi_can_configValues_t can1_initialArray[MAX_NUMBER_OF_CAN1_SIGNALS] = { CAN1_RECEIVED_SIGNALS };


#endif /* EJERCICIOS_PERIPH_CCAN_SRC_CAN_CFG_H_ */
