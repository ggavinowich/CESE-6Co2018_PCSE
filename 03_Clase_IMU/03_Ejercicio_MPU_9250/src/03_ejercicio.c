/* Copyright 2016, Alejandro Permingeat.
 * Copyright 2016, Eric Pernia.
 * Copyright 2018, Sergio Renato De Jesus Melean <sergiordj@gmail.com>.
 *
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Date: 2018-07-06 */

/*==================[inclusions]=============================================*/

#include "sapi.h"               // <= sAPI header
#include <string.h>

/*==================[macros and definitions]=================================*/

#define UART_BLUETOOTH UART_232
#define UART_PC        UART_USB

#define FRAME_DELAY     50

/*==================[internal data declaration]==============================*/

DEBUG_PRINT_ENABLE
CONSOLE_PRINT_ENABLE

// MPU9250 Address
MPU9250_address_t addr = MPU9250_ADDRESS_0; // If MPU9250 AD0 pin is connected to GND

/*==================[internal functions declaration]=========================*/
bool_t hm10bleTest( int32_t uart );
/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/


bool_t hm10bleTest( int32_t uart )
{
   uartWriteString( uart, "AT\r\n" );
   return waitForReceiveStringOrTimeoutBlocking( uart,
                                                 "OK\r\n", strlen("OK\r\n"),
                                                 50 );
}

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void){
   /* ------------- INICIALIZACIONES ------------- */
   uint8_t buffer[128];
   boardConfig();

   // Inicializar UART_USB para conectar a la PC
   debugPrintConfigUart( UART_PC, 9600 );
   debugPrintlnString( "UART_PC configurada." );

   // Inicializar UART_232 para conectar al modulo bluetooth
   consolePrintConfigUart( UART_BLUETOOTH, 9600 );
   debugPrintlnString( "UART_BLUETOOTH para modulo Bluetooth configurada." );


   uartWriteString( UART_PC, "Testeto si el modulo esta conectado enviando: AT\r\n" );
   if( hm10bleTest( UART_BLUETOOTH ) ){
      debugPrintlnString( "Modulo conectado correctamente." );
   }

   // Inicializar la IMU
   printf("Inicializando IMU MPU9250...\r\n" );
   int8_t status;
   status = mpu9250Init( addr );

   if( status < 0 ){
      printf( "IMU MPU9250 no inicializado, chequee las conexiones:\r\n\r\n" );
      printf( "MPU9250 ---- EDU-CIAA-NXP\r\n\r\n" );
      printf( "    VCC ---- 3.3V\r\n" );
      printf( "    GND ---- GND\r\n" );
      printf( "    SCL ---- SCL\r\n" );
      printf( "    SDA ---- SDA\r\n" );
      printf( "    AD0 ---- GND\r\n\r\n" );
      printf( "Se detiene el programa.\r\n" );
      while(1);
   }
   printf("IMU MPU9250 inicializado correctamente.\r\n\r\n" );

   /* ------------- REPETIR POR SIEMPRE ------------- */
   while(TRUE){

      //Leer el sensor y guardar en estructura de control
      mpu9250Read();
      sprintf(buffer, "G_X %f\r\n", mpu9250GetGyroX_rads());
      uartWriteString( UART_BLUETOOTH, buffer);
      delay(FRAME_DELAY); // Delay or the commands are merger in the received
      sprintf(buffer, "G_Y %f\r\n", mpu9250GetGyroY_rads());
      uartWriteString( UART_BLUETOOTH, buffer);
      delay(FRAME_DELAY);
      sprintf(buffer, "G_Z %f\r\n", mpu9250GetGyroZ_rads());
      uartWriteString( UART_BLUETOOTH, buffer);
      delay(FRAME_DELAY);
      sprintf(buffer, "A_X %f\r\n", mpu9250GetAccelX_mss());
      uartWriteString( UART_BLUETOOTH, buffer);
      delay(FRAME_DELAY);
      sprintf(buffer, "A_Y %f\r\n", mpu9250GetAccelY_mss());
      uartWriteString( UART_BLUETOOTH, buffer);
      delay(FRAME_DELAY);
      sprintf(buffer, "A_Z %f\r\n", mpu9250GetAccelZ_mss());
      uartWriteString( UART_BLUETOOTH, buffer);
      delay(FRAME_DELAY);
      sprintf(buffer, "M_X %f\r\n", mpu9250GetMagX_uT());
      uartWriteString( UART_BLUETOOTH, buffer);
      delay(FRAME_DELAY);
      sprintf(buffer, "M_Y %f\r\n", mpu9250GetMagY_uT());
      uartWriteString( UART_BLUETOOTH, buffer);
      delay(FRAME_DELAY);
      sprintf(buffer, "M_Z %f\r\n", mpu9250GetMagZ_uT());
      uartWriteString( UART_BLUETOOTH, buffer);
      delay(FRAME_DELAY);
      sprintf(buffer, "TEM %f\r\n", mpu9250GetTemperature_C());
      uartWriteString( UART_BLUETOOTH, buffer);



      // Imprimir resultados
      printf( "Giroscopo:      (%f, %f, %f)   [rad/s]\r\n",
              mpu9250GetGyroX_rads(),
              mpu9250GetGyroY_rads(),
              mpu9250GetGyroZ_rads()
            );

		printf( "Acelerometro:   (%f, %f, %f)   [m/s2]\r\n",
              mpu9250GetAccelX_mss(),
              mpu9250GetAccelY_mss(),
              mpu9250GetAccelZ_mss()
            );

		printf( "Magnetometro:   (%f, %f, %f)   [uT]\r\n",
              mpu9250GetMagX_uT(),
              mpu9250GetMagY_uT(),
              mpu9250GetMagZ_uT()
            );

		printf( "Temperatura:    %f   [C]\r\n\r\n",
              mpu9250GetTemperature_C()
            );

      delay(1000);
   }

   /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
      por ningun S.O. */
   return 0 ;
}

/*==================[end of file]============================================*/
