/* Copyright 2014, ChaN
 * Copyright 2016, Matias Marando
 * Copyright 2016, Eric Pernia
 * All rights reserved.
 *
 * This file is part of Workspace.
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
 *
 */

/*==================[inlcusiones]============================================*/

#include "sd_spi.h"   // <= own header (optional)
#include "sapi.h"     // <= sAPI header

#include "ff.h"       // <= Biblioteca FAT FS

/*==================[definiciones y macros]==================================*/

#define FILENAME "muestras.txt"


#define NUMERO_MUESTRAS 3
/*==================[definiciones de datos internos]=========================*/

static FATFS fs;           // <-- FatFs work area needed for each volume
static FIL fp;             // <-- File object needed for each open file

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// FUNCION que se ejecuta cada vezque ocurre un Tick
void diskTickHook( void *ptr );


/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.

 */
char* itoa(int value, char* result, int base) {
   // check that the base if valid
   if (base < 2 || base > 36) { *result = '\0'; return result; }

   char* ptr = result, *ptr1 = result, tmp_char;
   int tmp_value;

   do {
      tmp_value = value;
      value /= base;
      *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
   } while ( value );

   // Apply negative sign
   if (tmp_value < 0) *ptr++ = '-';
   *ptr-- = '\0';
   while(ptr1 < ptr) {
      tmp_char = *ptr;
      *ptr--= *ptr1;
      *ptr1++ = tmp_char;
   }
   return result;
}

/* Escribir dia y hora en formato "YYYY/MM/DD_hh:mm:ss" en puntero pBuffer */
void getDateAndTime( rtc_t * rtc, uint8_t * pBuffer ) {

   uint8_t bufferAux[10];
   uint32_t offset = 0;

   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->year), (char*)bufferAux, 10 ); /* 10 significa decimal */
   /* Guardo el año */
   memcpy(pBuffer, bufferAux, strlen(bufferAux));

   offset += 4;

   pBuffer[offset] = '/';

   offset += 1;

   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->month), (char*)bufferAux, 10 ); /* 10 significa decimal */
   /* Guardo el mes */
   if( (rtc->month)<10 ) {
      memcpy(pBuffer+offset, "0", 1);
      offset++;
   }
   memcpy(pBuffer+offset, bufferAux, strlen(bufferAux));

   offset += strlen(bufferAux);

   pBuffer[offset] = '/';

   offset += 1;


   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->mday), (char*)bufferAux, 10 ); /* 10 significa decimal */
   /* Guardo el dia */
   if( (rtc->mday)<10 ) {
         memcpy(pBuffer+offset, "0", 1);
         offset++;
   }
   memcpy(pBuffer+offset, bufferAux, strlen(bufferAux));

   offset += strlen(bufferAux);

   pBuffer[offset] = '_';

   offset += 1;


   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->hour), (char*)bufferAux, 10 ); /* 10 significa decimal */
   /* Guardo la hora */
   if( (rtc->hour)<10 ) {
      memcpy(pBuffer+offset, "0", 1);
      offset++;
   }
   memcpy(pBuffer+offset, bufferAux, strlen(bufferAux));
   offset += strlen(bufferAux);
   pBuffer[offset] = ':';
   offset += 1;

   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->min), (char*)bufferAux, 10 ); /* 10 significa decimal */
   /* Guardo los minutos */
   if( (rtc->min)<10 ) {
      memcpy(pBuffer+offset, "0", 1);
      offset++;
   }
   memcpy(pBuffer+offset, bufferAux, strlen(bufferAux));
   offset += strlen(bufferAux);
   pBuffer[offset] = ':';
   offset += 1;

   /* Conversion de entero a ascii con base decimal */
   itoa( (int) (rtc->sec), (char*)bufferAux, 10 ); /* 10 significa decimal */
   /* Guardo los segundos */
   if( (rtc->sec)<10 ) {
      memcpy(pBuffer+offset, "0", 1);
      offset++;
   }
   memcpy(pBuffer+offset, bufferAux, strlen(bufferAux));
   offset += strlen(bufferAux);

   pBuffer[offset] = '\0';
}



/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   /* Variable para almacenar el valor leido del ADC CH1 */
   int16_t muestra[NUMERO_MUESTRAS] = {0};

   /* Variables de delay no bloqueantes */
   delay_t delay1;

   /* Buffer */
   char bufferMuestras[10];
   char bufferRtc[48];

   /* Estructura RTC */
   rtc_t rtc;

   bool_t val = 0;
   uint8_t i = 0;

   // Inicializar UART_USB como salida de consola
   uartConfig(UART_USB, 115200);

   // SPI configuration
   spiConfig( SPI0 );

   // Inicializar el conteo de Ticks con resolucion de 10ms,
   // con tickHook diskTickHook
   tickConfig( 10 );
   tickCallbackSet( diskTickHook, NULL );

   rtc.year = 2018;
   rtc.month = 7;
   rtc.mday = 6;
   rtc.wday = 6;
   rtc.hour = 19;
   rtc.min = 34;
   rtc.sec= 0;

   /* Inicializar RTC */
   val = rtcConfig( &rtc );

   delay_t delay1s;
   delayConfig( &delay1s, 1000 );

   delay(12000); // El RTC tarda en setear la hora, por eso el delay tan grande

   // ------ PROGRAMA QUE ESCRIBE EN LA SD CH1;CH2;CH3;YYYY/MM/DD_hh:mm:ss;  -------

   UINT nbytes;

   // Give a work area to the default drive
   if( f_mount( &fs, "", 0 ) != FR_OK ){
      // If this fails, it means that the function could
      // not register a file system object.
      // Check whether the SD card is correctly connected
      printf("No se pudo montar\r\n");
  }



   adcConfig( ADC_ENABLE ); /* Habiltar ADC */

   /* Inicializar Retardo no bloqueante con tiempo en ms */
   delayConfig( &delay1,  500 );

   while( TRUE )
   {

      /* delayRead retorna TRUE cuando se cumple el tiempo de retardo */
      if ( delayRead( &delay1 ) ) {

         /* Leo la Entrada Analogica AI0 - ADC0 CH1 */
         muestra[0] = adcRead( CH1 );
         /* Leo la Entrada Analogica AI1 - ADC0 CH2 */
         muestra[1] = adcRead( CH2 );
         /* Leo la Entrada Analogica AI2 - ADC0 CH3 */
         muestra[2] = adcRead( CH3 );

         val = rtcRead( &rtc );
         /* Convierto la fecha a string */
         getDateAndTime(&rtc, bufferRtc);
   
         // Create/open a file, then write a string and close it
         if( f_open( &fp, FILENAME, FA_WRITE | FA_OPEN_APPEND ) == FR_OK ) {
            uint8_t i;

            for (i = 0; i < NUMERO_MUESTRAS; i++) {
               /* Conversión de la muestra entera a ascii con base decimal */
               itoa( muestra[i], bufferMuestras, 10 ); /* 10 significa decimal */
               /* Escribo la muestra */
               f_write( &fp, bufferMuestras, strlen(bufferMuestras), &nbytes );
               f_write( &fp, ";", 1, &nbytes ); /* LA ultima muestra tendrá el ; no parece grave */
               printf("%s;", bufferMuestras);
            }
            f_write( &fp, bufferRtc, strlen(bufferRtc), &nbytes );
            f_write( &fp, "\r\n", 2, &nbytes );
            printf("%s\r\n", bufferRtc);
            /* Cierro hasta la siguiente muestra */
            f_close(&fp);
         } else {
            printf("fopen failed\r\n");
         }
      }
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado/ por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// FUNCION que se ejecuta cada vezque ocurre un Tick
void diskTickHook( void *ptr ){
   disk_timerproc();   // Disk timer process
}


/*==================[fin del archivo]========================================*/
