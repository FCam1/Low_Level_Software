#ifndef _MODULE_VAR_H
#define _MODULE_VAR_H

#include <stdint.h>
//------------------------MACROS---------------------
#define f_MSB(var)  ((var) >>  8)
#define f_LSB(var)  ((var) & 0x00FF)


////dans .c: #include "variables.h"
//------------------------IMU---------------------
//Adresses registres
#define X_RATE_adress           	      0x0400 
#define Y_RATE_adress           	      0x0600
#define Z_RATE_adress           	      0x0800 
#define X_ACC_adress         	        	0x0A00
#define Y_ACC_adress          		        0x0C00 
#define Z_ACC_adress          		        0x0E00  
#define X_MAG_adress            	      0x1000
#define Y_MAG_adress                  	0x1200 
#define Z_MAG_adress                   	0x1400 
#define PRODUCT_ID_adress      		0x5600 
#define STDR_REGISTER_adress		  0x3400
#define DIAGNOSTIC_STATUS_adress	0x3C00 

#define FACTOR_RATE               200
#define FACTOR_ACC                4000
#define FACTOR_MAG                16384

//Variables
#define ZERO            		0x0000 //Following all commands in order to allow response (NSS)

#define CS_PIN              PA4
//------------------Definition Baudrate --------------------
#define BAUD_AX                 1000000 
#define BAUD_XM                 1000000
#define BAUD_ODRIVE           115200
//------------------SPI --------------------
#define SIZE_BUFFER          12
//------------------Definition des Pins --------------------
#define PIN_DATA_CTRL_AX       PG2 
#define PIN_DATA_CTRL_XM      PG3
//------------------Definition des Moteurs--------------------

#define OD_RIGHT    0
#define OD_LEFT     1
#define AX_RIGHT    1
#define AX_LEFT     2     
#define XM_RIGHT    1
#define XM_LEFT     2 
#define ALLXM       254 // Broadcast ID

//------------------Definition des Variables--------------------
// Variables "write" to send over SPI
//uint8_t wOdRL ; //write Odrive Right LSB
//uint8_t wOdRM ;//MSB
//uint8_t wOdLL ;//LSB
//uint8_t wOdLM ;//MSB
//uint8_t wAxRL ;//LSB
//uint8_t wAxRM ;//MSB
//uint8_t wAxLL ;//LSB
//uint8_t wAxLM ;//MSB
//uint8_t wXmRL ;//LSB
//uint8_t wXmRM ;//MSB
//uint8_t wXmLL ;//LSB
//uint8_t wXmLM ;//MSB

// Variables "read" received over SPI
//uint8_t rOdRL ;//read Odrive Right LSB
//uint8_t rOdRM ;//MSB
//uint8_t rOdLL ;//LSB
//uint8_t rOdLM ;//MSB
//uint8_t rAxRL ;//LSB
//uint8_t rAxRM ;//MSB
//uint8_t rAxLL ;//LSB
//uint8_t rAxLM ;//MSB
//uint8_t rXmRL ;//LSB
//uint8_t rXmRM ;//MSB
//uint8_t rXmLL ;//LSB
//uint8_t rXmLM ;//MSB
////IMU
uint8_t rXrateM;//LSB
uint8_t rXrateL;//MSB
uint8_t rYrateM;
uint8_t rYrateL;//LSB
uint8_t rZrateM;
uint8_t rZrateL;//LSB
uint8_t rXaccM;
uint8_t rXaccL;//LSB
uint8_t rYaccM;
uint8_t rYaccL;//LSB
uint8_t rZaccM;
uint8_t rZaccL;//LSB
uint8_t rXmagM;
uint8_t rXmagL;//LSB
uint8_t rYmagM;
uint8_t rYmagL;//LSB
uint8_t rZmagM;
uint8_t rZmagL;//LSB

////variable finales 
//uint16_t wOdR ; 
//uint16_t wOdL ;
//uint16_t wAxR ;
//uint16_t wAxL ;
//uint16_t wXmR ;
//uint16_t wXmL ;

uint16_t rOdR ; 
uint16_t rOdL ;
uint16_t rAxR ;
uint16_t rAxL ;
uint16_t rXmR ;
uint16_t rXmL ;

//IMU
//uint8_t rXrate;
//uint8_t rYrate;
//uint8_t rZrate;
//uint8_t rXacc;
//uint8_t rYacc;
//uint8_t rZacc;
//uint8_t rXmag;
//uint8_t rYmag;
//uint8_t rZmag;

uint8_t DIAGNOSTIC_STATUS;

//------------------structures--------------------

struct TrameWrite {
//uint8_t wOdRL ; //write Odrive Right LSB
//uint8_t wOdRM ;//MSB
//uint8_t wOdLL ;//LSB
//uint8_t wOdLM ;//MSB
//uint8_t wAxRL ;//LSB
//uint8_t wAxRM ;//MSB
//uint8_t wAxLL ;//LSB
//uint8_t wAxLM ;//MSB
//uint8_t wXmRL ;//LSB
//uint8_t wXmRM ;//MSB
//uint8_t wXmLL ;//LSB
//uint8_t wXmLM ;//MSB

//variable finales 
uint16_t wOdR ; 
uint16_t wOdL ;
uint16_t wAxR ;
uint16_t wAxL ;
uint16_t wXmR ;
uint16_t wXmL ;
};


struct TrameRead {
   //Odrive
  uint8_t rOdRL ;//read Odrive Right LSB
  uint8_t rOdRM ;//MSB
  uint8_t rOdLL ;//LSB
  uint8_t rOdLM ;//MSB*
   //AX
  uint8_t rAxRL ;//LSB
  uint8_t rAxRM ;//MSB
  uint8_t rAxLL ;//LSB
  uint8_t rAxLM ;//MSB
    //XM
  uint8_t rXmRL ;//LSB
  uint8_t rXmRM ;//MSB
  uint8_t rXmLL ;//LSB
  uint8_t rXmLM ;//MSB
  //IMU
  uint8_t rXrateM;//LSB
  uint8_t rXrateL;//MSB
  uint8_t rYrateM;
  uint8_t rYrateL;//LSB
  uint8_t rZrateM;
  uint8_t rZrateL;//LSB
  uint8_t rXaccM;
  uint8_t rXaccL;//LSB
  uint8_t rYaccM;
  uint8_t rYaccL;//LSB
  uint8_t rZaccM;
  uint8_t rZaccL;//LSB
  uint8_t rXmagM;
  uint8_t rXmagL;//LSB
  uint8_t rYmagM;
  uint8_t rYmagL;//LSB
  uint8_t rZmagM;
  uint8_t rZmagL;//LSB

  uint8_t rXrate;
  uint8_t rYrate;
  uint8_t rZrate;
  uint8_t rXacc;
  uint8_t rYacc;
  uint8_t rZacc;
  uint8_t rXmag;
  uint8_t rYmag;
  uint8_t rZmag;
};
#endif
