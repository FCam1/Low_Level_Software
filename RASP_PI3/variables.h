#ifndef _MODULE_VAR_H
#define _MODULE_VAR_H

////dans .c: #include "variables.h"
#include <stdint.h>


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
#define SIZE_BUFFER           12 //Number of variable x2 
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
//int wOdRL ; //write Odrive Right LSB
//int wOdRM ;//MSB
//int wOdLL ;//LSB
//int wOdLM ;//MSB
//int wAxRL ;//LSB
//int wAxRM ;//MSB
//int wAxLL ;//LSB
//int wAxLM ;//MSB
//int wXmRL ;//LSB
//int wXmRM ;//MSB
//int wXmLL ;//LSB
//int wXmLM ;//MSB

//// Variables "read" received over SPI
//int rOdRL ;//read Odrive Right LSB
//int rOdRM ;//MSB
//int rOdLL ;//LSB
//int rOdLM ;//MSB
//int rAxRL ;//LSB
//int rAxRM ;//MSB
//int rAxLL ;//LSB
//int rAxLM ;//MSB
//int rXmRL ;//LSB
//int rXmRM ;//MSB
//int rXmLL ;//LSB
//int rXmLM ;//MSB
////IMU
//int rXrateM;//LSB
//int rXrateL;//MSB
//int rYrateM;
//int rYrateL;//LSB
//int rZrateM;
//int rZrateL;//LSB
//int rXaccM;
//int rXaccL;//LSB
//int rYaccM;
//int rYaccL;//LSB
//int rZaccM;
//int rZaccL;//LSB
//int rXmagM;
//int rXmagL;//LSB
//int rYmagM;
//int rYmagL;//LSB
//int rZmagM;
//int rZmagL;//LSB

////variable finales 
//int  wOdR ; 
//int wOdL ;
//int wAxR ;
//int wAxL ;
//int wXmR ;
//int wXmL ;

//int rOdR ; 
//int rOdL ;
//int rAxR ;
//int rAxL ;
//int rXmR ;
//int rXmL ;

//IMU
//int16_t rXrate;
//int16_t rYrate;
//int16_t rZrate;
//int16_t rXacc;
//int16_t rYacc;
//int16_t rZacc;
//int16_t rXmag;
//int16_t rYmag;
//int16_t rZmag;

float  rXrate_scaled;
float   rYrate_scaled;
float   rZrate_scaled;
float   rXacc_scaled;
float   rYacc_scaled;
float   rZacc_scaled;
float   rXmag_scaled;
float   rYmag_scaled;
float   rZmag_scaled;

struct TrameWrite {


//variable finales 
int16_t wOdR ; 
int16_t wOdL ;
int16_t wAxR ;
int16_t wAxL ;
int16_t wXmR ;
int16_t wXmL ;
};

struct TrameRead  {
   //Odrive
  int8_t rOdRL ;//read Odrive Right LSB
  int8_t rOdRM ;//MSB
  int8_t rOdLL ;//LSB
  int8_t rOdLM ;//MSB*
   //AX
  int8_t rAxRL ;//LSB
  int8_t rAxRM ;//MSB
  int8_t rAxLL ;//LSB
  int8_t rAxLM ;//MSB
    //XM
  int8_t rXmRL ;//LSB
  int8_t rXmRM ;//MSB
  int8_t rXmLL ;//LSB
  int8_t rXmLM ;//MSB
  //IMU
  //int8_t rXrateM;//LSB
  //int8_t rXrateL;//MSB
  //int8_t rYrateM;
  //uint8_t rYrateL;//LSB
  //uint8_t rZrateM;
  //uint8_t rZrateL;//LSB
  //uint8_t rXaccM;
  //uint8_t rXaccL;//LSB
  //uint8_t rYaccM;
  //uint8_t rYaccL;//LSB
  //uint8_t rZaccM;
  //uint8_t rZaccL;//LSB
  //uint8_t rXmagM;
  //uint8_t rXmagL;//LSB
  //uint8_t rYmagM;
  //uint8_t rYmagL;//LSB
  //uint8_t rZmagM;
  //uint8_t rZmagL;//LSB

  int8_t rXrate;
  int8_t rYrate;
  int8_t rZrate;
  int8_t rXacc;
  int8_t rYacc;
  int8_t rZacc;
  int8_t rXmag;
  int8_t rYmag;
  int8_t rZmag;
};
#endif
