#ifndef _MODULE_VAR_H
#define _MODULE_VAR_H

////dans .c: #include "variables.h"
//------------------------IMU---------------------
//Adresses registres
#define X_RATE_adress           	0x0400 
#define Y_RATE_adress           	0x0600
#define Z_RATE_adress           	0x0800 
#define X_ACC_adress         	   	0x0A00
#define Y_ACC_adress          		0x0C00 
#define Z_ACC_adress          		0x0E00  
#define X_MAG_adress            	0x1000
#define Y_MAG_adress            	0x1200 
#define Z_MAG_adress            	0x1400 
#define PRODUCT_ID_adress      		0x5600 
#define STDR_REGISTER_adress		0x3400
#define DIAGNOSTIC_STATUS_adress	0x3C00 

#define FACTOR_RATE   200
#define FACTOR_ACC    4000
#define FACTOR_MAG    16384

//Variables
#define ZERO            		0x0000 //Following all commands in order to allow response (NSS)

//------------------Definition des Pins --------------------

#define PIN_DATA_CTRL_AX      PG2 
#define PIN_DATA_CTRL_XM      PG3

//------------------Definition des Moteurs--------------------

#define OD_RIGHT    0
#define OD_LEFT     1
#define AX_RIGHT    1
#define AX_LEFT     2     
#define XM_RIGHT    1
#define XM_LEFT     2 

//------------------Definition des Variables--------------------
// Variables "write" to send over SPI
int wOdRL ; //write Odrive Right LSB
int wOdRM ;//MSB
int wOdLL ;//LSB
int wOdLM ;//MSB
int wAxRL ;//LSB
int wAxRM ;//MSB
int wAxLL ;//LSB
int wAxLM ;//MSB
int wXmRL ;//LSB
int wXmRM ;//MSB
int wXmLL ;//LSB
int wXmLM ;//MSB

// Variables "read" received over SPI
int rOdRL ;//read Odrive Right LSB
int rOdRM ;//MSB
int rOdLL ;//LSB
int rOdLM ;//MSB
int rAxRL ;//LSB
int rAxRM ;//MSB
int rAxLL ;//LSB
int rAxLM ;//MSB
int rXmRL ;//LSB
int rXmRM ;//MSB
int rXmLL ;//LSB
int rXmLM ;//MSB
//IMU
int rXrateM;//LSB
int rXrateL;//MSB
int rYrateM;
int rYrateL;//LSB
int rZrateM;
int rZrateL;//LSB
int rXaccM;
int rXaccL;//LSB
int rYaccM;
int rYaccL;//LSB
int rZaccM;
int rZaccL;//LSB
int rXmagM;
int rXmagL;//LSB
int rYmagM;
int rYmagL;//LSB
int rZmagM;
int rZmagL;//LSB

//variable finales 
int  wOdR ; 
int wOdL ;
int wAxR ;
int wAxL ;
int wXmR ;
int wXmL ;

int rOdR ; 
int rOdL ;
int rAxR ;
int rAxL ;
int rXmR ;
int rXmL ;

//IMU
int16_t rXrate;
int16_t rYrate;
int16_t rZrate;
int16_t rXacc;
int16_t rYacc;
int16_t rZacc;
int16_t rXmag;
int16_t rYmag;
int16_t rZmag;

float  rXrate_scaled;
float   rYrate_scaled;
float   rZrate_scaled;
float   rXacc_scaled;
float   rYacc_scaled;
float   rZacc_scaled;
float   rXmag_scaled;
float   rYmag_scaled;
float   rZmag_scaled;

#endif
