// ---------------------------------------------------------------------------
/*
 * Pour rasberry pi, à lancer avec SPI_test.ino
 * reçoit et envoie 4 octets avec concaténation et déconcaténation 16bits
*/
//----------------------------------------------------------------------------
// Example program for bcm2835 library
// Shows how to interface with SPI to transfer a byte to and from an SPI device
//
// After installing bcm2835, you can build this 
// with something like:
// gcc -o spi spi.c -l bcm2835
// sudo ./spi
//
// Or you can test it before installing with:
// gcc -o spi -I ../../src ../../src/bcm2835.c spi.c
// sudo ./spi
//
// Author: Mike McCauley
// Copyright (C) 2012 Mike McCauley
// $Id: RF22.h,v 1.21 2012/05/30 01:51:25 mikem Exp $
//gcc -o spi_test_general spi_test_general.c -l bcm2835 -lm

#include <bcm2835.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "../STM32/test_general/variables.h"
#include <inttypes.h>

//fonctions
int16_t  fconcat (int8_t , int8_t );
int16_t   f_LSB (int8_t octet);
int16_t   f_MSB (int8_t octet);
int SPIbcm2835_setup ();
int i;

struct TrameRead  rbuffer;
struct TrameRead* ptr_rbuffer;

struct TrameWrite  wbuffer;
struct TrameWrite* ptr_wbuffer;


int main(int argc, char **argv)
{
 
 ptr_rbuffer=&rbuffer;
 ptr_wbuffer=&wbuffer;
  
 SPIbcm2835_setup (); // configuration SPI
 float ph = 1.57;


while (1)
{
 ph = ph+0.01;

 //To send 

 //wOdR= labs(255*cos(ph));
 wbuffer.wOdR= 0;
 wbuffer.wOdL= 0;
 wbuffer.wAxR=(255*(0.5+0.5*cos(ph)));
 wbuffer.wAxL=(255*(0.5+0.5*cos(ph)));
 wbuffer.wXmR=(4095*(0.5+0.5*cos(ph)));
 wbuffer.wXmL=(4095*(0.5+0.5*cos(ph)));
 
 
 //wbuffer.wAxR= 50;
 //wbuffer.wAxL= 20;
 
 
 //wbuffer.wOdR= 1100;
 //wbuffer.wOdL= 8000;
 
 //wbuffer.wXmR=11;
 //wbuffer.wXmL=30;
 
 

  //Trame à envoyer 
 // uint8_t TxBuffer [12]={wOdRM,wOdRL,wOdLM,wOdLL,wAxRM,wAxRL,wAxLM,wAxLL,wXmRM,wXmRL,wXmLM,wXmLL};
  
  //Trame à recevoir 
  //uint8_t RxBuffer [22];
  
  //Decoupage en octets avant envoie
  //wOdRM = f_MSB (wOdR);
  //wOdRL = f_LSB (wOdR);
  //wOdLM = f_MSB (wOdL);
  //wOdLL = f_LSB (wOdL);
  //wAxRM = f_MSB (wAxR);
  //wAxRL = f_LSB (wAxR);
  //wAxLM = f_MSB (wAxL);
  //wAxLL = f_LSB (wAxL); 
  //wXmRM = f_MSB (wXmR);
  //wXmRL = f_LSB (wXmR);
  //wXmLM = f_MSB (wXmL);
  //wXmLL = f_LSB (wXmL);                                             
                                    
                                                       
   //Activation SPI
  bcm2835_spi_begin();    
  
  //Envoie et reception des données
  //bcm2835_spi_transfernb(TxBuffer ,RxBuffer,22); 
  
  bcm2835_spi_transfernb(((uint8_t*)ptr_wbuffer) ,((uint8_t*)ptr_rbuffer),SIZE_BUFFER);   //TX,RX,size                                             
  
  
   //Arret SPI
  bcm2835_spi_end();
  
    //RECEPTION
  //Concatenation variables après reception
  //rOdR = fconcat(RxBuffer[0], RxBuffer[1]);
  //rOdL = fconcat(RxBuffer[2], RxBuffer[3]);
  
  //rXrate = fconcat(RxBuffer[4], RxBuffer[5]);
  //rYrate = fconcat(RxBuffer[6], RxBuffer[7]);
  //rZrate = fconcat(RxBuffer[8], RxBuffer[9]);
  //rXacc = fconcat(RxBuffer[10], RxBuffer[11]);
  //rYacc = fconcat(RxBuffer[12], RxBuffer[13]);
  //rZacc = fconcat(RxBuffer[14], RxBuffer[15]);
  //rXmag = fconcat(RxBuffer[16], RxBuffer[17]);
  //rYmag = fconcat(RxBuffer[18], RxBuffer[19]);
  //rZmag = fconcat(RxBuffer[20], RxBuffer[21]);
  
  //Mise à l'echelle data IMU
  //rXrate_scaled= (float) rXrate/ FACTOR_RATE ;
  //rYrate_scaled= (float) rYrate / FACTOR_RATE ;
  //rZrate_scaled= (float) rZrate / FACTOR_RATE ;
  //rXacc_scaled= (float) rXacc / FACTOR_ACC ;
  //rYacc_scaled= (float) rYacc / FACTOR_ACC ;
  //rZacc_scaled= (float) rZacc / FACTOR_ACC;
  //rXmag_scaled= (float) rXmag / FACTOR_MAG;
  //rYmag_scaled= (float) rYmag / FACTOR_MAG;
  //rZmag_scaled= (float) rZmag / FACTOR_MAG;
  
  
  
  printf("----Sent----\n");
  printf(" OD0 %d | OD1 %d | AX1 %d | AX2 %d | XM1 %d | XM2 %d \n",wbuffer.wOdR,wbuffer.wOdL,wbuffer.wAxR,wbuffer.wAxL,wbuffer.wXmR,wbuffer.wXmL);
  
  //printf("----ODrive----\n");
  //printf("Received rOdR %d \n",rxbuffer.rOdR);
  //printf("Received rOdL  %d \n",rxbuffer.rOdL);
  
  //printf("----IMU Scaled----\n");
  //printf("Received X rate  %f \n",rXrate_scaled);
  //printf("Received Y rate  %f \n",rYrate_scaled);
  //printf("Received Z rate  %f \n",rZrate_scaled);
  //printf("Received X accel %f  \n",rXacc_scaled);
  //printf("Received Y accel  %f  \n",rYacc_scaled);
  //printf("Received Z accel  %f  \n",rZacc_scaled);
  //printf("Received X mag  %f  \n",rXmag_scaled);
  //printf("Received Y mag  %f  \n",rYmag_scaled);
  //printf("Received Z mag  %f \n",rZmag_scaled);                                                     
  delay(10);                                                   
                                                       
    


}

return 0;
}

//Initialisation SPI
int SPIbcm2835_setup (){
    if (!bcm2835_init())
  {
  printf("bcm2835_init failed. Are you running as root??\n");
  return 1;
  }
  if (!bcm2835_spi_begin())
  {
  printf("bcm2835_spi_begin failed. Are you running as root??\n");
  return 1;
  }
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);      // The default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                   // The default *MODE1 caused problems with maple mini*
  //~ bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536); // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128); // 3.125MHz
  //bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256); // 1.5625MHz
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                      // The default
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0,LOW);      // the default 

 
}



int16_t fconcat (int8_t octet_msb, int8_t octet_lsb){
  int16_t val_concat = (octet_msb<<8 | octet_lsb); // concatenation des 2 octets de 8 bits
  return val_concat;
 }
 
int16_t f_MSB (int8_t word){
  return (word>>8); //on supprime les bits de poids faibles
 }
 
int16_t   f_LSB (int8_t  word){
    
  return (word & 0x00FF);//avec un masque on supprime les bits de poids forts
 }
 

 



