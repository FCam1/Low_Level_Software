/* 
 * Author: Michael Ring <mail@michael-ring.org>
 * Author: Thomas Ingleby <thomas.c.ingleby@intel.com>
 * Contributors: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 * Modified by F. Caminade the 06/04/2018
 * Copyright (c) 2014 Intel Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Example usage: Display set of patterns on MAX7219 repeately.
 *                Press Ctrl+C to exit
 * 
 * To compile : gcc -o principal principal.c -lmraa -lm
 * 
 */

/* standard headers */
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

/* mraa header */
#include "mraa/spi.h"

#include "../STM32/test_general/variables.h"
#include <math.h>


/* SPI declaration */
#define SPI_BUS 0
#define MSB_FIRST 0
#define SPI_FREQ 3125000 //3.125Mhz (max 10Mhz)


volatile sig_atomic_t flag = 1;

//struct TrameRead  rbuffer;
struct TrameRead* ptr_rbuffer;
//struct TrameWrite  wbuffer;
struct TrameWrite* ptr_wbuffer;


int main(int argc, char** argv)
{

//////////  
	float ph = 1.57;
	
//////////	
    //~ ptr_readbuf=&readbuf;
    //~ ptr_writebuf=&writebuf;
    
    ptr_rbuffer=&rbuffer;
    ptr_wbuffer=&wbuffer;

//////////SPI INITIALISATION///////////////////////////////////////////  
	mraa_result_t status = MRAA_SUCCESS;
    mraa_spi_context spi;

    /* initialize mraa for the platform (not needed most of the times) */
    mraa_init();
    
    /* initialize SPI bus */
    spi = mraa_spi_init(SPI_BUS);
    
    
    if (spi == NULL) {
        fprintf(stderr, "Failed to initialize SPI\n");
        mraa_deinit();
        return EXIT_FAILURE;
    }
    /* set SPI frequency */
    status = mraa_spi_frequency(spi, SPI_FREQ);
    if (status != MRAA_SUCCESS){
        goto err_exit;
    }
    /* set spi mode */
    status = mraa_spi_mode(spi,0); //mode 0
    if (status != MRAA_SUCCESS) {
        goto err_exit;
    }
        
    /* set big endian mode */
    status = mraa_spi_lsbmode(spi,MSB_FIRST); 
    if (status != MRAA_SUCCESS) {
        goto err_exit;
    }

    /* MAX7219/21 chip needs the data in word size */
    status = mraa_spi_bit_per_word(spi,8);
    if (status != MRAA_SUCCESS) {
        fprintf(stdout, "Failed to set SPI Device to 16Bit mode\n");
        goto err_exit;
    }
    
////////////////////////////////////////////////////////////////////////

//////////BOUCLE ENVOIE/RECEPTION SPI///////////////////////////////////////////  
   for(;;)
	{
		
	ph = ph+0.001;

 //To send 
 //wOdR= labs(255*cos(ph));
 wbuffer.wOdR_pos= (8000*(0.5+0.5*cos(ph)));;
 wbuffer.wOdL_pos= (8000*(0.5+0.5*cos(ph)));;
 wbuffer.wAxR_pos=(255*(0.5+0.5*cos(ph)));
 wbuffer.wAxL_pos=(255*(0.5+0.5*cos(ph)));
 wbuffer.wXmR_pos=(4095*(0.5+0.5*cos(ph)));
 wbuffer.wXmL_pos=(4095*(0.5+0.5*cos(ph)));
  
 //wbuffer.wAxR_pos= 50;
 //wbuffer.wAxL_pos= 20;
 //wbuffer.wOdR_pos= 1100;
 //wbuffer.wOdL_pos= 8000;
 //wbuffer.wXmR_pos=11;
 //wbuffer.wXmL_pos=30;                                                                                                                                                                  

  //Envoie et reception des données 8 bits, semble plus adapté avec le STM32 que le 16 bits
  mraa_spi_transfer_buf(spi,(uint8_t*)ptr_wbuffer ,(uint8_t*)ptr_rbuffer,SIZE_BUFFER);   //TX,RX,size 
     
  
  //Mise à l'echelle data IMU
  rXrate_scaled= (float) rbuffer.rXrate / FACTOR_RATE ;
  rYrate_scaled= (float) rbuffer.rYrate / FACTOR_RATE ;
  rZrate_scaled= (float) rbuffer.rZrate / FACTOR_RATE ;
  rXacc_scaled= (float)  rbuffer.rXacc / FACTOR_ACC ;
  rYacc_scaled= (float)  rbuffer.rYacc / FACTOR_ACC ;
  rZacc_scaled= (float)  rbuffer.rZacc / FACTOR_ACC;
  rXmag_scaled= (float)  rbuffer.rXmag / FACTOR_MAG;
  rYmag_scaled= (float)  rbuffer.rYmag / FACTOR_MAG;
  rZmag_scaled= (float)  rbuffer.rZmag / FACTOR_MAG;
  
  printf("----Sent----\n");
  printf(" OD0 %d | OD1 %d | AX1 %d | AX2 %d | XM1 %d | XM2 %d \n",wbuffer.wOdR_pos,wbuffer.wOdL_pos,wbuffer.wAxR_pos,wbuffer.wAxL_pos,wbuffer.wXmR_pos,wbuffer.wXmL_pos);
  
  //printf("----ODrive----\n");
  //printf("Received rOdR %d \n",rxbuffer.rOdR);
  //printf("Received rOdL  %d \n",rxbuffer.rOdL);
  printf("----Reception Position----\n");
  printf(" OD0 %d | OD1 %d | AX1 %d | AX2 %d | XM1 %d | XM2 %d \n",rbuffer.rOdR_pos,rbuffer.rOdL_pos,rbuffer.rAxR_pos,rbuffer.rAxL_pos,rbuffer.rXmR_pos,rbuffer.rXmL_pos);
  printf("----Reception Courant----\n");
  printf(" XM1 %d | XM2 %d \n",rbuffer.rXmR_cur,rbuffer.rXmL_cur);
  printf("----IMU Scaled----\n");
  printf("Received X rate  %f \n",rXrate_scaled);
  printf("Received Y rate  %f \n",rYrate_scaled);
  printf("Received Z rate  %f \n",rZrate_scaled);
  printf("Received X accel %f  \n",rXacc_scaled);
  printf("Received Y accel  %f  \n",rYacc_scaled);
  printf("Received Z accel  %f  \n",rZacc_scaled);
  printf("Received X mag  %f  \n",rXmag_scaled);
  printf("Received Y mag  %f  \n",rYmag_scaled);
  printf("Received Z mag  %f \n",rZmag_scaled);
  printf("----Lecture Directe codeurs----\n");
  printf(" codeur1 %d | codeur2 %d | codeur3 %d | codeur4 %d  \n",rbuffer.rCodRMot,rbuffer.rCodRHip,rbuffer.rCodLMot,rbuffer.rCodLHip);
                                                       
	sleep(0.1); 
	
	}
  
  
  
    /* stop spi */
    mraa_spi_stop(spi);
    //! [Interesting]
    /* deinitialize mraa for the platform (not needed most of the times) */
    mraa_deinit();
    return EXIT_SUCCESS;

err_exit:
    mraa_result_print(status);
    /* stop spi */
    mraa_spi_stop(spi);
    /* deinitialize mraa for the platform (not needed most of the times) */
    mraa_deinit();
    return EXIT_FAILURE;


}

