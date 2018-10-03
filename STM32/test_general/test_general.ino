#include <SPI.h>
#include <time.h>
#include <libmaple/dma.h>
#include <libmaple/usart.h>
#include "codeurs.h"
#include "variables.h"

//macro
#define lowPin() (gpio_write_bit(GPIOB, 12, 0))  //PB12 LOW
#define highPin() (gpio_write_bit(GPIOB, 12, 1)) //PB12 HIGH
/*methodes digital write :
  digitalWrite(PA15, LOW); lent
  gpio_write_bit(GPIOA,15,0); rapide
  GPIOA->regs->BRR=(1<<15); rapide
*/
#define testFlag(flag) ((ptr_wbuffer->w_flag & flag) != 0) // test flag

//structures
struct TrameWrite *ptr_wbuffer = &wbuffer;
struct TrameRead *ptr_rbuffer = &rbuffer;

//-----------------------------------------------------------
//----------------------DYNAMIXEL AX------------------------
//-----------------------------------------------------------
#include <SavageDynamixelSerial_Upgraded.h> ///ATENTION SERIAL 4 || Tx, Rx pins have to be modified in DynamixelSerial_Modified.cpp ||
DynamixelClass Dynamixel(Serial4);
//-----------------------------------------------------------
//----------------------DYNAMIXEL XM------------------------
//-----------------------------------------------------------
DynamixelXClass DynamixelX(Serial5);
//-----------------------------------------------------------
//----------------------SPI----------------------------------
//-----------------------------------------------------------
SPIClass SPI_1(1); //Create an instance of the SPI Class called SPI_1 that uses the SPI Port 1
SPIClass SPI_2(2); //Create an instance of the SPI Class called SPI_2 that uses the SPI Port 2
//-----------------------------------------------------------
//----------------------ODrive----------------------------------
//-----------------------------------------------------------
#include <ODriveArduino.h>
ODriveArduino odrive(Serial3);
template <class T>  inline Print &operator<<(Print &obj, T arg)
{
  obj.print(arg);
  return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
  obj.print(arg, 4);
  return obj;
}
//DMA
#define LENGHT  48 //50
#define dma_bufer_size2 LENGHT
dma_tube_config tube_config;
char dma_reg_Od[LENGHT];

//-----------------------------------------------------------
//-----------------------------------------------------------

//----------MESURE TEMPS PROGRAMME--------------
//-----------------------------------------------------------

long int top_chrono;
void demarrer_chrono()
{
  top_chrono = micros(); //mesure le temps ecoule en micro secondes depuis le demarrage du programme
}
void stop_chrono()
{
  long int arret_chrono = micros();
  float duree = ((arret_chrono - top_chrono) / (CLOCKS_PER_SEC * 10000.0));
  Serial.print("Le calcul a pris en secondes :");
  Serial.println(duree, 6);
  Serial.print("Soit en ms :");
  Serial.println(duree * 1000);
  Serial.print("Soit en us :");
  Serial.println(duree * 1000000);
}
//----------------------SETUP SPIs------------------------------
//-----------------------------------------------------------
void setupSPI2()
{
  // Setup SPI IMU (master)
  SPI_2.setBitOrder(MSBFIRST);            // Set the SPI_1 bit order
  SPI_2.setDataMode(SPI_MODE3);           //For IMU280ZA MODE3 CPHA=1 and CPOL=1
  SPI_2.setClockDivider(SPI_CLOCK_DIV64); // Slow speed (72 / 64 = 1.125 MHz SPI speed) || IMU fclk= 2Mhz max!
  SPI_2.begin();                          //Initialize the SPI_1 port.
  pinMode(PB12, OUTPUT);
}

void setupSPI1()
{
  //Set SPI2 Rasp3 (slave)
  SPI.setModule(1);
  // Initialize and reset SPI2
  spi_init(SPI1); //necessaire? car repris dans begin slave
  //Configure GPIO for Slave Mode
  SPI.beginSlave(); //Rx ONLY
  // Configure SPI2 - SPI MODE1- MSb Frist - 8bit Data - FULL DUPLEX
  spi_slave_enable(SPI1, SPI_MODE0, SPI_FRAME_MSB & ~SPI_SW_SLAVE | SPI_DFF_8_BIT & ~SPI_CR1_BIDIMODE & ~SPI_CR1_RXONLY & ~SPI_CR1_CRCEN); // For FULL DUPLEX : BIDIMODE=0 and RXONLY=0
}

///////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  //  //----------------------SERIAL USB------------------------------
  //  //-----------------------------------------------------------
  Serial.begin(9600); //USB
  while (!Serial)
  {
  }
  Serial.println("1/5 USB OK !");
  Serial.setTimeout(10);
  //  //----------------------DYNAMIXEL AX-------------------------
  //  //-----------------------------------------------------------
  Serial4.begin(BAUD_AX);

  Dynamixel.SetDirPin(PIN_DATA_CTRL_AX); // Inicialize the servo AX at 1Mbps and Pin Control
  delay(1);
  Dynamixel.torqueStatus(ALL, 0); //Enable free movement
  delay(10);
  Dynamixel.setRDT(ALL, 1); //Set return delay to 2us
  delay(10);
  Dynamixel.setMaxTorque(ALL, 1023); // max torque = 1023
  delay(10);
  Dynamixel.setCSlope(ALL, 128, 128);
  delay(10);
  while (!Serial)
  {
  }
  Serial.println("2/5 AX OK !");
  Serial.setTimeout(10);
  //  //----------------------DYNAMIXEL XM------------------------
  //  //-----------------------------------------------------------
  Serial5.begin(BAUD_XM);

  DynamixelX.SetDirPin(PIN_DATA_CTRL_XM); //Initialization
  delay(10);
  DynamixelX.setTorque(ALL, 0); //(ID, 0 ) Enable EEPROM
  delay(10);
  DynamixelX.setRDT(ALL, 1); //Set return delay to 2us
  delay(10);
  //DynamixelX.setTorque(ALL, 1); //(ID, 1) Enable moving
  //delay(10);
  while (!Serial)
  {
  }
  Serial.println("3/5 Torque XM OK !");
  Serial.setTimeout(10);

  //-------------------------SPI-------------------------------
  //-----------------------------------------------------------
  setupSPI2(); //as master
  setupSPI1(); //as slave
  while (!Serial)
  {
  }
  Serial.println("4/5 SPI OK !");
  Serial.setTimeout(10);
  //-------------------------ODrive------------------------------
  //-----------------------------------------------------------
  Serial3.begin(BAUD_ODRIVE);
  USART3->regs->CR3 = USART_CR3_DMAT; //enable DMA on UART
  DMA_Config_Buffer();

  
////  Hardware O/I configuration
  //Output for Odrive
  pinMode(PD10, OUTPUT);
  digitalWrite(PD10, HIGH);//No reset after an STM32 reset
  // Reset button
  pinMode(PF4, INPUT);
  pinMode(PF6, OUTPUT);
  digitalWrite(PF6, LOW); //Reset
  pinMode(PF2, OUTPUT);
  digitalWrite(PF6, HIGH); //No reset

  ODTestReset();//Function to test the activation of the rest button
  delay(100);

  while (!Serial)
  {
  }
  Serial.println("5/5 Odrive OK !");
  Serial.setTimeout(10);

  //-----------------------CODEURS DIRECT READ----------------------------
  //---------------------------------------------------------------------

  setupCodeurs();
  while (!Serial)
  {
  }
  Serial.println("Codeurs OK !");
  Serial.setTimeout(10);;;;
}

void loop()
{

  //------------------Affichage pour débogage-----------------------------------------
  //-----------------------------------------------------------
  //displayVar(); /Slow down a lot the microcontroller, may cause issue in spi transfer
  //SPI_2.transferSlave(((uint8_t*)ptr_rbuffer), ((uint8_t*)ptr_rbuffer),SIZE_BUFFER);  //Sends exactly what is received

  //serialEvent4();
  //printBuffer4();
  //ptr_wbuffer->w_flag = FLAG_WOD;
  //  delay(10);
  //-------------------------ENTREES SPI-------------------------------
  //-----------------------------------------------------------------
  do
  {
    ptr_wbuffer->wspi_test = NULL;                                                        // reset received test variable
    SPI_1.transferSlave(((uint8_t *)ptr_wbuffer), ((uint8_t *)ptr_rbuffer), SIZE_BUFFER); //(RX,TX,size) Transfer to the master
    ptr_rbuffer->rspi_test = ptr_wbuffer->wspi_test;                                      // Used to test data transmited = SESAME
  } while (rbuffer.rspi_test != SESAME);                                                  //block until receiving proper data

  //-------------------------MESURE TEMPS BOUCLE-------------------------------
  //---------------------------------------------------------
  //long int top_chrono;
  //top_chrono = millis();//mesure le temps ecoule en milli secondes depuis le demarrage du programme

  //-------------------------DYNAMIXEL AX-------------------------------
  //-----------------------------------------------------------

  if (testFlag(FLAG_RAX))
  {
    //Local variables
    int dataAx;

    /*Reception*/
    int error = Dynamixel.readPosition(AX_1, &dataAx); // int error =Dynamixel. readPosition(AX_1,(int*)ptr_rbuffer->rAx1_pos);

    while (dataAx > 1023) //Restart the serial port when receiving corrupted data, 1023 is the max possible
    {
      Dynamixel.end();
      delayMicroseconds(1);
      Dynamixel.begin(BAUD_AX, PIN_DATA_CTRL_AX);
      delayMicroseconds(1);
      error = Dynamixel.readPosition(AX_1, &dataAx);
    }
    ptr_rbuffer->rAx1_pos = dataAx;

    delayMicroseconds(80); //Waiting the end of the transmission

    rbuffer.rAx2_pos = Dynamixel.readPosition(AX_2, &dataAx);

    while (dataAx > 1023) //Restart the serial port when receiving corrupted data, 1023 is the max possible
    {
      Dynamixel.end();
      delayMicroseconds(1);
      Dynamixel.begin(BAUD_AX, PIN_DATA_CTRL_AX);
      delayMicroseconds(1);
      error = Dynamixel.readPosition(AX_2, &dataAx);
    }
    ptr_rbuffer->rAx2_pos = dataAx;
  }

  if (testFlag(FLAG_WAX))
  {
    /*Fonctions pour commande simple*/
    //  Dynamixel.move(AX_1,wbuffer.wAx1_pos); //DYNAMIXEL AX Right
    //  Dynamixel.move(AX_2,wbuffer.wAx2_pos); //DYNAMIXEL AX Left
    /*Fonctions pour commande double*/
    Dynamixel.synWritePos(AX_1, wbuffer.wAx1_pos, AX_2, wbuffer.wAx2_pos);
  }

  //-------------------------DYNAMIXEL XM-------------------------------
  //----------------------------------------------------------------

  if (testFlag(FLAG_TORQUEXM))
  {
    DynamixelX.setTorque(ALL, 1); //(ID, 1) Enable moving
    delay(10);
  }

  if (testFlag(FLAG_RXM))
  {
    //Local variables
    int dataXm1, dataXm2;

    DynamixelX.syncReadPos(XM_1, &dataXm1, XM_2, &dataXm2); //Returns 16 bits for each motor
    ptr_rbuffer->rXm1_pos = dataXm1;
    ptr_rbuffer->rXm2_pos = dataXm2;
    delayMicroseconds(250); //Waiting the end of the transmission

    //delayMicroseconds(350);//Waiting the end of the transmission

    /*Fonctions de reception position double*/
    //int  error = DynamixelX.syncReadPos(XM_1,(int*)ptr_rbuffer->rXm1_pos,XM_2,(int*)ptr_rbuffer->rXm2_pos); //Returns 16 bits for each motor

    /*Fonctions de reception courant double*/
    //    uint32_t incomingCur=DynamixelX.syncReadCur(XM_1,XM_2); //Returns 16 bits for each motor
    //    rbuffer.rXmR_cur= incomingCur >> 16;
    //    rbuffer.rXmL_cur=incomingCur & 0xFFFF;
    //Serial.print(" incomingCur ");  Serial.println(incomingCur,HEX);

    /*Fonctions de reception position simple*/
    //int posx =DynamixelX. readPosition(XM_1);

    /*Fonctions ping*/
    //uint32_t in = DynamixelX.ping(1);
    //Serial.print(" in ");  Serial.println(in,HEX);
  }

  if (testFlag(FLAG_WXM))
  {
    /*Fonctions pour commande position double*/
    DynamixelX.synWritePos(XM_1, wbuffer.wXm1_pos, XM_2, wbuffer.wXm2_pos);
    /*Fonctions pour commande simple*/
    //  DynamixelX.move(XM_1,wbuffer.wXm1_pos);
    //  DynamixelX.move(XM_2,wbuffer.wXm2_pos);
  }

  //-------------------------IMU-------------------------------
  //-----------------------------------------------------------

  if (testFlag(FLAG_IMU))
  {
    ImuRead(CS_PIN); // ask,read and print IMU data register
  }

  //-------------------------ODrive------------------------------
  //-----------------------------------------------------------
  //write 2motors : 742us
  //read 2motors : 8467us
  //r/w 2motors : 9611 us -> allow to work at 100Hz
  if (testFlag(FLAG_ROD))
  {
    Serial3 << "r axis" << OD_0 << ".encoder.pos_estimate\n";
    Serial3.flush(); //not working without
    rbuffer.rOd0_pos = (uint16_t)odrive.readInt();

    Serial3 << "r axis" << OD_1 << ".encoder.pos_estimate\n";
    Serial3.flush(); //not working without
    rbuffer.rOd1_pos = (uint16_t)odrive.readInt();

    //  rbuffer.rOd0_pos = odrive.readFloat(OD_0 , .encoder.pos_estimate);
    // rbuffer.rOd1_pos = odrive.readFloat(OD_1, encoder.pos_estimate);

    /*Affichage*/
    //  Serial.print("rOdR_pos : ");    Serial.println(rbuffer.rOdR_pos);
    //  Serial.print("rOdL_pos : ");    Serial.println(rbuffer.rOdL_pos);
  }

  if (testFlag(FLAG_WOD))
  {
    //UART3
    //read 2motors : 740us
    //demarrer_chrono();

    /*Standart sending*/ //Possible interferences with SPI at frequeny >100Hz
//    odrive.SetPosition(OD_0, wbuffer.wOd0_pos); //Motor 0
//    odrive.SetPosition(OD_1, wbuffer.wOd1_pos); //Motor 1


   for (int i=0;i<=LENGHT;i++)
 {
  dma_reg_Od [i]=0;
 }

    /*DMA sending*/
//    int a =sprintf(dma_reg_Od, "p 0 %3.1f 0 0 \np 1 %3.1f 0 0 \n", (wbuffer.wOd0_pos) * 1.0f, (wbuffer.wOd1_pos) * 1.0f); // BUG : only motor1 turns 
//    dma_tube_cfg(DMA1, DMA_CH2, &tube_config);                                                                     //return the error
//    dma_enable(DMA1, DMA_CH2);                                                                                     // Enable the channel and start the transfer

    int a =sprintf(dma_reg_Od, "p 0 %3.1f 0 0 \np 1 %3.1f 0 0 \np 0 %3.1f 0 0 \n", (wbuffer.wOd0_pos) * 1.0f, (wbuffer.wOd1_pos) * 1.0f,(wbuffer.wOd0_pos) * 1.0f); // Write formatted data to string
    dma_tube_cfg(DMA1, DMA_CH2, &tube_config);                                                                     //return the error
    dma_enable(DMA1, DMA_CH2);                                                                                     // Enable the channel and start the transfer

   delay(15); //prevent Odrive freeze



// for (i=0;i<=LENGHT;i++)
// {
//  Serial.print(i); Serial.print(": ");Serial.println( dma_reg_Od [i]);
//
// }
//   delay(1);
//    Serial.println("--------------------------------");

    // stop_chrono();
    //Serial.print("Od0_pos : ");  Serial.println( wbuffer.wOd0_pos*1.0f,DEC);
    // Serial.print("Od1_pos : ");  Serial.println( wbuffer.wOd1_pos*1.0f,DEC);
    //Serial.print("return sprintf: ");  Serial.println( a);

 
  }

  //-------------------------Codeurs------------------------------
  //-----------------------------------------------------------
  /*Starting from 0 : the counter upcount BUT downcount from the setOverflow value (PPR)
    getCount() returns count between 0 and 2048 (PPR) not taking in account the direction
    Scheme with PPR=2048: 0...2044_2045_2046_2047_2048_0_1_2_3_4_5...2048
    We want a symmetrical upcount and downcount from 0 following this scheme: -1025...-5_-4_-3_-2_-1_0_1_2_3_4_5..1024
    Max upper body amplitude +-45° = +-256
  */
  if (testFlag(FLAG_CODEURS))
  {

    int count1 = Timer1.getCount(); //Read the counter register
    int count4 = Timer4.getCount();

    if (count1 > 2096) //& Timer1.getDirection() == 1) // Symmetrical count
    {
      ptr_rbuffer->rCodHip0 = -4097 + count1; // Converted to negative
    }
    else
      ptr_rbuffer->rCodHip0 = count1; // Normal count

    if (count4 > 2096) //& Timer4.getDirection() == 1)// Symmetrical count
    {
      ptr_rbuffer->rCodHip1 = -4097+ count4; // Converted to negative
    }
    else
      ptr_rbuffer->rCodHip1 = count4; // Normal count
  }

    // Return the current OD command 
  rbuffer.wOd0_pos_fb = wbuffer.wOd0_pos;
  rbuffer.wOd1_pos_fb = wbuffer.wOd1_pos;
  
  // Serial.print(" rbuffer.wOd0_pos_fb  : ");  Serial.println(rbuffer.wOd0_pos_fb );
   //Serial.print(" rbuffer.wOd1_pos_fb : ");  Serial.println(rbuffer.wOd1_pos_fb );

  ODTestReset();

  //-------------------------Affichage----------------------------------------------
  //  Serial.print(" rAx1_pos : ");  Serial.println(rbuffer.rAx1_pos);
  //  Serial.print(" rAx2_pos : ");  Serial.println(rbuffer.rAx2_pos);
  /*Affichage*/
  //  Serial.print("wXm1_pos : ");  Serial.println(wbuffer.wXm1_pos,DEC);
  //  Serial.print("rXmL_pos : ");  Serial.println(wbuffer.wXm2_pos,DEC);

  //   Serial.print("Od0_pos : ");  Serial.println( wbuffer.wOd0_pos*1.0f,DEC);
  //   Serial.print("Od1_pos : ");  Serial.println( wbuffer.wOd1_pos*1.0f,DEC);

  //  Serial.print("wXm1_pos : ");  Serial.println(wbuffer.wXm1_pos,DEC);
  //  Serial.print("rXmL_pos : ");  Serial.println(wbuffer.wXm2_pos,DEC);

  //    Serial.print("Codeur1:  "); Serial.println(rbuffer.rCodHip0);
  //    Serial.print("Codeur2:  "); Serial.println(rbuffer.rCodHip1);

  //stop_chrono() ;


  //-------------------------MESURE TEMPS BOUCLE-------------------------------
  //---------------------------------------------------------
  //long int arret_chrono = millis();
  //rbuffer.looptime = (arret_chrono - top_chrono);//retour en ms



}

////----------------------------------------------SPI IMU BRUT --------------------------------------------------
////-------------------------------------------------------------------------------------------------------------
int ImuRead(unsigned char Cs_PIN)
{
  //Lecture de X_RATE,Y_RATE,Z_RATE,X_ACCEL,Y_ACCEL,Z_ACCEL,X_MAG,Y_MAG,Z_MAG,DIAGNOSTIC_STATUS
  lowPin();
  SPI_2.transfer16(X_RATE_adress); //gyroscope
  highPin();
  lowPin();
  ptr_rbuffer->rate[0] = SPI_2.transfer16(Y_RATE_adress); // read X rate
  highPin();
  lowPin();
  ptr_rbuffer->rate[1] = SPI_2.transfer16(Z_RATE_adress); // read Y rate
  highPin();
  lowPin();
  ptr_rbuffer->rate[2] = SPI_2.transfer16(X_ACC_adress); // read Z rate
  highPin();
  lowPin();
  ptr_rbuffer->acc[0] = SPI_2.transfer16(Y_ACC_adress);
  highPin();
  lowPin();
  ptr_rbuffer->acc[1] = SPI_2.transfer16(Z_ACC_adress);
  highPin();
  lowPin();
  ptr_rbuffer->acc[2] = SPI_2.transfer16(X_MAG_adress);
  highPin();
  lowPin();
  ptr_rbuffer->mag[0] = SPI_2.transfer16(Y_MAG_adress);
  highPin();
  lowPin();
  ptr_rbuffer->mag[1] = SPI_2.transfer16(Z_MAG_adress);
  highPin();
  lowPin();
  ptr_rbuffer->mag[2] = SPI_2.transfer16(ZERO);
  highPin();
}

void DMA_Config_Buffer() // Send a buffer//TX only
{
  // Setup of general flags for USART1
  tube_config.tube_src_size = DMA_SIZE_8BITS;
  tube_config.tube_dst = &(USART3->regs->DR); // Destination of data
  tube_config.tube_dst_size = DMA_SIZE_8BITS; // Size of the data register
  tube_config.target_data = 0;
  tube_config.tube_req_src = DMA_REQ_SRC_USART3_TX; // DMA request source.
  // Setup of specifics flags depending on DMA mode whished
  tube_config.tube_src = &dma_reg_Od; // Source of the data
  tube_config.tube_nr_xfers = dma_bufer_size2;
  tube_config.tube_flags = (DMA_FROM_MEM | DMA_MINC_MODE); // Read from memory to peripheral | Auto-increment memory address

  dma_init(DMA1); // Initialization
  // ! 1st transfer !
  int error = dma_tube_cfg(DMA1, DMA_CH2, &tube_config); //Setup of the DMA
  dma_set_priority(DMA1, DMA_CH2, DMA_PRIORITY_LOW);     // by default
  dma_enable(DMA1, DMA_CH2);                             // Enable the channel and start the transfer.

  Serial.print("error:  ");                      // return 0 if no error.
  Serial.println(error);                         //return the error
  Serial.print("dma_is_enabled:  ");             //return 0 if the tube is disabled, >0 if it is enabled.
  Serial.println(dma_is_enabled(DMA1, DMA_CH2)); //return 0 if the tube is disabled, >0 if it is enabled.
}

void displayVar()
{
  //    Serial.println("\n ------Reception Master---------");
  //    Serial.print(" wAx1_pos ");Serial.println(wbuffer.wAx1_pos) ;
  //    Serial.print(" wAx2_pos ");  Serial.println(wbuffer.wAx2_pos);
  //    Serial.print(" wXm1_pos ");  Serial.println(wbuffer.wXm1_pos);
  //    Serial.print(" wXm2_pos_pos");  Serial.println(wbuffer.wXm2_pos);
  //    Serial.print(" wOd0_pos ");  Serial.println(wbuffer.wOd0_pos);
  //    Serial.print(" wOd1_pos ");  Serial.println(wbuffer.wOd1_pos);
}

int fonction_MSB(int octet)
{
  //decalage et suppression bits de poids faibles
  int msb = (octet >> 8);
  return msb;
}

int fonction_LSB(int octet)
{
  //Masque de suppression bits de poids forts
  int lsb = (octet & 0x00FF);
  return lsb;
}

unsigned int count(unsigned int i)
{
  unsigned int ret = 1;
  while (i /= 10)
    ret++;
  return ret;
}

void ODTestReset()
{
////  ODrive and encoder Reset
//The goal is to synchronize the 0 of OD and the 0 of the hips encodeurs
//The position of the OD motors before a reset is memorized by OD and becomes the new 0
if (digitalRead (PF4)==0)// No Reset required
  {
  digitalWrite(PD10, HIGH); // No reset
 }

if (digitalRead (PF4)==1) // Reset required
{
  setupCodeurs();// reset codeur
  digitalWrite(PD10, LOW); // reset OD
}
}
//------------------------------inutilisé----------------------------------------------
//---------------------------------------------------------------
//int fonction_concat (int octet_MSB, int octet_LSB ) {
//  int octet_concat = (octet_MSB << 8 | octet_LSB); //Concatenation Right
//  return octet_concat;
//}

//void serialEvent5() //Test de presence de donnees sur la voie serie
//{
//  while (Serial5.available()) {
//    Serial.println(Serial5.read(), HEX);
//  }
//}
//
//void serialEvent4() //Test de presence de donnees sur la voie serie
//{
//  while (Serial4.available()) {
//    Serial.println(Serial4.read());
//  }
//}
//
//
//void printBuffer5()
//{
//  //delay(20);
//  while (Serial5.available()) {
//    //Serial.println(Serial5.read(),HEX);
//    //delay(2);
//  }
//}
//
//
//void printBuffer4()
//{
//  //delay(20);
//  while (Serial4.available()) {
//    Serial.println(Serial4.read(), HEX);
//    //delay(2);
//  }
//}
