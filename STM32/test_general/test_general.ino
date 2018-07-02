#include <SPI.h>
#include <time.h>
#include "codeurs.h"

#ifndef codeurs.h
#include "variables.h"
#endif

//macro
#define lowPin() (gpio_write_bit(GPIOB, 12, 0))  //PB12 LOW
#define highPin() (gpio_write_bit(GPIOB, 12, 1)) //PB12 HIGH
/*methodes digital write :
  digitalWrite(PA15, LOW); lent
  gpio_write_bit(GPIOA,15,0); rapide
  GPIOA->regs->BRR=(1<<15); rapide
*/

//structures
struct TrameWrite *ptr_wbuffer = &wbuffer;
struct TrameRead *ptr_rbuffer = &rbuffer;

//-----------------------------------------------------------
//----------------------DYNAMIXEL AX------------------------
//-----------------------------------------------------------
#include <SavageDynamixelSerial_Upgraded.h> ///ATENTION SERIAL 3 || Tx, Rx pins have to be modified in DynamixelSerial_Modified.cpp ||

//-----------------------------------------------------------
//----------------------DYNAMIXEL XM------------------------
//-----------------------------------------------------------
unsigned short CRC;
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
//-----------------------------------------------------------
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

  //  //Pointeurs Structures
  //  ptr_wbuffer = &wbuffer;
  //  ptr_rbuffer = &rbuffer;

  //----------------------SERIAL USB------------------------------
  //-----------------------------------------------------------
  Serial.begin(9600); //USB
  while (!Serial)
  {
  }
  Serial.println("1/5 USB OK !");
  Serial.setTimeout(10);
  //----------------------DYNAMIXEL AX-------------------------
  //-----------------------------------------------------------
  //!\/Serial 3 defined - to modify it go to DynamixelSerial_modified.cpp
  Dynamixel.begin(BAUD_AX, PIN_DATA_CTRL_AX); // Inicialize the servo AX at 1Mbps and Pin Control
  delay(1);
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
  //----------------------DYNAMIXEL XM------------------------
  //-----------------------------------------------------------
  DynamixelX.begin(BAUD_XM, PIN_DATA_CTRL_XM);

  //Initialization
  DynamixelX.setTorque(ALL, 0); //(ID, 0 ) Enable EEPROM
  delay(10);
  DynamixelX.setRDT(ALL, 1); //Set return delay to 2us
  delay(10);
  DynamixelX.setTorque(ALL, 1); //(ID, 1) Enable moving
  delay(10);
  while (!Serial)
  {
  }
  Serial.println("3/5 Torque XM OK !");
  Serial.setTimeout(10);

  //-------------------------SPI-------------------------------
  //-----------------------------------------------------------
  setupSPI2();                   //as master
  setupSPI1();                   //as slave
  ptr_rbuffer->rspi_test = NULL; // reset SPI test variable

  while (!Serial)
  {
  }
  Serial.println("4/5 SPI OK !");
  Serial.setTimeout(10);
  //-------------------------ODrive------------------------------
  //-----------------------------------------------------------
  Serial3.begin(BAUD_ODRIVE);

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
  Serial.setTimeout(10);
}

void loop()
{

  //------------------Affichage pour débogage-----------------------------------------
  //-----------------------------------------------------------
  //displayVar(); /Slow down a lot the microcontroller, may cause issue in spi transfer
  //SPI_2.transferSlave(((uint8_t*)ptr_rbuffer), ((uint8_t*)ptr_rbuffer),SIZE_BUFFER);  //Sends exactly what is received

  //serialEvent4();
  //printBuffer4();
  //-------------------------ENTREES SPI-------------------------------
  //-----------------------------------------------------------------

  SPI_1.transferSlave(((uint8_t *)ptr_wbuffer), ((uint8_t *)ptr_rbuffer), SIZE_BUFFER); //(RX,TX,size) Transfer to the master
  ptr_rbuffer->rspi_test = ptr_wbuffer->wspi_test;
  //demarrer_chrono() ;

  //-------------------------DYNAMIXEL AX-------------------------------
  //-----------------------------------------------------------
  /*Fonctions pour commande double*/
  //Dynamixel.synWritePos(AX_1, wbuffer.wAx1_pos, AX_2, wbuffer.wAx2_pos);
  /*Fonctions pour commande simple*/
  // Dynamixel.move(AX_1,wbuffer.wAx1_pos); //DYNAMIXEL AX Right
  // Dynamixel.move(AX_2,wbuffer.wAx2_pos); //DYNAMIXEL AX Left
  /*Reception*/
  //rbuffer.rAx1_pos =Dynamixel. readPosition(AX_1);
  // rbuffer.rAx2_pos =Dynamixel. readPosition(AX_2);
  rbuffer.rAx1_pos = 265;
  rbuffer.rAx2_pos = 444;
  /*Affichage*/
  //  Serial.print(" rAx1_pos : ");  Serial.println(rbuffer.rAx1_pos);
  //  Serial.print(" rAx2_pos : ");  Serial.println(rbuffer.rAx2_pos);

  //-------------------------DYNAMIXEL XM-------------------------------
  //----------------------------------------------------------------
  /*Fonctions pour commande position double*/
  //DynamixelX.synWritePos(XM_1, wbuffer.wXm1_pos, XM_2, wbuffer.wXm2_pos);
  /*Fonctions pour commande simple*/
  //  DynamixelX.move(XM_1,wbuffer.wXm1_pos);
  //  DynamixelX.move(XM_2,wbuffer.wXm2_pos);

  /*Fonctions de reception position double*/
  //    uint32_t incomingPos=DynamixelX.syncReadPos(XM_1,XM_2); //Returns 16 bits for each motor
  //    rbuffer.rXmR_pos= incomingPos >> 16;
  //    rbuffer.rXmL_pos=incomingPos & 0xFFFF;
  //Serial.print(" incomingPos ");  Serial.println(incomingPos,HEX);

  /*Fonctions de reception courant double*/
  //    uint32_t incomingCur=DynamixelX.syncReadCur(XM_1,XM_2); //Returns 16 bits for each motor
  //    rbuffer.rXmR_cur= incomingCur >> 16;
  //    rbuffer.rXmL_cur=incomingCur & 0xFFFF;
  //Serial.print(" incomingCur ");  Serial.println(incomingCur,HEX);

  /*Fonctions ping*/
  //uint32_t in = DynamixelX.ping(1);
  //Serial.print(" in ");  Serial.println(in,HEX);

  /*Affichage*/
  //  Serial.print("wXm1_pos : ");  Serial.println(wbuffer.wXm1_pos,DEC);
  //  Serial.print("rXmL_pos : ");  Serial.println(wbuffer.wXm2_pos,DEC);

  //  Serial.print("rXmR_pos : ");  Serial.println(rbuffer.rXmR_pos,DEC);
  //  Serial.print("rXmL_pos : ");  Serial.println(rbuffer.rXmL_pos,DEC);

  //int posx =DynamixelX. readPosition(XM_1);
  //Serial.print(" posx ");  Serial.println(posx);

  //-------------------------IMU-------------------------------
  //-----------------------------------------------------------
  //demarrer_chrono() ;
  ImuRead(CS_PIN); // ask,read and print IMU data register
  // stop_chrono() ;
  //-------------------------ODrive------------------------------
  //-----------------------------------------------------------

  // odrive.SetPosition(OD_RIGHT ,wbuffer.wOd0_pos) ; //Motor 0
  //  odrive.SetPosition(OD_LEFT ,wbuffer.wOd1_pos); //Motor 1
  //
  //  rbuffer.rOdR = odrive.GetParameter(OD_RIGHT , odrive.PARAM_FLOAT_ENCODER_PLL_POS);
  //  rbuffer.rOdL = odrive.GetParameter(OD_LEFT , odrive.PARAM_FLOAT_ENCODER_PLL_POS);

  /*Affichage*/
  //  Serial.print("rOdR_pos : ");    Serial.println(rbuffer.rOdR_pos);
  //  Serial.print("rOdL_pos : ");    Serial.println(rbuffer.rOdL_pos);

  //stop_chrono() ;

  //-------------------------Codeurs------------------------------
  //-----------------------------------------------------------

  //  readCodeurs (); //envoie la valeur des 4 codeurs

  /*Affichage*/
  //  Serial.print("Codeur1:  "); Serial.println(rbuffer.rCodRMot);
  //  Serial.print("Codeur2:  "); Serial.println(rbuffer.rCodRHip);

  //delay(10);
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

int testinit()
{

  //  uint32_t MnumberAx = 0;
  //  uint32_t MnumberXM = 0;
  //
  //    uint32_t a=  Dynamixel.ping(AX_1);
  //    uint32_t b=Dynamixel.ping(AX_2);
  //    uint32_t c=DynamixelX.ping(XM_1);
  //    uint32_t d= DynamixelX.ping(XM_2);
  //
  //    if (a != MnumberAx || b != MnumberAx || c != MnumberXm || d != MnumberXm)
  //
  //    else
  //
  //
  //
}

//------------------------------inutilisé----------------------------------------------
//---------------------------------------------------------------
//int fonction_MSB (int octet) {
//  //decalage et suppression bits de poids faibles
//  int msb = (octet >> 8);
//  return msb;
//}
//
//int fonction_LSB (int octet) {
//  //Masque de suppression bits de poids forts
//  int lsb = (octet & 0x00FF);
//  return lsb;
//}
//
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
