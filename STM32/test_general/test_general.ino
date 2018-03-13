#include <SPI.h>
#include <time.h>
#include "variables.h"


//macro
#define lowPin()       (digitalWrite(Cs_PIN, LOW))
#define highPin()     (digitalWrite(Cs_PIN, HIGH))

//structures
struct TrameWrite wbuffer;
struct TrameWrite* ptr_wbuffer;

struct TrameRead  rbuffer;
struct TrameRead* ptr_rbuffer;

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
SPIClass SPI_1(1);//Create an instance of the SPI Class called SPI_1 that uses the SPI Port 1
SPIClass SPI_2(2);//Create an instance of the SPI Class called SPI_2 that uses the SPI Port 2

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
void demarrer_chrono() {
  top_chrono = micros(); //mesure le temps ecoule en micro secondes depuis le demarrage du programme
}
void stop_chrono() {
  long int arret_chrono = micros();
  float duree = ((arret_chrono - top_chrono) / (CLOCKS_PER_SEC * 10000.0));
  Serial.print("Le calcul a pris en secondes :"); Serial.println(duree, 6);
  Serial.print("Soit en ms :"); Serial.println(duree * 1000);
  Serial.print("Soit en us :"); Serial.println(duree * 1000000);
}
//-----------------------------------------------------------
//-----------------------------------------------------------
void setupSPI1() {
  // Setup SPI IMU (master)
  //Dans : STM32F1/cores/maple/libmaple/spi_f1.c décommenter  //gpio_set_mode(nss_dev, nss_bit, GPIO_AF_OUTPUT_PP); pour activer le NSS harware management
  SPI_1.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI_1.setDataMode(SPI_MODE3); //For IMU280ZA MODE3 CPHA=1 and CPOL=1
  SPI_1.setClockDivider(SPI_CLOCK_DIV64);      // Slow speed (72 / 64 = 1.125 MHz SPI speed) || IMU fclk= 2Mhz max!
  SPI_1.begin(); //Initialize the SPI_1 port.
  spi_peripheral_disable(SPI1); // Configuration NSS harware management SSOE=1, SSM=0, SSI=0
  bb_peri_set_bit(&SPI1->regs->CR2, SPI_CR2_SSOE_BIT, 1);
  bb_peri_set_bit(&SPI1->regs->CR1, SPI_CR1_SSM_BIT,0);
  bb_peri_set_bit(&SPI1->regs->CR1, SPI_CR1_SSI_BIT,0);
  spi_peripheral_enable(SPI1); 
}

void setupSPI2() {
  //Set SPI2 Rasp3 (slave)
  SPI.setModule(2);
  // Initialize and reset SPI2
  spi_init(SPI2); //necessaire? car repris dans begin slave
  //Configure GPIO for Slave Mode
  SPI.beginSlave();//Rx ONLY
  // Configure SPI2 - SPI MODE1- MSb Frist - 8bit Data - FULL DUPLEX
  spi_slave_enable(SPI2, SPI_MODE0, SPI_FRAME_MSB & ~SPI_SW_SLAVE | SPI_DFF_8_BIT & ~SPI_CR1_BIDIMODE & ~SPI_CR1_RXONLY & ~SPI_CR1_CRCEN); // For FULL DUPLEX : BIDIMODE=0 and RXONLY=0
}


void setup() {

  //Pointeurs Structures
  ptr_wbuffer = &wbuffer;
  ptr_rbuffer = &rbuffer;

  //----------------------SERIAL USB------------------------------
  //-----------------------------------------------------------
  Serial.begin (9600); //USB
  while (!Serial) {} Serial.println("1/5 USB OK !"); Serial.setTimeout(10);
  //----------------------DYNAMIXEL AX-------------------------
  //-----------------------------------------------------------
  //!\/Serial 3 defined - to modify it go to DynamixelSerial_modified.cpp
  Dynamixel.begin(BAUD_AX, PIN_DATA_CTRL_AX); // Inicialize the servo AX at 1Mbps and Pin Control
  delay(1);
  Dynamixel.setRDT (ALL , 1); //Set return delay to 2us
  delay(10);
  while (!Serial) {} Serial.println("2/5 AX OK !"); Serial.setTimeout(10);
  //----------------------DYNAMIXEL XM------------------------
  //-----------------------------------------------------------
  DynamixelX.begin(BAUD_XM, PIN_DATA_CTRL_XM);

  //Initialization
  DynamixelX.setTorque (ALL, 0); //(ID, 0 ) Enable EEPROM
  delay(10);
  DynamixelX.setRDT(ALL, 1); //Set return delay to 2us
  delay(10);
  DynamixelX.setTorque (ALL, 1); //(ID, 1) Enable moving
  delay(10);
  while (!Serial) {} Serial.println("3/5 Torque XM OK !"); Serial.setTimeout(10);

  //-------------------------SPI-------------------------------
  //-----------------------------------------------------------
  setupSPI1();//as master
  setupSPI2();//as slave

  while (!Serial) {} Serial.println("4/5 SPI OK !"); Serial.setTimeout(10);
  //-------------------------ODrive------------------------------
  //-----------------------------------------------------------
  Serial3.begin (BAUD_ODRIVE);

  while (!Serial) {} Serial.println("5/5 Odrive OK !"); Serial.setTimeout(10);

}

void loop()
{
  //------------------Affichage pour débogage-----------------------------------------
  //-----------------------------------------------------------
  //
  //  Serial.println("\n ------Reception Rasp3---------");
  //  Serial.print(" wAxR ");Serial.println(wbuffer.wAxR) ;
  //  Serial.print(" wAxL ");  Serial.println(wbuffer.wAxL);
  //  Serial.print(" wXmR ");  Serial.println(wbuffer.wXmR);
  //  Serial.print(" wXmL ");  Serial.println(wbuffer.wXmL);
  //  Serial.print(" wOdR ");  Serial.println(wbuffer.wOdR);
  //  Serial.print(" wOdL ");  Serial.println(wbuffer.wOdL);

  //serialEvent4();
  //printBuffer4();
  //-------------------------ENTREES SPI-------------------------------
  //-----------------------------------------------------------

  SPI_2.transferSlave(((uint8_t*)ptr_wbuffer), ((uint8_t*)ptr_rbuffer), SIZE_BUFFER);  //To Raspberry
  //demarrer_chrono() ;

  //-------------------------DYNAMIXEL-------------------------------
  //-----------------------------------------------------------
  //AX
  // Dynamixel.move(AX_RIGHT,wbuffer.wAxR); //DYNAMIXEL AX Right
  // Dynamixel.move(AX_LEFT,wbuffer.wAxL); //DYNAMIXEL AX Left
    Dynamixel.synWritePos(AX_RIGHT,wbuffer.wAxR,AX_LEFT, wbuffer.wAxL);
  //  rbuffer.rAxR =Dynamixel. readPosition(AX_RIGHT);
  //  rbuffer.rAxL =Dynamixel. readPosition(AX_LEFT);
  //

  //Serial.print(" pos ");  Serial.println(rbuffer.rAxL);
  //printBuffer4();
  //XM
  //DynamixelX.move(XM_RIGHT,wbuffer.wXmR);
  //DynamixelX.move(XM_LEFT,wbuffer.wXmL);

    DynamixelX.synWritePos(XM_RIGHT,wbuffer.wXmR,XM_LEFT, wbuffer.wXmL);
  
    uint32_t incoming=DynamixelX.syncReadPos(XM_RIGHT,XM_LEFT); //Returns 16 bits for each motor
    rbuffer.rXmR= incoming >> 16;
    rbuffer.rXmL=incoming & 0xFFFF;
  //Serial.print(" incoming ");  Serial.println(incoming,HEX);

  //uint32_t in = DynamixelX.ping(1);
  //Serial.print(" in ");  Serial.println(in,HEX);

  //Serial.print(" r");  Serial.println(rbuffer.rXmR,DEC);
  //Serial.print(" l ");  Serial.println(rbuffer.rXmL,DEC);

  //int posx =DynamixelX. readPosition(XM_RIGHT);
  //Serial.print(" posx ");  Serial.println(posx);

  // int dete = DynamixelX.move(XM_LEFT,55);
  //printBuffer5();
  //delay (1);
  //Serial.println(dete,DEC);

  //      char diti =Serial5.read();
  //       Serial.print("direct read :");
  //       Serial.println(diti,HEX);

  //-------------------------IMU-------------------------------
  //-----------------------------------------------------------
 // demarrer_chrono() ;
  ImuRead(CS_PIN);            // ask,read and print IMU data register
  //stop_chrono() ;
  //-------------------------ODrive------------------------------
  //-----------------------------------------------------------

  //  odrive.SetPosition(OD_RIGHT ,wbuffer.wOdR) ; //Motor 0
  //  odrive.SetPosition(OD_LEFT ,wbuffer.wOdL); //Motor 1
  //
  //  rbuffer.rOdR = odrive.GetParameter(OD_RIGHT , odrive.PARAM_FLOAT_ENCODER_PLL_POS);
  //  rbuffer.rOdL = odrive.GetParameter(OD_LEFT , odrive.PARAM_FLOAT_ENCODER_PLL_POS);
  //
  //  Serial.println(rbuffer.rOdR);
  //  Serial.println(rbuffer.rOdL);

}



////----------------------------------------------SPI IMU BRUT --------------------------------------------------
////-------------------------------------------------------------------------------------------------------------
int ImuRead(unsigned char Cs_PIN) {
  //Lecture de X_RATE,Y_RATE,Z_RATE,X_ACCEL,Y_ACCEL,Z_ACCEL,X_MAG,Y_MAG,Z_MAG,DIAGNOSTIC_STATUS
 // Valable uniquement en NSS harware management (gain de 10us / au NSS sw management) :
    SPI_1.transfer16(X_RATE_adress);//gyroscope
    rbuffer.rXrate = SPI_1.transfer16(Y_RATE_adress);
    rbuffer.rYrate = SPI_1.transfer16(Z_RATE_adress);
    rbuffer.rZrate = SPI_1.transfer16(X_ACC_adress);
    rbuffer.rXacc = SPI_1.transfer16(Y_ACC_adress);
    rbuffer.rYacc = SPI_1.transfer16(Z_ACC_adress);
    rbuffer.rZacc = SPI_1.transfer16(X_MAG_adress);
    rbuffer.rXmag = SPI_1.transfer16(Y_MAG_adress);
    rbuffer.rYmag = SPI_1.transfer16(Z_MAG_adress);
    rbuffer.rZmag = SPI_1.transfer16(ZERO);
    
// Valable uniquement en NSS sofware management :
//      lowPin();
//     SPI_1.transfer16(X_RATE_adress);//gyroscope
//     highPin();
//     lowPin();
//     rbuffer.rXrate = SPI_1.transfer16(Y_RATE_adress);
//     highPin() ;
//     lowPin();
//    rbuffer.rYrate = SPI_1.transfer16(Z_RATE_adress);
//     highPin()  ;
//     lowPin();
//    rbuffer.rYrate = SPI_1.transfer16(Z_RATE_adress);
//     rbuffer.rZrate = SPI_1.transfer16(X_ACC_adress);
//     highPin()  ;
//      lowPin();
//     rbuffer.rXacc = SPI_1.transfer16(Y_ACC_adress);
//     highPin()  ;
//     lowPin();
//    rbuffer.rYacc = SPI_1.transfer16(Z_ACC_adress);
//     highPin()  ;
//     lowPin();
//    rbuffer.rYacc = SPI_1.transfer16(Z_ACC_adress);
//     rbuffer.rZacc = SPI_1.transfer16(X_MAG_adress);
//     highPin()  ;
//     lowPin();
//     rbuffer.rXmag = SPI_1.transfer16(Y_MAG_adress);
//     highPin()  ;
//     lowPin();
//     rbuffer.rYmag = SPI_1.transfer16(Z_MAG_adress);
//     highPin()  ;
//     lowPin();
//    rbuffer.rZmag = SPI_1.transfer16(ZERO);
//     highPin()  ;
//    rbuffer.rZmag = SPI_1.transfer16(ZERO);
}

//------------------------------Fonctions de concatenation/deconcatenation----------------------------------------------
//-----------------------------------------------------------------------------------------------------------
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



void serialEvent5() //Test de presence de donnees sur la voie serie
{
  while (Serial5.available()) {
    Serial.println(Serial5.read(), HEX);
  }
}

void serialEvent4() //Test de presence de donnees sur la voie serie
{
  while (Serial4.available()) {
    Serial.println(Serial4.read());
  }
}


void printBuffer5()
{
  //delay(20);
  while (Serial5.available()) {
    //Serial.println(Serial5.read(),HEX);
    //delay(2);
  }
}


void printBuffer4()
{
  //delay(20);
  while (Serial4.available()) {
    Serial.println(Serial4.read(), HEX);
    //delay(2);
  }
}

