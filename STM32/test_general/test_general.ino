#include <SPI.h>
#include <time.h>
#include "variables.h"



//uint8_t rbuffer.[22];
//uint8_t  RxBuffer[12];
//structures
struct TrameWrite wbuffer;
struct TrameWrite* ptr_wbuffer;

struct TrameRead  rbuffer;
struct TrameRead* ptr_rbuffer;

//-----------------------------------------------------------
//----------------------DYNAMIXEL AX------------------------
//-----------------------------------------------------------
#include <SavageDynamixelSerial_Upgraded.h> ///ATENTION SERIAL 3 || Tx, Rx pins have to be modified in DynamixelSerial_Modified.cpp ||
//#include <Dynamixel_Protocol2.h> 

//DynamixelXClass DynamixelX ;



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

//long int top_chrono;
//void demarrer_chrono() {
//        top_chrono = micros(); //mesure le temps ecoule en micro secondes depuis le demarrage du programme
//}
//void stop_chrono() {
//        long int arret_chrono = micros();
//        float duree = ((arret_chrono - top_chrono)/ (CLOCKS_PER_SEC *10000.0));
//        Serial.println("Le calcul a pris en secondes :");
//        Serial.println(duree,6);
//        Serial.println("\n");
//}
//-----------------------------------------------------------
//-----------------------------------------------------------
void setupSPI1() {
  // Setup SPI IMU (master)
  SPI_1.begin(); //Initialize the SPI_1 port.
  SPI_1.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI_1.setDataMode(SPI_MODE3); //For IMU280ZA MODE3 CPHA=1 and CPOL=1
  SPI_1.setClockDivider(SPI_CLOCK_DIV64);      // Slow speed (72 / 64 = 1.125 MHz SPI speed) || IMU fclk= 2Mhz max!
  pinMode(CS_PIN, OUTPUT);//if SPI 1
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
  ptr_wbuffer= &wbuffer;
  ptr_rbuffer = &rbuffer;

  //----------------------DYNAMIXEL AX-------------------------
  //-----------------------------------------------------------
  //!\/Serial 3 defined - to modify it go to DynamixelSerial_modified.cpp
  Dynamixel.begin(BAUD_AX, PIN_DATA_CTRL_AX); // Inicialize the servo AX at 1Mbps and Pin Control
  delay(1);
  //----------------------DYNAMIXEL XM------------------------
  //-----------------------------------------------------------
  DynamixelX.begin(BAUD_XM,PIN_DATA_CTRL_XM);
  Serial.flush();
  //Initialization
  DynamixelX.setTorque (ALLXM,0); //(ID, 0 ) Enable EEPROM
  delay (10);//!\Very important/!
  
  DynamixelX.setTorque (ALLXM,1); //(ID, 1) Enable moving
  delay (1);
  Serial.println("setTorque XM OK");
  //-------------------------SPI-------------------------------
  //-----------------------------------------------------------
  setupSPI1();//as master
  setupSPI2();//as slave
  delay (1);
  //-------------------------ODrive------------------------------
  //-----------------------------------------------------------
  Serial3.begin (BAUD_ODRIVE);
  delay (1);
  //----------------------SERIAL USB------------------------------
  //-----------------------------------------------------------
  Serial.begin (9600); //USB
  while (!Serial) {}
  delay(400);
  Serial.println("Setup OK !");
  Serial.setTimeout(10);
}

void loop()
{
serialEvent3(); //Teste la presence de donnees sur la voie serie 
  //-------------------------ENTREES SPI-------------------------------
  //-----------------------------------------------------------
//  rOdR = 1234 ; //position angulaire ODrive
//  rOdL = 9999 ; //position angulaire ODrive
  SPI_2.transferSlave(((uint8_t*)ptr_wbuffer), ((uint8_t*)ptr_rbuffer), SIZE_BUFFER);  //To Raspberry

  //-------------------------DYNAMIXEL-------------------------------
  //-----------------------------------------------------------
  //AX 
   Dynamixel.move(AX_RIGHT,wbuffer.wAxR); //DYNAMIXEL AX Right
   Dynamixel.move(AX_LEFT,wbuffer.wAxL); //DYNAMIXEL AX Right

  //XM
    DynamixelX.move(XM_RIGHT,wbuffer.wXmR); 
    DynamixelX.move(XM_LEFT,wbuffer.wXmL);
    
  //-------------------------IMU-------------------------------
  //-----------------------------------------------------------

  ImuRead(CS_PIN);            // ask,read and print IMU data register

  //-------------------------ODrive------------------------------
  //-----------------------------------------------------------
  //Concatenation des octets recus
//    wOdR = fonction_concat (RxBuffer [0],RxBuffer [1] );
//    wOdL = fonction_concat (RxBuffer [2],RxBuffer [3] );
   // Serial.println("\n -----------ODrive move !----------");
   // Serial.println(wOdR);
   
    odrive.SetPosition(OD_RIGHT ,wbuffer.wOdR) ; //Motor 0
    odrive.SetPosition(OD_LEFT ,wbuffer.wOdL); //Motor 1
    
// rOdR = odrive.GetParameter(OD_RIGHT , odrive.PARAM_FLOAT_ENCODER_PLL_POS); 
//  rOdL = odrive.GetParameter(OD_LEFT , odrive.PARAM_FLOAT_ENCODER_PLL_POS);
//
//Serial.println(rOdR);
//
//    
//  //  Decoupage en 2 octets pour envoie au rasp
//    rOdRM =fonction_MSB(rOdR);//MSB
//    rOdRL =fonction_LSB(rOdR);//LSB
//    rOdLM =fonction_MSB(rOdL);//MSB
//    rOdLL =fonction_LSB(rOdL);//LSB

  //-----------------------------------------------------------
  //-----------------------------------------------------------
  //
  //  Serial.println("\n ------Reception Rasp3---------");
  //  Serial.print(" wAxR ");Serial.println(wAxR) ;
  //  Serial.print(" wAxL ");  Serial.println(wAxL);
  //  Serial.print(" wXmR ");  Serial.println(wXmR);
  //  Serial.print(" wXmL ");  Serial.println(wXmL);
  //  Serial.print(" wOdR ");  Serial.println(wOdR);
  //  Serial.print(" wOdL ");  Serial.println(wOdL);


  //}

  //demarrer_chrono();
  //stop_chrono();

}

//------------------------------Fonctions de concatenation/deconcatenation----------------------------------------------
//-----------------------------------------------------------------------------------------------------------
uint16_t fonction_MSB (uint8_t octet) {
  //decalage et suppression bits de poids faibles
  int msb = (octet >> 8);
  return msb;
}

uint16_t fonction_LSB (uint8_t  octet) {
  //Masque de suppression bits de poids forts
  int lsb = (octet & 0x00FF);
  return lsb;
}

uint16_t fonction_concat (int octet_MSB, int octet_LSB ) {
  int octet_concat = (octet_MSB << 8 | octet_LSB); //Concatenation Right
  return octet_concat;
}


////----------------------------------------------SPI IMU BRUT --------------------------------------------------
////-------------------------------------------------------------------------------------------------------------
//
int ImuRead(unsigned char Cs_PIN) {
  //Lecture de X_RATE,Y_RATE,Z_RATE,X_ACCEL,Y_ACCEL,Z_ACCEL,X_MAG,Y_MAG,Z_MAG,DIAGNOSTIC_STATUS


  
  digitalWrite(Cs_PIN, LOW);
  SPI_1.transfer16(X_RATE_adress);//gyroscope
  digitalWrite(Cs_PIN, HIGH);
  digitalWrite(Cs_PIN, LOW);
  rbuffer.rXrate = SPI_1.transfer16(Y_RATE_adress); 
  digitalWrite(Cs_PIN, HIGH);
  digitalWrite(Cs_PIN, LOW);
  rbuffer.rYrate = SPI_1.transfer16(Z_RATE_adress);
  digitalWrite(Cs_PIN, HIGH);
  digitalWrite(Cs_PIN, LOW);
  rbuffer.rZrate = SPI_1.transfer16(X_ACC_adress);
  digitalWrite(Cs_PIN, HIGH);
  digitalWrite(Cs_PIN, LOW);
  rbuffer.rXacc = SPI_1.transfer16(Y_ACC_adress);
  digitalWrite(Cs_PIN, HIGH);
  digitalWrite(Cs_PIN, LOW);
  rbuffer.rYacc = SPI_1.transfer16(Z_ACC_adress);
  digitalWrite(Cs_PIN, HIGH);
  digitalWrite(Cs_PIN, LOW);
  rbuffer.rZacc = SPI_1.transfer16(X_MAG_adress);
  digitalWrite(Cs_PIN, HIGH);
  digitalWrite(Cs_PIN, LOW);
  rbuffer.rXmag = SPI_1.transfer16(Y_MAG_adress);
  digitalWrite(Cs_PIN, HIGH);
  digitalWrite(Cs_PIN, LOW);
  rbuffer.rYmag = SPI_1.transfer16(Z_MAG_adress);
  digitalWrite(Cs_PIN, HIGH);
  digitalWrite(Cs_PIN, LOW);
  rbuffer.rZmag = SPI_1.transfer16(ZERO);
  digitalWrite(Cs_PIN, HIGH);

  //Formatage des octets avant envoie
  //RATE
//  rXrateM = fonction_MSB(imu.rXrate); //MSB
//  rXrateL = fonction_LSB(imu.rXrate); //LSB
//  rYrateM = fonction_MSB(imu.rYrate); //MSB
//  rYrateL = fonction_LSB(imu.rYrate); //LSB
//  rZrateM = fonction_MSB(imu.rZrate); //MSB
//  rZrateL = fonction_LSB(imu.rZrate); //LSB
//  //ACCELERATION
//  rXaccM = fonction_MSB(imu.rXacc); //MSB
//  rXaccL = fonction_LSB(imu.rXacc); //LSB
//  rYaccM = fonction_MSB(imu.rYacc); //MSB
//  rYaccL = fonction_LSB(imu.rYacc); //LSB
//  rZaccM = fonction_MSB(imu.rZacc); //MSB
//  rZaccL = fonction_LSB(imu.rZacc); //LSB
//  //MAGNITUDE
//  rXmagM = fonction_MSB(imu.rXmag); //MSB
//  rXmagL = fonction_LSB(imu.rXmag); //LSB
//  rYmagM = fonction_MSB(imu.rYmag); //MSB
//  rYmagL = fonction_LSB(imu.rYmag); //LSB
//  rZmagM = fonction_MSB(imu.rZmag); //MSB
//  rZmagL = fonction_LSB(imu.rZmag); //LSB
}

void serialEvent3() //Test de presence de donnees sur la voie serie
{
  while (Serial3.available()) {
   int8_t data = Serial3.read();
    Serial.println(data);
   }
}
