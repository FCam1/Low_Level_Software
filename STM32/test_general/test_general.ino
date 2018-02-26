#include <SPI.h>
#include <time.h>
#include "variables.h"
#include "Imu_DMU280ZA.h"


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
  pinMode(PA4, OUTPUT);//if SPI 1
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


  //DynamixelX.setBD(XM_LEFT, 3);


  
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
  //-------------------------ENTREES-------------------------------
  //-----------------------------------------------------------
  rOdR = 1234 ; //position angulaire ODrive
  rOdL = 9999 ; //position angulaire ODrive
  //-------------------------SPI-------------------------------
  //-----------------------------------------------------------
  uint8_t TxBuffer[22] = { /// TODO A DEPLACER HORS DU LOOP
    //Variables Odrive
    rOdRM,
    rOdRL,
    rOdLM,
    rOdLL,
    //Variables IMU
    rXrateM,
    rXrateL,
    rYrateM,
    rYrateL,
    rZrateM,
    rZrateL,
    rXaccM,
    rXaccL,
    rYaccM,
    rYaccL,
    rZaccM,
    rZaccL,
    rXmagM,
    rXmagL,
    rYmagM,
    rYmagL,
    rZmagM,
    rZmagL
  }; //Tx buffer

  uint8_t  RxBuffer[12] = {wOdRM, wOdRL, wOdLM, wOdLL, wAxRM, wAxRL, wAxLM, wAxLL, wXmRM, wXmRL, wXmLM, wXmLL}; //Rx buffer

  SPI_2.transferSlave(RxBuffer, TxBuffer, sizeof(TxBuffer));  //To Raspberry



  //-------------------------DYNAMIXEL-------------------------------
  //-----------------------------------------------------------
  //AX
    wAxR = fonction_concat (RxBuffer [4], RxBuffer [5]); //TODO uniformiser les formats en entree pour AX et XM: choisir avec ou sans concatenation
    wAxL = fonction_concat (RxBuffer [6], RxBuffer [7]);   

    Dynamixel.move(AX_RIGHT,wAxR); //DYNAMIXEL AX Right
   Dynamixel.move(AX_LEFT,wAxL); //DYNAMIXEL AX Right

    //XM

    wXmR = fonction_concat (RxBuffer [8], RxBuffer [9] ); 
    wXmL = fonction_concat (RxBuffer [10], RxBuffer [11] );

    //PosCtrl (XM_RIGHT,wXmR ); //DYNAMIXEL XM Right
   // PosCtrl (XM_LEFT,wXmL); //DYNAMIXEL XM Left
     
    DynamixelX.move(XM_RIGHT,wXmR ); 
    DynamixelX.move(XM_LEFT,wXmL );


  //-------------------------IMU-------------------------------
  //-----------------------------------------------------------

  IMU();            // ask,read and print IMU data register

  //-------------------------ODrive------------------------------
  //-----------------------------------------------------------
  //Concatenation des octets recus
    wOdR = fonction_concat (RxBuffer [0],RxBuffer [1] );
    wOdL = fonction_concat (RxBuffer [2],RxBuffer [3] );
   // Serial.println("\n -----------ODrive move !----------");
   // Serial.println(wOdR);
    odrive.SetPosition(OD_RIGHT ,wOdR) ; //Motor 0
    odrive.SetPosition(OD_LEFT ,wOdL); //Motor 1
    
    
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
int fonction_MSB (int octet) {
  //decalage et suppression bits de poids faibles
  int msb = (octet >> 8);
  return msb;
}

int fonction_LSB (int octet) {
  //Masque de suppression bits de poids forts
  int lsb = (octet & 0x00FF);
  return lsb;
}

int fonction_concat (int octet_MSB, int octet_LSB ) {
  int octet_concat = (octet_MSB << 8 | octet_LSB); //Concatenation Right
  return octet_concat;
}

//------------------------------------------------DYNAMIXEL XM----------------------------------------------
//-----------------------------------------------------------------------------------------------------------
//void setTorque(int ID, bool value)
//{
//  unsigned  char TxPacket[] = {0xFF, 0xFF, 0xFD, 0x00, ID, 0x06, 0x00, 0x03, 0x40, 0x00, value, 0x00, 0x00};
//  CRC = update_crc (0, TxPacket, 11); //init crc_accum | packet | 5+Packet Lenght()
//  byte CRC_L = (CRC & 0x00FF);
//  byte CRC_H = (CRC >> 8) & 0x00FF;
//
//
//  digitalWrite (PIN_DATA_CTRL_XM, HIGH); //Half Duplex command : TRANSMITION
//  delay(1);
//  Serial5.write(0xFF); //Header
//  Serial5.write(0xFF); //Header
//  Serial5.write(0xFD); //Header
//  Serial5.write(0x00); //Reserved
//  Serial5.write(ID); //ID 0x01:default ; OxFE:all connected devices
//  Serial5.write(0x06); //Packet lenght LEN_L - number of parameters +3
//  Serial5.write(0x00); //Packet lenght LEN_H
//  Serial5.write(0x03); //instruction to WRITE_DATA
//  Serial5.write(0x40); //PARAMETER 1 : RAM adress ()
//  Serial5.write(0x00); //PARAMETER 2 : RAM adress ()
//  Serial5.write(value); //PARAMETER 3
//  Serial5.write(CRC_L);
//  Serial5.write(CRC_H);
//  Serial5.flush();
//  digitalWrite(PIN_DATA_CTRL_XM, LOW); //Half Duplex command : RECEPTION
//  delay(1);
//}



//void PosCtrl (int ID, int PosLSB, int PosMSB)
//{
//  unsigned  char TxPacket[] = {0xFF, 0xFF, 0xFD, 0x00, ID, 0x09, 0x00, 0x03, 0x74, 0x00, PosLSB, PosMSB, 0x00, 0x00, 0x00, 0x00};
//  CRC = update_crc (0, TxPacket, 14); //init crc_accum ; packet ; =5+Packet Lenght()
//  byte CRC_L = (CRC & 0x00FF);
//  byte CRC_H = (CRC >> 8) & 0x00FF;
//
//
//  digitalWrite (PIN_DATA_CTRL_XM, HIGH); //Half Duplex command : TRANSMITION
//  delay(1);
//  Serial5.write(0xFF); //Header
//  Serial5.write(0xFF); //Header
//  Serial5.write(0xFD); //Header
//  Serial5.write(0x00); //Reserved
//  Serial5.write(ID); //ID 0x01:default ; OxFE:all connected devices
//  Serial5.write(0x09); //Packet lenght LEN_L - number of parameters +3
//  Serial5.write(0x00); //Packet lenght LEN_H
//  Serial5.write(0x03); //instruction to WRITE_DATA
//  Serial5.write(0x74); //PARAMETER 1 : RAM adress (116)
//  Serial5.write(0x00); //PARAMETER 2 : RAM adress
//  Serial5.write(PosLSB); //PARAMETER 3
//  Serial5.write(PosMSB); //PARAMETER 4 : Value to write
//  Serial5.write(0x00); //PARAMETER 5
//  Serial5.write(0x00); //PARAMETER 6
//  Serial5.write(CRC_L);
//  Serial5.write(CRC_H);
//  Serial5.flush();
//  digitalWrite(PIN_DATA_CTRL_XM, LOW); //Half Duplex command : RECEPTION
//  delay(1);
//}
void PosCtrl (byte ID, byte POS)
{
  byte POS_LSB = (POS & 0x00FF);
  byte POS_MSB = (POS >> 8) & 0x00FF;
  unsigned  char TxPacket[]={0xFF,0xFF,0xFD,0x00,ID,0x09,0x00,0x03,0x74,0x00,POS_LSB,POS_MSB,0x00,0x00,0x00,0x00};
  CRC= update_crc (0,TxPacket,14); //init crc_accum ; packet ; =5+Packet Lenght()
  byte CRC_L= (CRC & 0x00FF);
  byte CRC_H= (CRC >> 8) & 0x00FF;
  
  digitalWrite (PIN_DATA_CTRL_XM,HIGH); //Half Duplex command : TRANSMITION
  delay(1);
  Serial5.write(0xFF); //Header
  Serial5.write(0xFF); //Header
  Serial5.write(0xFD); //Header
  Serial5.write(0x00); //Reserved
  Serial5.write(ID); //ID 0x01:default ; OxFE:all connected devices
  Serial5.write(0x09); //Packet lenght LEN_L - number of parameters +3
  Serial5.write(0x00); //Packet lenght LEN_H
  Serial5.write(0x03); //instruction to WRITE_DATA
  Serial5.write(0x74); //PARAMETER 1 : RAM adress (116)
  Serial5.write(0x00); //PARAMETER 2 : RAM adress
  Serial5.write(POS_LSB); //PARAMETER 3
  Serial5.write(POS_MSB); //PARAMETER 4 : Value to write
  Serial5.write(0x00); //PARAMETER 5
  Serial5.write(0x00); //PARAMETER 6
  Serial5.write(CRC_L);
  Serial5.write(CRC_H);
  Serial5.flush();
  digitalWrite(PIN_DATA_CTRL_XM,LOW); //Half Duplex command : RECEPTION
  delay(4);//Very important : if <3ms only 1 motor can work simultaneously
}


//ROBOTIS CRC16 CALCULATOR
//crc_accum : set as 0
//*data_blk_ptr : packet to send
//data_blk_size : number of bytes in the packet excluding the CRC
//Return 16 bit CRC value

unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{

  unsigned short i, j;

  unsigned short crc_table[256] = {

    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,

    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,

    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,

    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,

    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,

    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,

    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,

    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,

    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,

    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,

    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,

    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,

    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,

    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,

    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,

    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,

    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,

    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,

    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,

    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,

    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,

    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,

    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,

    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,

    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,

    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,

    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,

    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,

    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,

    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,

    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,

    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202

  };
  for (j = 0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;

    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }
  return crc_accum;
}


//----------------------------------------------SPI IMU BRUT --------------------------------------------------
//-------------------------------------------------------------------------------------------------------------

void IMU() {
  //Lecture de X_RATE,Y_RATE,Z_RATE,X_ACCEL,Y_ACCEL,Z_ACCEL,X_MAG,Y_MAG,Z_MAG,DIAGNOSTIC_STATUS
  digitalWrite(PA4, LOW);
  SPI_1.transfer16(X_RATE_adress);//gyroscope
  digitalWrite(PA4, HIGH);
  digitalWrite(PA4, LOW);
  rXrate = SPI_1.transfer16(Y_RATE_adress);
  digitalWrite(PA4, HIGH);
  digitalWrite(PA4, LOW);
  rYrate = SPI_1.transfer16(Z_RATE_adress);
  digitalWrite(PA4, HIGH);
  digitalWrite(PA4, LOW);
  rZrate = SPI_1.transfer16(X_ACC_adress);
  digitalWrite(PA4, HIGH);
  digitalWrite(PA4, LOW);
  rXacc = SPI_1.transfer16(Y_ACC_adress);
  digitalWrite(PA4, HIGH);
  digitalWrite(PA4, LOW);
  rYacc = SPI_1.transfer16(Z_ACC_adress);
  digitalWrite(PA4, HIGH);
  digitalWrite(PA4, LOW);
  rZacc = SPI_1.transfer16(X_MAG_adress);
  digitalWrite(PA4, HIGH);
  digitalWrite(PA4, LOW);
  rXmag = SPI_1.transfer16(Y_MAG_adress);
  digitalWrite(PA4, HIGH);
  digitalWrite(PA4, LOW);
  rYmag = SPI_1.transfer16(Z_MAG_adress);
  digitalWrite(PA4, HIGH);
  digitalWrite(PA4, LOW);
  rZmag = SPI_1.transfer16(ZERO);
  digitalWrite(PA4, HIGH);

  //Formatage des octets avant envoie
  //RATE
  rXrateM = fonction_MSB(rXrate); //MSB
  rXrateL = fonction_LSB(rXrate); //LSB
  rYrateM = fonction_MSB(rYrate); //MSB
  rYrateL = fonction_LSB(rYrate); //LSB
  rZrateM = fonction_MSB(rZrate); //MSB
  rZrateL = fonction_LSB(rZrate); //LSB
  //ACCELERATION
  rXaccM = fonction_MSB(rXacc); //MSB
  rXaccL = fonction_LSB(rXacc); //LSB
  rYaccM = fonction_MSB(rYacc); //MSB
  rYaccL = fonction_LSB(rYacc); //LSB
  rZaccM = fonction_MSB(rZacc); //MSB
  rZaccL = fonction_LSB(rZacc); //LSB
  //MAGNITUDE
  rXmagM = fonction_MSB(rXmag); //MSB
  rXmagL = fonction_LSB(rXmag); //LSB
  rYmagM = fonction_MSB(rYmag); //MSB
  rYmagL = fonction_LSB(rYmag); //LSB
  rZmagM = fonction_MSB(rZmag); //MSB
  rZmagL = fonction_LSB(rZmag); //LSB
}

void serialEvent3() //Test de presence de donnees sur la voie serie
{
  while (Serial3.available()) {
   int8_t data = Serial3.read();
    Serial.println(data);
   }
}
