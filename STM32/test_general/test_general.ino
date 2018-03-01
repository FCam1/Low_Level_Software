#include <SPI.h>
#include <time.h>
#include "variables.h"

//macro
#define lowPin()     (digitalWrite(Cs_PIN, LOW))
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

  //----------------------SERIAL USB------------------------------
  //-----------------------------------------------------------
  Serial.begin (9600); //USB
  while (!Serial) {}delay(400);Serial.println("1/5 USB OK !"); Serial.setTimeout(10);
  //----------------------DYNAMIXEL AX-------------------------
  //-----------------------------------------------------------
  //!\/Serial 3 defined - to modify it go to DynamixelSerial_modified.cpp
  Dynamixel.begin(BAUD_AX, PIN_DATA_CTRL_AX); // Inicialize the servo AX at 1Mbps and Pin Control
  delay(1);
  Dynamixel.setRDT (ALL ,1); //Set return delay to 2us
  
  while (!Serial) {} delay(400);Serial.println("2/5 AX OK !");Serial.setTimeout(10);
  //----------------------DYNAMIXEL XM------------------------
  //-----------------------------------------------------------
  DynamixelX.begin(BAUD_XM,PIN_DATA_CTRL_XM);

  //Initialization
  DynamixelX.setTorque (ALL,0); //(ID, 0 ) Enable EEPROM
  delay(10);
  DynamixelX.setRDT(ALL ,1); //Set return delay to 2us
  delay(10);
  DynamixelX.setTorque (ALL,10); //(ID, 1) Enable moving
 delay(10);
  while (!Serial) {}delay(400);Serial.println("3/5 Torque XM OK !");Serial.setTimeout(10);
 
  //-------------------------SPI-------------------------------
  //-----------------------------------------------------------
  setupSPI1();//as master
  setupSPI2();//as slave

   while (!Serial) {}delay(400);Serial.println("4/5 SPI OK !"); Serial.setTimeout(10);
  //-------------------------ODrive------------------------------
  //-----------------------------------------------------------
  Serial3.begin (BAUD_ODRIVE);

   while (!Serial) {}delay(400);Serial.println("5/5 Odrive OK !"); Serial.setTimeout(10);

}

void loop()
{

  //-------------------------ENTREES SPI-------------------------------
  //-----------------------------------------------------------
//  rOdR = 1234 ; //position angulaire ODrive
//  rOdL = 9999 ; //position angulaire ODrive

//while(1)
//{
    SPI_2.transferSlave(((uint8_t*)ptr_wbuffer), ((uint8_t*)ptr_rbuffer), SIZE_BUFFER);  //To Raspberry


  
    //-------------------------DYNAMIXEL-------------------------------
    //-----------------------------------------------------------
    //AX 
//    Serial.print(wbuffer.wOdR);   Serial.print(" ");
//    Serial.print(wbuffer.wOdL); Serial.print(" ");
//    Serial.print("RECU    ");  Serial.print(wbuffer.wAxR);Serial.println(" ");
//    Serial.print(wbuffer.wAxL);Serial.print(" ");
//    Serial.print(wbuffer.wXmR);Serial.print(" ");
//    Serial.print(wbuffer.wXmL);Serial.print("\r\n");
  
    Dynamixel.move(AX_RIGHT,wbuffer.wAxR); //DYNAMIXEL AX Right
    Dynamixel.move(AX_LEFT,wbuffer.wAxL); //DYNAMIXEL AX Left
    
    //int data=Dynamixel.move(AX_RIGHT,wbuffer.wAxR); //DYNAMIXEL AX Right
    //Serial.print(data);
   // Dynamixel.move(AX_LEFT,wbuffer.wAxL); //DYNAMIXEL AX Right
  
    //XM
  DynamixelX.move(XM_RIGHT,wbuffer.wXmR); 
  DynamixelX.move(XM_LEFT,wbuffer.wXmL);
    
//      int dete = DynamixelX.move(XM_RIGHT,wbuffer.wXmR); 
  //    Serial.println(dete,DEC);
      
//      char diti =Serial5.read();
//       Serial.print("direct read :");
//       Serial.println(diti,HEX);
 
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
     
     // odrive.SetPosition(OD_RIGHT ,wbuffer.wOdR) ; //Motor 0
     // odrive.SetPosition(OD_LEFT ,wbuffer.wOdL); //Motor 1
      
   //rOdR = odrive.GetParameter(OD_RIGHT , odrive.PARAM_FLOAT_ENCODER_PLL_POS); 
  // rOdL = odrive.GetParameter(OD_LEFT , odrive.PARAM_FLOAT_ENCODER_PLL_POS);
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
//}
}



////----------------------------------------------SPI IMU BRUT --------------------------------------------------
////-------------------------------------------------------------------------------------------------------------
int ImuRead(unsigned char Cs_PIN) {
  //Lecture de X_RATE,Y_RATE,Z_RATE,X_ACCEL,Y_ACCEL,Z_ACCEL,X_MAG,Y_MAG,Z_MAG,DIAGNOSTIC_STATUS
  
  lowPin();
  SPI_1.transfer16(X_RATE_adress);//gyroscope
   highPin();  
   lowPin();
  rbuffer.rXrate = SPI_1.transfer16(Y_RATE_adress); 
   highPin() ; 
   lowPin();
  rbuffer.rYrate = SPI_1.transfer16(Z_RATE_adress);
   highPin()  ;
   lowPin();
  rbuffer.rZrate = SPI_1.transfer16(X_ACC_adress);
   highPin()  ;
   lowPin();
  rbuffer.rXacc = SPI_1.transfer16(Y_ACC_adress);
   highPin()  ;
   lowPin();
  rbuffer.rYacc = SPI_1.transfer16(Z_ACC_adress);
   highPin()  ;
   lowPin();
  rbuffer.rZacc = SPI_1.transfer16(X_MAG_adress);
   highPin()  ;
   lowPin();
  rbuffer.rXmag = SPI_1.transfer16(Y_MAG_adress);
   highPin()  ;
   lowPin();
  rbuffer.rYmag = SPI_1.transfer16(Z_MAG_adress);
   highPin()  ;
   lowPin();
  rbuffer.rZmag = SPI_1.transfer16(ZERO);
   highPin()  ;
}


