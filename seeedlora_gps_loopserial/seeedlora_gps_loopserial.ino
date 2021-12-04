/** ------------------------------------------------------------------
 *  For M3
 **/

/** ------------------------------------------------------------------
 *  Important libraries  
 **/
#include <Wire.h>
#include "rgb_lcd.h"            // LCD Library
#include <CayenneLPP.h>         // Cayenne Library
#include <LoRaWan.h>            // Seeduino LoRaWAN library (also referred as "lora" below)

/** -------------------------------------------------------------------
 *  General setup for Seeeduino LoRaWAN microcontroller
 **/
// Provide power to the 4 Grove connectors of the board
#define PIN_GROVE_POWER 38

// Loop variable for anything that need a loop counting
unsigned int nloops = 0; 

/** -------------------------------------------------------------------
 *  Setup for GPS
 **/
// To define if the LoRaWAN module has GPS (1 = Yes, 0 = No)
#define USE_GPS 1

// For TinyGPS++ object
#ifdef USE_GPS
#include "TinyGPS++.h"
TinyGPSPlus gps;
#endif

// Float variables for longitude and latitude
float latitude;
float longitude;

/** -------------------------------------------------------------------
 *  Setup for LCD display
 **/
// Setup of background color of lcd display
rgb_lcd lcd;
const int colorR = 0;
const int colorG = 255;
const int colorB = 0; 

/** -------------------------------------------------------------------
 *  Setup for SWM and TTN LoRaWAN
 **/
// Hien's SWM LoRa keys 
#define SWM_H_AppEUI "70B3D57ED002F952"
#define SWM_H_DevEUI "00D2981B966FE2C6"
#define SWM_H_AppKey "D3A9861046E34B018A080CA6AEA8C751"

// Hien's TTN LoRa keys 
#define TTN_H_AppEUI "70B3D57ED00302CB"
#define TTN_H_DevEUI "00A31F100B916A7A"
#define TTN_H_AppKey "DD256AD58DA97A7673D4AADBAFDAD939"

// Bach's SWM LoRa keys 
#define SWM_B_AppEUI "70B3D57ED002F952"
#define SWM_B_DevEUI "00B193384D6E47B8"
#define SWM_B_AppKey "D9902CCE630C3FA5591FE68A451140D9"

// Bach's TTN LoRa keys 
#define TTN_B_AppEUI "70B3D57ED00302CB"
#define TTN_B_DevEUI "00B5551E089D69B4"
#define TTN_B_AppKey "AE736536AADA2BB103FBD906CFA14226"


/** -------------------------------------------------------------------
 *  Setup for Cayenne
 **/
// Cayenne buffer
CayenneLPP lpp(51);

// Buffer for text messages received from the LoRaWAN module for display
char buffer[256];   

// Boolean for LoRa package transferring
bool result = false;                  


/** ------------------------------------------------------------------
 *  setup() function
 **/
void setup() 
{
  /** --------------------------------------------------------------------------------
   *  Seeeduino LoRaWAN & Grove TCS34725 Startup Section
   **/
  // Provide power to the 4 Grove connectors of the board
  digitalWrite(PIN_GROVE_POWER, HIGH);                

  // Switching the board to PowerSaver mode
  for(int i = 0; i < 26; i ++)          // Set all pins to HIGH to save power (reduces the
  {                                     // current drawn during deep sleep by around 0.7mA).
    if (i!=13)                          // Don't switch on the onboard user LED (pin 13).
    {                    
      pinMode(i, OUTPUT);
      digitalWrite(i, HIGH);
    }
  }  

  delay(5000);                          // Wait 5 secs after reset/booting to give time for potential upload
                                        // of a new sketch (sketches cannot be uploaded when in sleep mode)

  // Initialize serial connection  
  Serial.begin(115200);
  while(!Serial) {};
  delay(500);

  Serial2.begin(9600);     // open the GPS
  while(!Serial2) {};

  // Wait for serial port to connect. Needed for native USB port only
  while (!Serial) {};

  SerialUSB.begin(115200);
  while(!SerialUSB);
  
  Serial.println();  
  Serial.println("Seeeduino LoRaWAN board started!");


  // Start LCD display connection
  lcd.begin(16, 2);  
  lcd.setRGB(colorR, colorG, colorB);
  lcd.print("SeeedLoRa Init");


  /** --------------------------------------------------------------------------------
   *  LoRaWAN & TTN Setup Section
   **/
  // Configuring LoRaWAN
  lora.init();                             // Initialize the LoRaWAN module
  
  Serial.println();
  Serial.println("Connecting to 'TUM Geoinformatik3 TTN LoRaWAN' ....... ");
  Serial.println();

  // Checking LoRaWAN version and id
  memset(buffer, 0, 256);                  // clear text buffer
  lora.getVersion(buffer, 256, 1);
  memset(buffer, 0, 256);                  // We call getVersion() two times, because after a reset the LoRaWAN module can be
  lora.getVersion(buffer, 256, 1);         // in sleep mode and then the first call only wakes it up and will not be performed.
  if (Serial) 
  {
    Serial.print(buffer);
  }
  memset(buffer, 0, 256);
  lora.getId(buffer, 256, 1);
  if (Serial) 
  {
    Serial.print(buffer);
  }
  
  // void setId(char *DevAddr, char *DevEUI, char *AppEUI);
  lora.setId(NULL, TTN_H_DevEUI, TTN_H_AppEUI);  

  // setKey(char *NwkSKey, char *AppSKey, char *AppKey);
  lora.setKey(NULL, NULL, TTN_H_AppKey);
    
  lora.setDeciveMode(LWOTAA);              // select OTAA join mode (note that setDeciveMode is not a typo; it is misspelled in the library)
  lora.setDataRate(DR5, EU868);         // SF7, 125 kbps (highest data rate)
  // lora.setDataRate(DR3, EU868);            // SF9, 125 kbps (medium data rate and range)
  // lora.setDataRate(DR0, EU868);         // SF12, 125 kbps (lowest data rate, highest max. distance)

  // lora.setAdaptiveDataRate(false);  
  lora.setAdaptiveDataRate(true);       // automatically adapt the data rate
    
  lora.setChannel(0, 868.1);
  lora.setChannel(1, 868.3);
  lora.setChannel(2, 868.5);
  lora.setChannel(3, 867.1);
  lora.setChannel(4, 867.3);
  lora.setChannel(5, 867.5);
  lora.setChannel(6, 867.7);
  lora.setChannel(7, 867.9);

  lora.setDutyCycle(false);             // for debugging purposes only - should normally be activated
  lora.setJoinDutyCycle(false);         // for debugging purposes only - should normally be activated
    
  lora.setPower(14);                    // LoRa transceiver power (14 is the maximum for the 868 MHz band)
  lora.setPort(33);                     // all data packets are sent to LoRaWAN port 33

  // Retrying until the node has successfully joined TTN
  while (!lora.setOTAAJoin(JOIN, 20)) 
  {
    nloops++;
    if (Serial) 
    {
      Serial.println((String)"Join failed, retry: " + nloops);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Try LoRaWAN ...");
      lcd.setCursor(0, 1);
      lcd.print((String)"Try #: " + nloops);
      delay(2000);
    }
  }
  
  Serial.println();
  Serial.println("Succesfully joined Geoinformatik3 TTN LoRaWAN !");
  Serial.println();                     
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Joined LoRaWAN!!");
  delay(2000);

  /** --------------------------------------------------------------------------------
   *  GPS Setup Section
   **/
  char c;
  #ifdef USE_GPS
    bool locked;
  #endif

  #ifdef USE_GPS
    locked = false;
   
    // For S&G, let's get the GPS fix now, before we start running arbitary
    // delays for the LoRa section
    
    while (!gps.location.isValid()) 
    {
      while (Serial2.available() > 0) 
      {
        if (gps.encode(c=Serial2.read())) 
        {
          if (gps.location.isValid()) 
          {        
            break;
          }
        }
      }
    }
  #endif
}


/** ------------------------------------------------------------------
 *  loop() function
 **/
void loop() 
{   
  sendGPS();
  delay(5000);
}


/** ------------------------------------------------------------------
 *  Send GPS
 *  
 **/
void sendGPS()
{
  nloops++;       
  if (Serial)
  {
    Serial.println((String)"Sending GPS data packet [#" + nloops + "] to FROST Server .... ");
    Serial.println();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sending GPS ....");
    delay(2000);
    lcd.setCursor(0, 1);
    lcd.print((String)"Packet#: " + nloops);
    delay(2000);
  }
  
  if (gps.location.isValid()) 
  {
    Serial.println(F("Current GPS Location of the device"));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Current GPS");
    delay(2000);

    latitude = gps.location.lat();
    Serial.println((String)"Latitude: " + latitude);
    longitude = gps.location.lng();
    Serial.println((String)"Longitude: " + longitude);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print((String)"Lati: " + latitude);
    delay(2000);
    lcd.setCursor(0, 1);
    lcd.print((String)"Longi: " + longitude);
    delay(2000);
  }
  else
  {
    Serial.print(F("INVALID"));
  }
                          
  lpp.reset();                                    // Resets the Cayenne buffer
  lpp.addGPS(1, latitude, longitude, 448);        // Encodes the GPS location data on channel 1 in Cayenne GPS format 

  // Send the Cayenne encoded data packet (n bytes) off with a default timeout of 5 secs
  //  result = lora.transferPacket(lpp.getBuffer(), lpp.getSize(), 10);
  result = lora.transferPacketWithConfirmed(lpp.getBuffer(), lpp.getSize(), 10);

  if(result)
  {
    short length;
    short rssi;
    
    memset(buffer, 0, 256);
    length = lora.receivePacket(buffer, 256, &rssi);
    
    if(length)
    {
      Serial.println((String)"Length is: " + length);
      Serial.println((String)"RSSI is: " + rssi);
      Serial.println((String)"Data is: ");
      for(unsigned char i = 0; i < length; i ++)
      {
          Serial.print("0x");
          Serial.print(buffer[i], HEX);
          Serial.print(" ");
      }
      Serial.println();
    }
  }

  if (Serial)
  {
    // Message for when data package got sent
    Serial.println();
    Serial.println((String)"GPS data packet [#" + nloops + "] sent succesfully!\n");
    Serial.println("-------------------------------------------------------------");
    Serial.println();
    delay(1000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("GPS Data Sent !!");
    delay(2000);
    lcd.setCursor(0, 1);
    lcd.print((String)"Packet#: " + nloops);
    delay(2000);
  }
}
