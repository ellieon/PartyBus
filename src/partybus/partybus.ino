#include <MPU6050.h>
#include <Wire.h>
#include "SoftwareSerial.h"

# define PARTY_LENGTH 26500
# define FLASH_INTERVAL 221
# define RED_PIN 8
# define GREEN_PIN 7
# define BLUE_PIN 6
# define SERIAL_WAIT 100
# define ANALOG_MAX 255
# define MOTION_DETECTION_THRESHOLD 1
# define MOTION_DETECTION_DURATION 100

boolean isPlaying = false;
long lastMillis = 0;
long lastFlash = 0;
bool ledOn = false;

MPU6050 mpu;
SoftwareSerial mySerial(10, 11);

void setup () {
  initialiseLED();
  initialiseDFPlayer();
  setupIMU();
  setRGB(0,0,0);
}

void loop () { 
  long currentMillis = millis();
  checkMotion(currentMillis);
  updateLED(currentMillis);
  
  if(currentMillis - lastMillis > PARTY_LENGTH && isPlaying){
      pauseSong();
  }
}

void checkMotion(long &currentMillis) {
  Activites act = mpu.readActivites();
  if(act.isActivity){
    lastMillis = currentMillis;
    lastFlash = currentMillis;
    if(!isPlaying) {
       startSong();
    }
  }
}

void updateLED(long &currentMillis) {
  if(isPlaying){
    if(currentMillis - lastFlash > FLASH_INTERVAL){
      if(!ledOn)
        enableLED();      
      else
        disableLED();
      lastFlash = currentMillis;
    }
  } else {
     disableLED();
  }
}

void enableLED(){
  setRGB(ANALOG_MAX,0,ANALOG_MAX);
  ledOn = true;
}

void disableLED() {
  setRGB(0,0,0);
  ledOn = false;
}

void startSong()
{
  execute_CMD(0x11,0,1); 
  isPlaying = true;
}

void pauseSong()
{
  execute_CMD(0x0E,0,0);
  isPlaying = false;
}

void resumeSong()
{
  execute_CMD(0x0D,0,1); 
  //delay(SERIAL_WAIT);
  isPlaying = true;
}

void setVolume(int volume)
{
  execute_CMD(0x06, 0, volume); // Set the volume (0x00~0x30)
  delay(SERIAL_WAIT);
}

void setRGB(int r, int g, int b)
{
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}

# define Start_Byte 0x7E
# define Version_Byte 0xFF
# define Command_Length 0x06
# define End_Byte 0xEF
# define Acknowledge 0x00 //Returns info with command 0x41 [0x01: info, 0x00: no info]
void execute_CMD(byte CMD, byte Par1, byte Par2)
// Excecute the command and parameters
{
  // Calculate the checksum (2 bytes)
  word checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + Par1 + Par2);
  // Build the command line
  byte Command_line[10] = { Start_Byte, Version_Byte, Command_Length, CMD, Acknowledge,
  Par1, Par2, highByte(checksum), lowByte(checksum), End_Byte};
  //Send the command line to the module
  for (byte k=0; k<10; k++)
  {
    mySerial.write( Command_line[k]);
  }
}

//--------------------------------
//Initialisation functions
//--------------------------------
void setupIMU() {
  Serial.begin(115200);
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);
  mpu.setIntFreeFallEnabled(false);  
  mpu.setIntZeroMotionEnabled(false);
  mpu.setIntMotionEnabled(false);
  mpu.setDHPFMode(MPU6050_DHPF_5HZ);
  mpu.setMotionDetectionThreshold(MOTION_DETECTION_THRESHOLD);
  mpu.setMotionDetectionDuration(MOTION_DETECTION_DURATION);
}

void initialiseLED() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  setRGB(ANALOG_MAX, 0, 0); 
}

void initialiseDFPlayer() {
  mySerial.begin (9600);
  execute_CMD(0x3F, 0, 0);
  delay(SERIAL_WAIT);
  setVolume(20);
}
