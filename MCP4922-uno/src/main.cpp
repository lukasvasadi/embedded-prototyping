#include <Arduino.h>
#include <SPI.h>

const int chipSelect = 10;
const int LDAC = 8; // LDAC pin

float dacOutputA;
float dacOutputB;

void writeDAC(uint16_t data, uint8_t chipSelectPin)
{
  // Take top 4 bits of config and top 4 valid bits (data is actually a 12 bit number) and OR them together
  uint8_t config = 0x30;
  uint8_t topMsg = (config & 0xF0) | (0x0F & (data >> 8));

  uint8_t lowerMsg = (data & 0x00FF); // Take the bottom octet of data

  digitalWrite(chipSelectPin, LOW); // Select DAC, active LOW

  SPI.transfer(topMsg);   // Send first 8 bits
  SPI.transfer(lowerMsg); // Send second 8 bits

  digitalWrite(chipSelectPin, HIGH); // Deselect DAC
}

void writeMCP4922(uint16_t data)
{
  // uint8_t config = 0x30; // bin 0011 0000
  // uint8_t topMsg = (config & 0xF0) | ((0xF00 & data) >> 8);
  // uint8_t lowerMsg = data & 0xFF;

  int msg = 0b0111111111111111;
  // int msg2 = 0b1111111111111111;

  // Serial.print("topMsg: "); Serial.print(topMsg, BIN);
  // Serial.print(", ");
  Serial.print("Msg: "); Serial.println(msg, BIN);

  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

  // Set CS pin low to transfer data
  digitalWrite(SS, LOW);
  SPI.transfer16(msg);
  digitalWrite(SS, HIGH);

  SPI.endTransaction();

  // digitalWrite(chipSelect, LOW);
  // SPI.transfer(highByte(msg2));
  // SPI.transfer(lowByte(msg2));
  // digitalWrite(chipSelect, HIGH);

  // digitalWrite(LDAC, LOW);
  // delay(10);
  // digitalWrite(LDAC, HIGH);
}

void setup() {
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);

  // Configure LDAC pin
  pinMode(LDAC, OUTPUT);
  digitalWrite(LDAC, HIGH);

  Serial.begin(115200);

  // Initialize SPI communication (DAC)
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
}

void loop() {
  // Write to DAC A
  // writeMCP4922(512);
  writeDAC(4095, chipSelect);

  // Measure DAC output
  // dacOutputA = (float)analogRead(A0) * (5.0 / 1024.0);
  int dacOutput = analogRead(A0);
  // dacOutputB = analogRead(A1);

  Serial.print("DAC A output: "); Serial.println(dacOutput);
  // Serial.print("DAC B output: "); Serial.println(dacOutputB);

  delay(2000);
}