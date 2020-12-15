#include <Arduino.h>
#include <SPI.h>

int dacOutputA;
int dacOutputB;

const int chipSelect = 2;

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

void writeMCP4922(uint16_t dataA)
{
  uint8_t config = 0x30; // bin 1011 0000
  uint8_t topMsg = (config & 0xF0) | ((0xF00 & dataA) >> 8);
  uint8_t lowerMsg = dataA & 0xFF;

  Serial.print("topMsg: "); Serial.print(topMsg, BIN);
  Serial.print(", ");
  Serial.print("lowerMsg: "); Serial.println(lowerMsg, BIN);

  // Set CS pin low to transfer data
  digitalWrite(chipSelect, LOW);

  SPI.transfer(topMsg);
  SPI.transfer(lowerMsg);
  delay(100);

  digitalWrite(chipSelect, HIGH);
}

/*
void writeMCP4922_16(uint16_t dataA)
{
  uint16_t config = 0xFFFF; // bin 1111 1111 1111 1111
  uint16_t msg = config & dataA;

  Serial.print("Msg: "); Serial.println(msg, BIN);

  // Set CS pin low to transfer data
  digitalWrite(chipSelect, LOW);

  SPI.transfer16(msg);

  digitalWrite(chipSelect, HIGH);
}
*/

void setup() {
  Serial.begin(9600);

  // Initialize SPI communication (DAC)
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  // Set CS default high
  digitalWrite(chipSelect, HIGH);

  // Read 12 bit analog input
  analogReadResolution(12);

  // Set analog pins for DAC references
  // analogWrite(A0, 4095); // DAC A reference
  // analogWrite(A1, 4095); // DAC B reference
}

void loop() {
  // Write to DAC A
  writeDAC(4095, chipSelect);

  // Measure DAC output
  dacOutputA = analogRead(A0);
  // dacOutputB = analogRead(A1);

  Serial.print("DAC A output: "); Serial.println(dacOutputA);
  // Serial.print("DAC B output: "); Serial.println(dacOutputB);

  delay(2000);
}