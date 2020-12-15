#include <Arduino.h>
#include <SPI.h>

float dacOutputA;
float dacOutputB;

const byte chipSelect = 9;
const byte LDAC = 10;

void writeMCP4922(uint8_t dac, uint16_t data)
{
  if (!dac) data |= 0x7000;
  else data |= 0xF000;

  Serial.print("DATA: ");
  Serial.println(data, HEX);
  
  SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

  // Set CS pin low to transfer data
  digitalWrite(chipSelect, LOW);
  SPI.transfer16(data);
  digitalWrite(chipSelect, HIGH);

  SPI.endTransaction();
}

void setup() {
  // Set CS default high
  pinMode(chipSelect, OUTPUT);       // Set SPI CS pin as output
  digitalWrite(chipSelect, HIGH);    // Initialize CS pin in default state

  pinMode(LDAC, OUTPUT);
  digitalWrite(LDAC, HIGH);

  analogReadResolution(12);

  // Output 2 V on internal DAC
  analogWriteResolution(10);
  analogWrite(A0, 621);

  // Initialize SPI communication (DAC)
  SPI.begin();

  Serial.begin(9600);
  // Wait for serial connection
  while (!Serial) ;
  Serial.println("Outputing 1 V and 2 V");
  Serial.println();
}

void loop() {
  writeMCP4922(0, 4095); // Write to DAC A
  writeMCP4922(1, 2048); // Write to DAC B

  digitalWrite(LDAC, LOW);
  delay(10);
  digitalWrite(LDAC, HIGH);

  // Measure DAC output
  dacOutputA = (float)analogRead(A1) * (3.3/4096.0);
  dacOutputB = (float)analogRead(A2) * (3.3/4096.0);

  Serial.print("DAC A output: "); Serial.println(dacOutputA);
  Serial.print("DAC B output: "); Serial.println(dacOutputB);
  Serial.println();

  delay(2000);
}
