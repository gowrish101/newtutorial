#include <SPI.h>

int CS1 = 10;

#define POWER_CTL     0x2D
#define DATA_FORMAT   0x31
#define INT_ENABLE    0x2E
#define INT_MAP       0x2F
#define INT_SOURCE    0x30
#define BW_RATE       0x2C
#define DATAX0        0x32 //X-Axis Data 0
#define DATAX1        0x33 //X-Axis Data 1
#define DATAY0        0x34 //Y-Axis Data 0
#define DATAY1        0x35 //Y-Axis Data 1
#define DATAZ0        0x36 //Z-Axis Data 0
#define DATAZ1        0x37 //Z-Axis Data 1
#define INTERRUPT_PIN 5

unsigned int data;
byte values[10];
int x, y, z;
double pitch, roll, Xg, Yg, Zg;
const float alpha = 0.5;
double fXg = 0;
double fYg = 0;
double fZg = 0;

void setup()
{
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  Serial.begin(9600);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);

  writeRegister(DATA_FORMAT, 0x00, CS1); //Set range to +/-2g
  writeRegister(BW_RATE, 0x09, CS1);  //Set Output rate to 50Hz
  writeRegister(INT_MAP, 0x7F, CS1);  //Map the interrrupt to INT1
  writeRegister(INT_ENABLE, 0x80, CS1);  //Enable DATA_READY interrrupt
  writeRegister(POWER_CTL, 0x08, CS1); //Measurement mode
  delay(50);
}

void loop()
{
  data = readRegister1(INT_SOURCE, 1, CS1);
  if (data & 0b10000000)
  {
    readRegister(DATAX0, 6, values, CS1);
    x = ((int)values[1] << 8) | (int)values[0];
    y = ((int)values[3] << 8) | (int)values[2];
    z = ((int)values[5] << 8) | (int)values[4];
    Xg = x / 256.0;
    Yg = y / 256.0;
    Zg = z / 256.0;

    fXg = Xg * alpha + (fXg * (1.0 - alpha));
    fYg = Yg * alpha + (fYg * (1.0 - alpha));
    fZg = Zg * alpha + (fZg * (1.0 - alpha));

    //Roll & Pitch Equations
    roll  = (atan2(-fYg, fZg) * 180.0) / M_PI;
    pitch = (atan2(fXg, sqrt(fYg * fYg + fZg * fZg)) * 180.0) / M_PI;

    Serial.print(roll);Serial.print(",");Serial.println(pitch);
    //Serial.print(x * 0.004); Serial.print(","); Serial.print(y * 0.004); Serial.print(","); Serial.println(z * 0.004);
  }
}

void writeRegister(char registerAddress, char value, int CS)
{
  digitalWrite(CS, LOW);
  SPI.transfer(registerAddress);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

void readRegister(char registerAddress, int numBytes, byte * values, int CS)
{
  char address = 0x80 | registerAddress;
  if (numBytes > 1)address = address | 0x40;

  digitalWrite(CS, LOW);
  SPI.transfer(address);
  for (int i = 0; i < numBytes; i++)
  {
    values[i] = SPI.transfer(0x00);
  }
  digitalWrite(CS, HIGH);
}

unsigned int readRegister1(char registerAddress, int numBytes, int CS)
{
  unsigned int values;
  char address = 0x80 | registerAddress;

  digitalWrite(CS, LOW);
  SPI.transfer(address);
  values = SPI.transfer(0x0000);
  digitalWrite(CS, HIGH);
  return values;
}
