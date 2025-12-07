#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

const int MPU_ADDR = 0x68;    //  Adresa default pentru MPU6050, se poate transforma in 0x69 daca pinul AD0 e legat la 3.3v

TinyGPSPlus gps;    //  Initializam obiect gps
HardwareSerial SerialGPS(2);    //  Initializam comunicare pe serial 2
long GPS_Lat, GPS_Long;

// Offsets pentru calibrare
long AcX_off = 0, AcY_off = 0, AcZ_off = 0;
long GyX_off = 0, GyY_off = 0, GyZ_off = 0;

// Pini GPS si baud rate
static const int RXPin = 17;  // GPS TX → ESP32 RX
static const int TXPin = 16;  // GPS RX → ESP32 TX
static const uint32_t GPSBaud = 115200;   //  Trebuie testat daca e okay cu rate-u asta


void configMPU(){
  //  Incepem comunicarea si trimitem un cod de reset
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);                     //  Adresa cu registru de reset
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);     //  Stabilim comunicarea cu MPU6050 la adresa de mai sus
  Wire.write(0x1C);                     //  Selectam adresa 0x1C, responsabila cu registrele pentru config Acc
  Wire.write(0x18);                     //  Trimitem 0x18 - 0001 1000 pentru a seta plaja de valori +/- 16g
  Wire.endTransmission(true);           //  Incheiem transmisia pentru a putea comunica la urmatoarea adresa

  }

void calibrateMPU() {
  Serial.println("Calibrating MPU6050... Keep it still.");
  int samples = 500;
  //  Declaram variabile
  long AcX_sum = 0, AcY_sum = 0, AcZ_sum = 0;
  long GyX_sum = 0, GyY_sum = 0, GyZ_sum = 0;
  //  Iteram si masuram
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);   //  La de la 0x3B incep registrele cu datele de la accelerometru, temp, giroscop. Au format byte High si byte Low, deci total 14 bytes
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);   //  se poate 

    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    Wire.read();Wire.read(); // purge temp value
    int16_t GyX = Wire.read() << 8 | Wire.read();
    int16_t GyY = Wire.read() << 8 | Wire.read();
    int16_t GyZ = Wire.read() << 8 | Wire.read();
    //  Facem suma tuturor valorilor
    AcX_sum += AcX;
    AcY_sum += AcY;
    AcZ_sum += (AcZ - 2048);  // remove 1g from Z
    GyX_sum += GyX;
    GyY_sum += GyY;
    GyZ_sum += GyZ;

    delay(5);
  }
  //  Facem media
  AcX_off = AcX_sum / samples;
  AcY_off = AcY_sum / samples;
  AcZ_off = AcZ_sum / samples;
  GyX_off = GyX_sum / samples;
  GyY_off = GyY_sum / samples;
  GyZ_off = GyZ_sum / samples;

  Serial.println("Calibration complete.");
}

void readMPU(){
  //  Se incepe transmisia
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  //  Se citesc valorile
  int16_t AcX = Wire.read() << 8 | Wire.read();
  int16_t AcY = Wire.read() << 8 | Wire.read();
  int16_t AcZ = Wire.read() << 8 | Wire.read();
  Wire.read();Wire.read();
  int16_t GyX = Wire.read() << 8 | Wire.read();
  int16_t GyY = Wire.read() << 8 | Wire.read();
  int16_t GyZ = Wire.read() << 8 | Wire.read();

  // Se aplica valoarea de offset
  AcX -= AcX_off;
  AcY -= AcY_off;
  AcZ -= AcZ_off;
  GyX -= GyX_off;
  GyY -= GyY_off;
  GyZ -= GyZ_off;

  // Conversie pentru scala +/- 16g 
  float AcX_g = AcX / 2048;
  float AcY_g = AcY / 2048;
  float AcZ_g = AcZ / 2048;

  // Conversie pentru dps
  float GyX_dps = GyX / 131.0;
  float GyY_dps = GyY / 131.0;
  float GyZ_dps = GyZ / 131.0;

  // Print rezultate
  Serial.print("Acc (g): X=");
  Serial.print(AcX_g, 3);
  Serial.print(" Y=");
  Serial.print(AcY_g, 3);
  Serial.print(" Z=");
  Serial.print(AcZ_g, 3);

  Serial.print(" | Gyro (°/s): X=");
  Serial.print(GyX_dps, 2);
  Serial.print(" Y=");
  Serial.print(GyY_dps, 2);
  Serial.print(" Z=");
  Serial.println(GyZ_dps, 2);
}

void readGPS(){
  //  Citim date de la GPS
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
    Serial.write(c);
  }
  //  Citim locatia, daca GPS nu e conectat dam skip
  if (gps.location.isValid()) {
    Serial.print("Latitude : ");
    GPS_Lat=gps.location.lat();   //  Citim latitudinea
    Serial.println(GPS_Lat);

    Serial.print("Longitude: ");
    GPS_Long=gps.location.lng();    //  Citim longitudinea
    Serial.println(GPS_Long);
  } else {
    Serial.println("Location: INVALID (no GPS fix yet)");   //  Printam si iesim din if
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  //  Configuram si calibram MPU
  configMPU();
  calibrateMPU();
  Serial.println("MPU OK! Values set for +/- 16g and 2000dps.");

  SerialGPS.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Serial.println("GPS OK! Communicating on pins RX17 and TX16.");

  delay(1500); 
}

void loop() {
  readMPU();    //  Citim acc si gyr
  readGPS();    //  Citim lat si lng
  delay(50);
}
