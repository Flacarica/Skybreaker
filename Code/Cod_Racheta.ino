#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>


const int MPU_ADDR = 0x68;    //  Adresa default pentru MPU6050, se poate transforma in 0x69 daca pinul AD0 e legat la 3.3v

TinyGPSPlus gps;    //  Initializam obiect gps
HardwareSerial SerialGPS(2);    //  Initializam comunicare pe serial 2
double GPS_Lat, GPS_Long;

// --- HM-TRLR-TTL-433 / config-mode pins ---
const int LORA_SLEEP_PIN = 25;    // LoRa SLEEP -> ESP32 GPIO
const int LORA_RESET_PIN = 26;    // LoRa RESET -> ESP32 GPIO
const int LORA_CONFIG_PIN = 27;   // LoRa CONFIG (enter config when LOW)

// LoRa serial (UART1)
HardwareSerial SerialLoRa(1);
const int LORA_RX_PIN = 4;  // LoRa TX -> ESP32 RX
const int LORA_TX_PIN = 5;  // LoRa RX <- ESP32 TX
const uint32_t LORA_BAUD = 9600;

// LoRa configuration commands to send in config mode
const String loraConfigCommands[] = {
  "AT",            // basic handshake
  "AT+SPR=3",      // set UART speed (SPR=3 -> 9600 bps)
  "AT+POWER=0",    // set TX power to 20 dBm (module encoding: 0 -> 20dBm)
  "AT+SYNL=6",     // 6-byte syncword length
  "AT+NODE=0,0",   // disable node ID function
  "AT+LRCRC=1",    // LoRa with CRC enabled
  "AT+LRSBW=7",    // SBW code 7 -> 125 kHz
  "AT+LRSF=9",     // spreading factor 9
  "AT+LRCR=0",     // coding rate 4/5 (module encoding 0)
  "AT+LRHF=0",     // FHSS disabled
  "AT+LRPL=32",    // packet length 32 bytes
  "AT+LRHPV=10",   // hopping period 10
  "AT+LRFSV=1638", // frequency step value for 100 kHz step (module-specific encoding)
  "AT+MODE=0",     // LoRa mode
  "AT+BAND=0"      // 433 MHz band (module encoding: 0)
};

const size_t loraConfigCommandsCount = sizeof(loraConfigCommands) / sizeof(loraConfigCommands[0]);    // number of commands


// Offsets pentru calibrare
long AcX_off = 0, AcY_off = 0, AcZ_off = 0;
long GyX_off = 0, GyY_off = 0, GyZ_off = 0;

// Latest converted sensor values (filled by readMPU)
float AcX_g_val = 0, AcY_g_val = 0, AcZ_g_val = 0;
float GyX_dps_val = 0, GyY_dps_val = 0, GyZ_dps_val = 0;

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

  // store latest
  AcX_g_val = AcX_g;
  AcY_g_val = AcY_g;
  AcZ_g_val = AcZ_g;

  // Conversie pentru dps
  float GyX_dps = GyX / 131.0;
  float GyY_dps = GyY / 131.0;
  float GyZ_dps = GyZ / 131.0;

  GyX_dps_val = GyX_dps;
  GyY_dps_val = GyY_dps;
  GyZ_dps_val = GyZ_dps;

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

// Helper: toggle pins to enter config mode
void enterLoRaConfigMode() {
  pinMode(LORA_SLEEP_PIN, OUTPUT);
  pinMode(LORA_RESET_PIN, OUTPUT);
  pinMode(LORA_CONFIG_PIN, OUTPUT);

  // 1) Pull sleep LOW
  digitalWrite(LORA_SLEEP_PIN, LOW);
  // 2) Pull reset HIGH
  digitalWrite(LORA_RESET_PIN, HIGH);
  delay(2);
  // 3) Pull config LOW for at least 5 ms
  digitalWrite(LORA_CONFIG_PIN, LOW);
  delay(8); // keep slightly longer than 5ms
  // Keep pins as-is while sending AT commands
}

void exitLoRaConfigMode() {
  // Release config pin (set HIGH) and allow module normal operation
  digitalWrite(LORA_CONFIG_PIN, HIGH);
  // Optionally put sleep pin HIGH to allow normal operation
  digitalWrite(LORA_SLEEP_PIN, HIGH);
}

// Send an AT command and wait for expected substring. Returns true if found.
bool sendATCommand(const String &cmd, const String &expect, unsigned long timeout = 500) {
  while (SerialLoRa.available()) SerialLoRa.read();
  Serial.print("-> LoRa AT: "); Serial.println(cmd);
  SerialLoRa.println(cmd);
  unsigned long start = millis();
  String resp = "";
  while (millis() - start < timeout) {
    while (SerialLoRa.available()) {
      char c = SerialLoRa.read();
      resp += c;
    }
    if (expect.length() > 0 && resp.indexOf(expect) >= 0) {
      Serial.print("<- LoRa resp: "); Serial.println(resp);
      return true;
    }
    delay(10);
  }
  Serial.print("<- LoRa resp timeout (got): "); Serial.println(resp);
  return false;
}

// Initialize LoRa UART and, if requested, enter config-mode and run AT commands.
void initLoRa() {
  Serial.print("Init LoRa UART on RX="); Serial.print(LORA_RX_PIN);
  Serial.print(" TX="); Serial.print(LORA_TX_PIN);
  Serial.print(" @"); Serial.println(LORA_BAUD);

  SerialLoRa.begin(LORA_BAUD, SERIAL_8N1, LORA_RX_PIN, LORA_TX_PIN);
  delay(200);

  // Enter config mode by toggling SLEEP/RESET/CONFIG pins as documented
  enterLoRaConfigMode();
  delay(50); // give the module a moment

  // Run configured AT command list (best-effort).
  Serial.println("LoRa: sending automatic config sequence (best-effort).\n");
  for (size_t i = 0; i < loraConfigCommandsCount; ++i) {
    String cmd = loraConfigCommands[i];
    sendATCommand(cmd, "OK", 1000);
    delay(100);
  }

  // Leave config mode to normal operation
  exitLoRaConfigMode();
  Serial.println("LoRa: config sequence completed (see logs for responses).\n");
  Serial.println("If commands did not match your module's firmware, enter interactive mode:\n  - Open Serial Monitor at 115200\n  - Type 'c' then press Enter to enter LoRa interactive config shell\n  - Type 'q' then Enter to exit the shell\");
}

// Interactive forwarder: read from USB Serial and forward lines to the LoRa UART
// while printing responses back. Exit when user sends a line with just 'q'.

void interactiveLoRaShell() {
  Serial.println("-- Entering LoRa interactive config shell. Type 'q' to quit --");
  while (true) {
    // Forward any user input to LoRa
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;
      if (line == "q") {
        Serial.println("-- Exiting interactive shell --");
        break;
      }
      // Ensure module is in config mode while user configures
      enterLoRaConfigMode();
      SerialLoRa.println(line);
      Serial.print("-> "); Serial.println(line);
      unsigned long start = millis();
      String resp = "";
      while (millis() - start < 1000) {
        while (SerialLoRa.available()) {
          char c = SerialLoRa.read();
          resp += c;
        }
        if (resp.length()) break;
      }
      Serial.print("<- "); Serial.println(resp);
      exitLoRaConfigMode();
    }
    // Also print any unsolicited responses from LoRa
    if (SerialLoRa.available()) {
      String r = "";
      while (SerialLoRa.available()) r += (char)SerialLoRa.read();
      Serial.print("<- "); Serial.println(r);
    }
    delay(10);
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  // Initialize LoRa UART and (optionally) run AT configuration sequence
  initLoRa();
  //  Configuram si calibram MPU
  configMPU();
  calibrateMPU();
  Serial.println("MPU OK! Values set for +/- 16g and 2000dps.");

  SerialGPS.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Serial.println("GPS OK! Communicating on pins RX17 and TX16.");

  delay(1500); 
}

void loop() {
  // Check for user request to enter interactive LoRa config shell
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "c") {
      interactiveLoRaShell();
    }
  }

  readMPU();    //  Citim acc si gyr
  readGPS();    //  Citim lat si lng

  // Build telemetry payload (compact JSON-like)
  String payload = "{";
  payload += "lat:" + String(GPS_Lat, 6) + ",";
  payload += "lon:" + String(GPS_Long, 6) + ",";
  payload += "ax:" + String(AcX_g_val, 3) + ",ay:" + String(AcY_g_val, 3) + ",az:" + String(AcZ_g_val, 3) + ",";
  payload += "gx:" + String(GyX_dps_val, 2) + ",gy:" + String(GyY_dps_val, 2) + ",gz:" + String(GyZ_dps_val, 2);
  payload += "}";

  // Send over LoRa UART (transparent write)
  SerialLoRa.println(payload);
  Serial.print("LoRa payload: "); Serial.println(payload);

  delay(50);
}
