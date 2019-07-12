#include <Arduino.h>

#include <Wire.h>
#include <SdFat.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// #include <TimeLib.h>

#undef DEBUG

// pino A4 -> SDA do MPU
// pino A5 -> SCL do MPU
// endereco I2C do MPU6050
const int MPU = 0x68;

// Pino 4 -> TX do Gps
// Pino 3 -> RX do Gps
static const int RXPin = 4, TXPin = 5;
static const uint32_t GPSBaud = 9600;

//Variaveis para armazenar valores do GPS e MPU
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// Pino 8 -> CS do SD
// Pino 11 -> MOSI do SD
// Pino 12 -> MISO do SD
// Pino 13 -> SCK do SD
SdFat sdCard;
// SdFile dadosAcc;
// SdFile dadosGPS;
SdFile dataFile;

// pino placa pedro
// const uint8_t chipSelect = 10;
// pino placa jua labs
const uint8_t chipSelect = 8;
const uint16_t period = 60000; // 60000 -> 1 minute
const uint8_t periodMPU = 100;
const uint8_t num_accel_samples = 10;

unsigned long time_now = 0;

struct acc_samples_t {
  // uint32_t ts;
  int16_t AcX;
  int16_t AcY;
  int16_t AcZ;
  int16_t Tmp;
  int16_t GyX;
  int16_t GyY;
  int16_t GyZ;
} acc_samples[num_accel_samples];

uint8_t cur_sample = 0;
// time_t start_ts = 0;
char start_time[20];
char filename[30];

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void lerGPS() {
  if (gps.location.isUpdated() && gps.location.isValid()) {
    sprintf(filename, "%s-gps.txt", start_time);
    if (!dataFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      digitalWrite(9, HIGH);
      sdCard.errorHalt(F("\nerro na abertura do arquivo do gps!"));
    }
    // setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
    //        gps.date.day(), gps.date.month(), gps.date.year());
    #ifdef DEBUG
    Serial.print(F("\ntem localizacao..."));
    Serial.print(F("ts = ")); Serial.print(millis()); // Raw time in HHMMSSCC format (u32)
    Serial.print(F(" | lat = ")); Serial.print(gps.location.lat(), 6);
    Serial.print(F(" | lng = ")); Serial.println(gps.location.lng(), 6);
    #endif
    // escreve dados do GPS no arquivo
    dataFile.print(millis());
    dataFile.print(";"); dataFile.print(gps.location.lat(), 6);
    dataFile.print(";"); dataFile.println(gps.location.lng(), 6);
    dataFile.close();
  }
  digitalWrite(9, HIGH);
  smartDelay(500);
  digitalWrite(9, LOW);
}

void writeAccData() {
  sprintf(filename, "%s-acc.txt", start_time);
  if (!dataFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
    digitalWrite(9, HIGH);
    sdCard.errorHalt(F("Erro na abertura do arquivo do acelerometro!"));
  }
  #ifdef DEBUG
  Serial.print(F("writing accelerometer data..."));
  #endif
  // escreve todas as amostras coletadas
  for (uint8_t i = 0; i < cur_sample; i++) {
    // dadosAcc.print(acc_samples[i].ts);
    // escreve valor 'x' do acelerometro no arquivo
    dataFile.print(F(";")); dataFile.print(acc_samples[i].AcX);
    // escreve valor 'y' do acelerometro no arquivo
    dataFile.print(F(";")); dataFile.print(acc_samples[i].AcY);
    // escreve valor 'z' do acelerometro no arquivo
    dataFile.print(F(";")); dataFile.print(acc_samples[i].AcZ);
    // escreve valor da temperatura no arquivo
    dataFile.print(F(";")); dataFile.print(acc_samples[i].Tmp / 340.00 + 36.53);
    // escreve valor 'x' do giroscopio no arquivo
    dataFile.print(F(";")); dataFile.print(acc_samples[i].GyX);
    // escreve valor 'y' do giroscopio no arquivo
    dataFile.print(F(";")); dataFile.print(acc_samples[i].GyY);
    // escreve valor 'z' do giroscopio no arquivo
    dataFile.print(F(";")); dataFile.println(acc_samples[i].GyZ);
  }
  #ifdef DEBUG
  Serial.println(F("ok"));
  #endif
  // fecha o arquivo
  dataFile.close();
}

void lerMPU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  // solicita os dados do sensor
  Wire.requestFrom(MPU, 14, true);

  //Armazena o valor dos sensores nas variaveis correspondentes
  acc_samples[cur_sample].AcX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acc_samples[cur_sample].AcY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acc_samples[cur_sample].AcZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  acc_samples[cur_sample].Tmp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  acc_samples[cur_sample].GyX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  acc_samples[cur_sample].GyY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  acc_samples[cur_sample].GyZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  // acc_samples[cur_sample].ts = now();
  #ifdef DEBUG
  // Serial.print(F("millis = ")); Serial.print(acc_samples[cur_sample].ts);
  // envia valor 'x' do acelerometro para a serial
  Serial.print(F(" | acc_x = ")); Serial.print(acc_samples[cur_sample].AcX);
  // envia valor 'y' do acelerometro para a serial
  Serial.print(F(" | acc_y = ")); Serial.print(acc_samples[cur_sample].AcY);
  // envia valor 'z' do acelerometro para a serial
  Serial.print(F(" | acc_z = ")); Serial.print(acc_samples[cur_sample].AcZ);
  // envia valor da temperatura para a serial
  Serial.print(F(" | tmp = ")); Serial.print(acc_samples[cur_sample].Tmp / 340.00 + 36.53);
  // envia valor 'x' do giroscopio para a serial
  Serial.print(F(" | gyr_x = ")); Serial.print(acc_samples[cur_sample].GyX);
  // envia valor 'y' do giroscopio para a serial
  Serial.print(F(" | gyr_y = ")); Serial.print(acc_samples[cur_sample].GyY);
  // envia valor 'z' do giroscopio para a serial
  Serial.print(F(" | gyr_z = ")); Serial.println(acc_samples[cur_sample].GyZ);
  #endif
  cur_sample++;
  if (cur_sample == 10) {
    writeAccData();
    // reinicializa o buffer temporario
    cur_sample = 0;
  }
}


void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif
  pinMode(9, OUTPUT);
  ss.begin(GPSBaud);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);

  if (!sdCard.begin(chipSelect, SPI_HALF_SPEED)) {
    digitalWrite(9, HIGH);
    sdCard.initErrorHalt();
  }
  // inicializa o MPU-6050
  Wire.write(0);
  Wire.endTransmission(true);
  // aguarda que a hora seja atualiazada pelo GPS
  #ifdef DEBUG
  Serial.print(F("\nwaiting gps fix: "));
  #endif
  do {
    digitalWrite(9, HIGH);
    smartDelay(250);
    digitalWrite(9, LOW);
    smartDelay(250);
  } while (!gps.location.isValid() || !gps.location.isUpdated() || 
           !gps.time.isValid() || !gps.time.isUpdated() ||
           !gps.date.isValid() || !gps.date.isUpdated());
  #ifdef DEBUG
  Serial.println(F("ok"));
  #endif
  // captura a hora do GPS
  // setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
  //        gps.date.day(), gps.date.month(), gps.date.year());
  // start_ts = now();
  sprintf(start_time, "%04d%02d%02d%02d%02d%02d",
                      gps.date.year(), gps.date.month(), gps.date.day(),
                      gps.time.hour(), gps.time.minute(), gps.time.second());
  Serial.println(start_time);
  digitalWrite(9, LOW);
}

void loop() {
  // verifica se passou o tempo entre leituras do GPS e escreve no arquivo
  if (millis() > time_now + period) {
    time_now = millis();
    lerGPS();
  }
  smartDelay(periodMPU);
  lerMPU();
}