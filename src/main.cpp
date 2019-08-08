#include <Arduino.h>
// SD card required lib
#include <SdFat.h>
// GPS required libs
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// MPU-6050 required libs
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
// other libs
#include <jled.h>
#include <TimeLib.h>

#define DEBUG

// MPU-6050 constants
static const uint8_t INTERRUPT_PIN = 18;

// MPU-6050 related variables
MPU6050 mpu;

// GPS constants | GPS TX - PIN 4 | GPS RX - PIN 5
// changed gps tx pin from 4 to 10 due to mega2560 limitations for rx signal 
static const uint8_t GPS_RX_PIN = 10, GPS_TX_PIN = 5;
static const uint32_t GPSBaud = 9600;
// GPS related variables
TinyGPSPlus gps;
SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);

// SD card constants
// define chip select pin | 10 - pedro board | 8 - jua labs board
const uint8_t SD_CARD_CS = 8;

// SD card related variables
SdFat sd_card;
SdFile data_file;

// application constants
const uint16_t GPS_READ_PERIOD_MS = 60000;
const uint16_t MPU_READ_PERIOD_MS = 2000;
const uint8_t NUM_SAMPLES = 20;
const uint8_t STATUS_LED_PIN = 13;

// application variables
unsigned long time_now = 0;
auto status_led = JLed(STATUS_LED_PIN);
bool dmp_ready = false;  // set true if DMP init was successful
uint8_t mpu_int_status;   // holds actual interrupt status byte from MPU
uint8_t dev_status;      // return status after each device operation (0 = success, !0 = error)
uint16_t packet_size;    // expected DMP packet size (default is 42 bytes)
uint16_t fifo_count;     // count of all bytes currently in FIFO
uint8_t fifo_buffer[64]; // FIFO storage buffer

struct mpu_samples_t {
  uint32_t ts;
  Quaternion q;
  int16_t gX;
  int16_t gY;
  int16_t gZ;
  int16_t aX;
  int16_t aY;
  int16_t aZ;
} mpu_samples[NUM_SAMPLES];

uint8_t cur_sample = 0;
time_t start_ts = 0;
char filename[30];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpu_interrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpu_interrupt = true;
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void read_gps() {
  if (gps.location.isUpdated() && gps.location.isValid()) {
    sprintf(filename, "%lu-gps.txt", start_ts);
    if (!data_file.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      status_led.Blink(100, 100).Forever();
      sd_card.errorHalt(F("\nerro na abertura do arquivo do gps!"));
    }
    setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
            gps.date.day(), gps.date.month(), gps.date.year());
    char sample_str[100];
    char lat_str[20];
    char lng_str[20];
    dtostrf(gps.location.lat(), 4, 6, lat_str);
    dtostrf(gps.location.lng(), 4, 6, lng_str);
    sprintf(sample_str, "%lu;%s;%s",
            now(), lat_str, lng_str);
#ifdef DEBUG
    Serial.print(F("\ntem localizacao... "));
    Serial.println(sample_str);
#endif
    // write GPS data on file
    data_file.println(sample_str);
    data_file.close();
    status_led.Blink(250, 250).Repeat(2);
  }
  status_led.Blink(500, 500).Forever();
}

void write_mpu_data() {
  sprintf(filename, "%lu-mpu.txt", start_ts);
  if (!data_file.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
    status_led.Blink(100, 100).Forever();
    sd_card.errorHalt(F("Erro na abertura do arquivo do acelerometro!"));
  }
#ifdef DEBUG
  Serial.println(F("writing accelerometer data..."));
#endif
  char sample_str[200];
  char qw_str[20];
  char qx_str[20];
  char qy_str[20];
  char qz_str[20];
  // escreve todas as amostras coletadas
  for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
    dtostrf(mpu_samples[i].q.w, 4, 6, qw_str);
    dtostrf(mpu_samples[i].q.x, 4, 6, qx_str);
    dtostrf(mpu_samples[i].q.y, 4, 6, qy_str);
    dtostrf(mpu_samples[i].q.z, 4, 6, qz_str);
    sprintf(sample_str, "%lu;%s;%s;%s;%s;%d;%d;%d;%d;%d;%d",
            mpu_samples[i].ts, qw_str, qx_str, qy_str, qz_str,
            mpu_samples[i].gX, mpu_samples[i].gY, mpu_samples[i].gZ,
            mpu_samples[i].aX, mpu_samples[i].aY, mpu_samples[i].aZ);
    // escreve valor 'x' do acelerometro no arquivo
    data_file.println(sample_str);
#ifdef DEBUG
  Serial.println(sample_str);
#endif
  }
#ifdef DEBUG
  Serial.println(F("ok"));
#endif
  // fecha o arquivo
  data_file.close();
}

void read_mpu() {
  fifo_count = mpu.getFIFOCount();
#ifdef DEBUG
  Serial.print("ps: "); Serial.print(packet_size);
  Serial.print(" | fc: "); Serial.println(fifo_count);
#endif
  while(fifo_count >= packet_size) {
    mpu.getFIFOBytes(fifo_buffer, packet_size);
    fifo_count -= packet_size;
    mpu_samples[cur_sample].ts = now();
    mpu.dmpGetQuaternion(&mpu_samples[cur_sample].q, fifo_buffer);
    mpu_samples[cur_sample].gX = (fifo_buffer[16] << 8) | fifo_buffer[17];
    mpu_samples[cur_sample].gY = (fifo_buffer[20] << 8) | fifo_buffer[21];
    mpu_samples[cur_sample].gZ = (fifo_buffer[24] << 8) | fifo_buffer[25];
    mpu_samples[cur_sample].aX = (fifo_buffer[28] << 8) | fifo_buffer[29];
    mpu_samples[cur_sample].aY = (fifo_buffer[32] << 8) | fifo_buffer[33];
    mpu_samples[cur_sample].aZ = (fifo_buffer[36] << 8) | fifo_buffer[37];
    cur_sample++;
    if (cur_sample == NUM_SAMPLES) {
      write_mpu_data();
      // reset buffer index
      cur_sample = 0;
    }
  }
}

void mpu_setup() {
    Wire.begin();
    Wire.setClock(400000);
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

#if 0
    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
#endif
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    dev_status = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (dev_status == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpu_int_status = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmp_ready = true;

        // get expected DMP packet size for later comparison
        packet_size = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(dev_status);
        Serial.println(F(")"));
    }
}


void setup() {
#ifdef DEBUG
  // init serial interface
  Serial.begin(115200);
#endif
  // init GPS communication
  ss.begin(GPSBaud);
  // init MPU-6050
  mpu_setup();
  // init SD card
  if (!sd_card.begin(SD_CARD_CS, SPI_HALF_SPEED)) {
    sd_card.initErrorHalt();
  }
  // wait for GPS fix
  status_led.Blink(250, 250).Forever();
#if 1
#ifdef DEBUG
  Serial.print(F("\nwaiting gps fix: "));
#endif
  do {
    status_led.Update();
    smartDelay(250);
  } while (!gps.location.isValid() || !gps.location.isUpdated() || 
           !gps.time.isValid() || !gps.time.isUpdated() ||
           !gps.date.isValid() || !gps.date.isUpdated());
#ifdef DEBUG
  Serial.println(F("ok"));
#endif
#endif
  // get time from GPS
  setTime(gps.time.hour(), gps.time.minute(), gps.time.second(),
          gps.date.day(), gps.date.month(), gps.date.year());
  start_ts = now();
}

void loop() {
  if (!dmp_ready) return;
  // verifies GPS read period to write data on file
  if (millis() > time_now + GPS_READ_PERIOD_MS) {
    time_now = millis();
    read_gps();
  }
  smartDelay(MPU_READ_PERIOD_MS);
  // reads the MPU data
  read_mpu();
}