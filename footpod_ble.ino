#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PeakDetection.h>

#define bleServerName "FITEASY_BLE"
#define SERVICE_UUID "49adbb08-db9e-11ec-9d64-0242ac120002"

BLECharacteristic thrustCharacteristic("f2a83e7e-dba1-11ec-9d64-0242ac120002", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor thrustDescriptor(BLEUUID((uint16_t)0x2902));

#define MAX_SECONDS_BEFORE_SLEEP 30
#define MAX_THRUST 10
#define MIN_THRUST 1.5
#define ZERO_VECT_ADJUST 9.0

Adafruit_MPU6050 mpu;
PeakDetection peakDetection;

bool deviceConnected = false;

int interruptCounter = 0;
uint16_t thrust = 0;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer){
    deviceConnected = false;
  }
};

void setup() {
  Serial.begin(115200);
  
  BLEDevice::init(bleServerName);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *fiteasyService = pServer->createService(SERVICE_UUID);
  fiteasyService->addCharacteristic(&thrustCharacteristic);
  thrustDescriptor.setValue("FITEASY THRUST DATA");
  thrustCharacteristic.addDescriptor(&thrustDescriptor);
  fiteasyService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a connection to notify");
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  timer = timerBegin(0, 240, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);

  while(!mpu.begin()) {
    delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  peakDetection.begin(20, 4, 0.6);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  double accelX = a.acceleration.x;
  double accelY = a.acceleration.y;
  double accelZ = a.acceleration.z;

  double current_vector = sqrt((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));
  double total_vector = abs(current_vector - ZERO_VECT_ADJUST);
  peakDetection.add(total_vector);
  int peak_vector = peakDetection.getPeak();
  double filtered_vector = peakDetection.getFilt();

  if(filtered_vector > MAX_THRUST){
    thrust = 255;
  }else if(filtered_vector < MIN_THRUST){
    thrust = 0;
  }else{
    thrust = (uint16_t)(((filtered_vector - MIN_THRUST)*255.0)/(MAX_THRUST - MIN_THRUST));
  }

  Serial.println(thrust);
  
  if(deviceConnected){
    thrustCharacteristic.setValue(thrust);
    thrustCharacteristic.notify();
  }

  if(filtered_vector > MIN_THRUST){
    interruptCounter = 0;
  }

  if(interruptCounter > MAX_SECONDS_BEFORE_SLEEP){
    esp_deep_sleep_start();
    mpu.enableSleep(true);
  }
  
  delay(100);
}
