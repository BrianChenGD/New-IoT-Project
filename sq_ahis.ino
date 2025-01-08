#include "bsp.h"
#include "siliqs_esp32.h"
#include "sensors/gps_measurement.h"
#include "sensors/air_pressure_measurement.h"
#include "sensors/magnetic_measurement.h"
#include "sensors/motion_measurement.h"


SemaphoreHandle_t serial1Mutex = xSemaphoreCreateMutex();
GPSMeasurement gps(pGPS_RX, pGPS_TX);
RGBLedService led(pRGB_LED, 1);
AirPressureMeasurement airPressureSensor;
Magnetometer mag(MQC5883_I2C_ADDRESS, pMQC5883_I2C_SDA, pMQC5883_I2C_SCL); // Address, SDA pin, SCL pin
MotionMeasurement motionSensor(MPU6050_I2C_ADDRESS, pMPU6050_I2C_SDA, pMPU6050_I2C_SCL, -1);


void io_led_test()
{
  int color_green[] = {0xFF, 0x00, 0x00};
  int color_blue[] = {0x00, 0x00, 0xFF};
  int color_red[] = {0x00, 0xFF, 0x00};
  int color_white[] = {0xFF, 0xFF, 0xFF};
  int color_off[] = {0x00, 0x00, 0x00};

  pinMode(pDIO1, INPUT_PULLUP);
  pinMode(pDIO2, INPUT_PULLUP);
  pinMode(pDIO3, INPUT_PULLUP);
  pinMode(pPMC, INPUT_PULLUP);

  if (!digitalRead(pDIO1))
  {
    Serial.println("DIO1 HIGH");
    led.set_led(color_green);
    return;
  }

  if (!digitalRead(pDIO2))
  {
    Serial.println("DIO2 HIGH");
    led.set_led(color_blue);
    return;
  }

  if (!digitalRead(pDIO3))
  {
    Serial.println("DIO3 HIGH");
    led.set_led(color_red);
    return;
  }

  if (!digitalRead(pPMC))
  {
    Serial.println("PMC HIGH");
    led.set_led(color_white);
    return;
  }
  if (digitalRead(pDIO1) && digitalRead(pDIO2) && digitalRead(pDIO3) && digitalRead(pPMC))
  {
    Serial.println("DIO1 DIO2 DIO3 PMC ALL HIGH");
    led.set_led(color_off);
    return;
  }
}
 void get_ble_scan_data()
 {
   String target_address = "D5:12:D8:96:00:31"; // 54 39 16 FF // 18 05 06 17 0D // 31 04 09 59 54 39
   static uint32_t time = millis();
   if (millis() - time > 5000)
   {
     time = millis();
     nimbleService.printDiscoveredDevices();
   }
   // Check if a specific device was found
   if (!nimbleService.foundDevice.getName().empty())
   {
     Serial.printf("Found Device by Name: %s \n", nimbleService.foundDevice.toString().c_str());

     if (String(nimbleService.foundDevice.getAddress().toString().c_str()) == target_address)
     {
       Serial.printf("Match Device Address: %s \n", nimbleService.foundDevice.getAddress().toString().c_str());
       Serial.printf("Device manufacturer data: %s \n", nimbleService.foundDevice.getManufacturerData().c_str());
     }
     else
     {
       Serial.println("Device address not match, skipped.");
     }

     // Reset foundDevice to an empty state after processing
     nimbleService.foundDevice = NimBLEAdvertisedDevice();
   }
 }

void get_gps_data()
{
  if (gps.gpsData.valid)
  {
    Serial.printf("Lat: %.2f Lon: %.2f Alt: %.2f\n", gps.gpsData.latitude, gps.gpsData.longitude, gps.gpsData.altitude);
    Serial.printf("%02d/%02d/%02dT%02d:%02d:%02d+%02d:%02d\n", gps.gpsData.time.yy, gps.gpsData.time.mm, gps.gpsData.time.dd, gps.gpsData.time.hr, gps.gpsData.time.min, gps.gpsData.time.sec, gps.gpsData.time.utcOffset_hours, gps.gpsData.time.utcOffset_minutes);
  }
}

// 创建 BLEATCommandService 实例
//BLEATCommandService BLEatService;
// 创建 UARTATCommandService 实例，使用 Serial 作为通信接口
//UARTATCommandService UARTatService;
void setup()
{
  // Initialize system and LED peripherals
  siliqs_esp32_setup(SQ_INFO);

  led.begin(); // Initialize the service

  Serial.println("RMT initialized with 100ns tick");

  //BLEatService.startTask();

   Serial.println("初始化 NimBLE 服务...");

   //初始化 NimBLE 服务
   nimbleService.init();

   //创建一个 FreeRTOS 任务来处理 BLE 扫描
   xTaskCreate(
       SQNimBLEService::bleTaskWrapper, // 任务函数包装器
       "NimBLE Scan Task",              // 任务名称
       4096,                            // 堆栈大小（字节）
       &nimbleService,                  // 传递给任务的参数（NimBLE 服务实例）
       1,                               // 任务优先级
       NULL                             // 任务句柄（可以为 NULL）
   );
   nimbleService.toFindDeviceName = "YT9";
   Serial.println("NimBLE 服务初始化完成，开始扫描...");

  // 初始化串口监视器
  // Serial.begin(115200);
  // Serial.println("Starting GPS Service Example...");

  // 启动GPS服务
  // siliqs_esp32_setup(SQ_DEBUG);
  // pinMode(pGPS_VCTRL, OUTPUT);
  // digitalWrite(pGPS_VCTRL, LOW);

  // gps.start(1000, &serial1Mutex);
  //pinMode(0,INPUT);

  //AirPressure measurement
  pinMode(pVext, OUTPUT);
  digitalWrite(pVext, LOW); // Power on
  airPressureSensor.begin();
  //Magnetometer
  mag.begin();
  //Motion Measurement
  Serial.println("Initializing motion sensor...");
  motionSensor.start(500, &i2cMutex);

}

void colorLoop()
{
  int color_green[] = {0xFF, 0x00, 0x00};
  int color_blue[] = {0x00, 0x00, 0xFF};
  int color_red[] = {0x00, 0xFF, 0x00};

  //Serial.println("I'm in colorLoop");
  led.set_led(color_blue);
  delay(500);
  led.set_led(color_red);
  delay(500);
  led.set_led(color_green);
  delay(500);
}

void airPressure()
{
  airPressureSensor.getMeasurement();
  Serial.print("Air Pressure: ");
  Serial.print(airPressureSensor.pressure);
  Serial.println(" hPa");
  Serial.print("DSP310 Temperature: ");
  Serial.print(airPressureSensor.temperature);
  Serial.println(" degrees of Celsius");
}

void magnetMeter()
{
  mag.getMeasurement();
  delay(1000);
}

void motion()
{
  Serial.println("ax: " + String(motionSensor.ax) + ", ay: " + String(motionSensor.ay) + ", az: " + String(motionSensor.az));
  Serial.println("gx: " + String(motionSensor.gx) + ", gy: " + String(motionSensor.gy) + ", gz: " + String(motionSensor.gz));
  delay(500); // Adjust the delay based on your application's needs
}

void loop()
{
  //io_led_test();
  //get_ble_scan_data();
  //get_gps_data();
  //delay(1000);
  static int c=0;

  if (digitalRead(0)==LOW)
  { 
    switch(c)
    {
      case 0:
      colorLoop(); //Serial.println("0");
      break;
      case 1:
      airPressure();//Serial.println("1");
      break;
      case 2:
      magnetMeter();//Serial.println("2");
      break;
      case 3:
      motion();//Serial.println("3");
      break;
      case 4:
      Serial.println("4");
      break;
      case 5:
      Serial.println("5");
      break;
      case 6:
      Serial.println("6");
      break;
   } 
   c++;
    if (c>6)
    {
      c=0;
    }
  }
  delay(1000);
}
