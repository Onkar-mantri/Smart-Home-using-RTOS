#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "esp32-hal-ledc.h"



hd44780_I2Cexp lcd;  // I2C LCD object  
DHT dht(25, DHT11);

TaskHandle_t TaskLCD_Handle; // Task handle for LCD task
TaskHandle_t TaskDHT_Handle; // Task handle for LCD task
TaskHandle_t TaskLDR_Handle; // Task handle for LCD task
TaskHandle_t TaskPIR_Handle; // Task handle for LCD task
TaskHandle_t TaskOpen_Handle; // Task handle for LCD task
TaskHandle_t TaskClose_Handle; // Task handle for LCD task
TaskHandle_t TaskHealthCheck_Handle; // Task handle for LCD task

SemaphoreHandle_t xClose;
SemaphoreHandle_t xOpen;

bool light, motion, curtains = false;
volatile float temp;
float kp = 0.3, ki = 0.3 , kd = 0.1;

const int pwmPin = 32;   // PWM channel
const int pwmFreq = 5000;   // Frequency in Hz
const int pwmResolution = 8; // 8-bit resolution (0-255)


void TaskLCD(void *pvParameters) {
    (void)pvParameters;
    for (;;) 
    {
      lcd.clear();
      if(motion == false)
      {
        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(temp);
        lcd.setCursor(0,1);
        lcd.print("lights: ");
        if(light == false)
        {
          lcd.print("OFF");
        }
        else
        {
          lcd.print("ON");
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Curtains ");
        if(curtains == false)
        {
          lcd.print("Open");
        }
        else if (curtains == true)
        {
          lcd.print("Closed");
        }

      }
      else if(motion == true)
      {
        lcd.setCursor(5, 0);
        lcd.print("Motion");
        lcd.setCursor(4,1);
        lcd.print("Detected!");
      }
      vTaskDelay(pdMS_TO_TICKS(1500)); 
    }
}

void TaskDHT(void *pvParameters)
{
  (void)pvParameters;
  for(;;)
  {
    temp = dht.readTemperature();
    if(!isnan(temp))
    {
      Serial.print("Temp: ");
      Serial.println(temp);
    }
    else
    {
      Serial.println("Failed to read DHT11");
    }
    if(curtains == false && temp > 30)    //Curtains open and high temp
    {
      // Serial.println("Closing Curtains Semaphore");
      xSemaphoreGive(xClose);             //giving semaphore 
    }
    else if(curtains == true && temp < 30)
    {
      // Serial.println("Opening Curtains Semaphore");
      xSemaphoreGive(xOpen);              //Giving Semaphore
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void TaskLDR(void *pvParameters)
{
  (void)pvParameters;
  for(;;)
  {
    int l = analogRead(34);
    Serial.print("LDR: ");
    Serial.println(l);
    if(l<2000)      //Low light
    {   
      digitalWrite(18,HIGH);
      light = true;
    }
    else            //sufficient light
    {
      digitalWrite(18, LOW);
      light = false;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void TaskPIR(void *pvParameters)
{
  (void)pvParameters;
  for(;;)
  {
    if(digitalRead(35) == HIGH)   //motion detected
    {
      digitalWrite(19,HIGH);
      motion = true;
    }
    else                          // motion stopped
    {
      digitalWrite(19, LOW);
      motion = false;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void TaskOpen(void *pvParameters)
{
  (void)pvParameters;
  for(;;)
  if (xSemaphoreTake(xOpen, portMAX_DELAY) == pdTRUE)
  {

    // Suspend Tasks
    vTaskSuspend(TaskLCD_Handle);
    vTaskSuspend(TaskDHT_Handle);
    vTaskSuspend(TaskPIR_Handle);
    vTaskSuspend(TaskLDR_Handle);
    vTaskSuspend(TaskHealthCheck_Handle);
    {
      Serial.println("Opening Curtains");
      float integral, derrivative = 0;
      int current = 100;
      int target = 0;
      float error = current - target;
      float output, pwm;
      float lastError = 0;
      Serial.print("Error: ");
      Serial.println(error);

      // Configure motor to run forwards
      digitalWrite(16, LOW);
      digitalWrite(17, HIGH);
      ledcWrite(pwmPin, 0); 

      // Open Curtains
        while(abs(error)>5)
        {
          Serial.print("a");
          error = current - target;       //target = 0 and current = 100
          integral += error;
          derrivative = error - lastError;

          output = kp*error + ki*integral + kd*derrivative;
          Serial.println(output);   

          lastError = error;
          current = output;

          pwm = 100-output;
          if(pwm>100)
          {
            pwm = 100;
          }
          else if(pwm < 0)
          {
            pwm = 0;
          }
          ledcWrite(pwmPin, pwm); 

          delay(500);
        }
      //stop motor 
      digitalWrite(16, LOW);
      digitalWrite(17, LOW);
      ledcWrite(pwmPin, 0); 

      Serial.println("Curtains Opned");
      curtains = false;

      // Resume Tasks
      vTaskResume(TaskLCD_Handle);
      vTaskResume(TaskDHT_Handle);
      vTaskResume(TaskPIR_Handle);
      vTaskResume(TaskLDR_Handle);
      vTaskResume(TaskHealthCheck_Handle);
    }
  }
}

void TaskClose(void *pvParameters)
{
  (void)pvParameters;
  for(;;)
  {
    if (xSemaphoreTake(xClose, portMAX_DELAY) == pdTRUE)
    {
      // Suspend Tasks
      vTaskSuspend(TaskLCD_Handle);
      vTaskSuspend(TaskDHT_Handle);
      vTaskSuspend(TaskPIR_Handle);
      vTaskSuspend(TaskLDR_Handle);
      vTaskSuspend(TaskHealthCheck_Handle);

      float integral, derrivative = 0;
      int current = 0;
      int target = 100;
      float error = target - current;
      float output, pwm;
      float lastError = 0;
      Serial.println("Closing Curtains");
      Serial.print("Error: ");
      Serial.println(error);


      // Configure motor to run backwards
      digitalWrite(16, HIGH);
      digitalWrite(17, LOW);
      ledcWrite(pwmPin, 0); 
      // Curtains Closing
      while(abs(error)>5)
      {
        error = target - current;       //target = 100 and current = 0
        integral += error;
        derrivative = error - lastError;

        output = kp*error + ki*integral + kd*derrivative;
        Serial.println(output); 
        lastError = error;
        current = output;
        pwm = 100 - output;
        if(pwm>100)
        {
          pwm = 100;
        }
        else if(pwm<0)
        {
          pwm = 0;
        }
        ledcWrite(pwmPin, pwm); 
        delay(500);
      }

      //stop motor
      digitalWrite(16, LOW);
      digitalWrite(17, LOW);
      ledcWrite(pwmPin, 0);

      Serial.println("Curtains Closed");   
      curtains = true;

      // Resume Tasks
      vTaskResume(TaskLCD_Handle);
      vTaskResume(TaskDHT_Handle);
      vTaskResume(TaskPIR_Handle);
      vTaskResume(TaskLDR_Handle);
      vTaskResume(TaskHealthCheck_Handle);
    }
  }
}

void TaskSystemHealth(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        Serial.println("System Health Check:");

        //Check Free Heap Memory
        Serial.print("Free Heap: ");
        Serial.print(esp_get_free_heap_size());
        Serial.println(" bytes");


        // Since LDR and PIR do not require much memory, sys check is not performed
        // LCD check
        Serial.print("LCD Task Stack: ");
        Serial.println(uxTaskGetStackHighWaterMark(TaskLCD_Handle));
        
        // DHT check
        Serial.print("DHT Task Stack: ");
        Serial.println(uxTaskGetStackHighWaterMark(TaskDHT_Handle));

        vTaskDelay(pdMS_TO_TICKS(5000));  // Run every 5 seconds
    }
}







void setup() {
    Serial.begin(115200);

    // DHT Init
    dht.begin();

    // LCD Init
    lcd.begin(16, 2);  
    lcd.backlight();
    delay(100);


    pinMode(35,INPUT);
    pinMode(18,OUTPUT);
    pinMode(19,OUTPUT);
    pinMode(17,OUTPUT);   //IN1
    pinMode(16,OUTPUT);   //IN2

    // ledcSetup(pwmChannel, pwmFreq, pwmResolution);
    ledcAttach(pwmPin,pwmFreq, pwmResolution );

    // Ceate Semaphores
    xClose = xSemaphoreCreateBinary();
    xOpen = xSemaphoreCreateBinary();

    if (xClose == NULL || xOpen == NULL) 
    {
      Serial.println("ERROR: Semaphore creation failed!");
      while (1); // Halt execution if semaphores fail
    }
    else{
      Serial.println("semaphores Succefully Created");
    }

    // Create FreeRTOS Task for LCD control
    xTaskCreate(TaskSystemHealth, "Health Check Task", 4096, NULL, 1, &TaskHealthCheck_Handle);
    xTaskCreate(TaskLCD, "LCD Task", 4096, NULL, 2, &TaskLCD_Handle);
    xTaskCreate(TaskDHT, "DHT Task", 2048, NULL, 3, &TaskDHT_Handle);
    xTaskCreate(TaskLDR, "LDR Task", 1024, NULL, 4, &TaskLDR_Handle);
    xTaskCreate(TaskPIR, "PIR Task", 1024, NULL, 5, &TaskPIR_Handle);
    xTaskCreate(TaskOpen, "Curtain Open", 1024, NULL, 6,&TaskOpen_Handle);
    xTaskCreate(TaskClose, "Curtain Close", 1024, NULL, 6, &TaskClose_Handle);
}

void loop() {}
