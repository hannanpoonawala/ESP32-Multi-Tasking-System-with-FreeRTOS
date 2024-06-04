#include <freertos/FreeRTOS.h> // Include FreeRTOS header
#include <freertos/task.h> // Include FreeRTOS task header
#include <freertos/queue.h> // Include FreeRTOS queue header
#include <Adafruit_SSD1306.h> // Include Adafruit SSD1306 OLED display library
#include <Adafruit_GFX.h> // Include Adafruit GFX library
#include <ESP32Servo.h> // Include ESP32Servo library
#include <DHTesp.h> // Include DHT sensor library for ESP32
#include <Wire.h> // Include Wire library for I2C communication

// Define OLED display width and height
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Define OLED reset pin (not used in this setup)
#define OLED_RESET -1

// Define pins for components
#define BLUE 14 // Blue LED pin
#define RED 12 // Red LED pin
#define DHT_PIN 15 // DHT22 sensor pin
#define SERVO_1 16 // Servo 1 pin
#define SERVO_2 4 // Servo 2 pin

// Setup OLED display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Setup servo objects
Servo servo1; // Servo 1 object
Servo servo2; // Servo 2 object
DHTesp dht; // DHT22 sensor object

// Define task handles for each task
TaskHandle_t BlueLedHandle = NULL;
TaskHandle_t RedLedHandle = NULL;
TaskHandle_t DHT22SensorTaskHandle = NULL;
TaskHandle_t ServoAngle1Handle = NULL;
TaskHandle_t ServoAngle2Handle = NULL;
TaskHandle_t OLEDDisplayTaskHandle = NULL;
TaskHandle_t SerialTaskHandle = NULL;

// Define queues for inter-task communication
QueueHandle_t TemperatureQueue; // Queue for temperature data
QueueHandle_t HumidityQueue; // Queue for humidity data
QueueHandle_t ServoAngle1Queue; // Queue for servo 1 angle
QueueHandle_t ServoAngle2Queue; // Queue for servo 2 angle
QueueHandle_t BlueLedStateQueue; // Queue for blue LED state
QueueHandle_t RedLedStateQueue; // Queue for red LED state
QueueHandle_t SerialQueue; // Queue for serial commands

// Structure to hold serial commands
struct SerialCommand {
  String command; // Command string
};

// Task for controlling the blue LED
void blinkBlue(void *pvParameter){
  pinMode(BLUE, OUTPUT); // Set blue LED pin as output
  bool blueLedState = false; // Initial state of blue LED
  while(1){
    // Receive blue LED state from queue
    if(xQueueReceive(BlueLedStateQueue, &blueLedState, portMAX_DELAY)){
      digitalWrite(BLUE, blueLedState ? HIGH : LOW); // Set blue LED state
    }
    vTaskDelay(pdMS_TO_TICKS(250)); // Delay for 250 ms
  }
}

// Task for controlling the red LED
void blinkRed(void *pvParameter){
  pinMode(RED, OUTPUT); // Set red LED pin as output
  bool redLedState = false; // Initial state of red LED
  while(1){
    // Receive red LED state from queue
    if(xQueueReceive(RedLedStateQueue, &redLedState, portMAX_DELAY)){
      digitalWrite(RED, redLedState ? HIGH : LOW); // Set red LED state
    }
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 ms
  }
}

// Task for reading data from the DHT22 sensor
void DHT22SensorTask(void *pvParameters) {
  while (1) {
    TempAndHumidity data = dht.getTempAndHumidity(); // Read temperature and humidity data
    float temperature = data.temperature; // Store temperature value
    float humidity = data.humidity; // Store humidity value

    // Send temperature and humidity data to their respective queues
    xQueueSend(TemperatureQueue, &temperature, portMAX_DELAY);
    xQueueSend(HumidityQueue, &humidity, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ms
  }
}

// Task for controlling Servo 1
void Servo1Task(void *pvParameter){
  int angle1; // Variable to store servo angle
  while(1){
    // Receive servo angle from queue
    if(xQueueReceive(ServoAngle1Queue, &angle1, portMAX_DELAY)){
      servo1.write(angle1); // Set servo 1 angle
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 ms
  }
}

// Task for controlling Servo 2
void Servo2Task(void *pvParameter){
  int angle2; // Variable to store servo angle
  while(1){
    // Receive servo angle from queue
    if(xQueueReceive(ServoAngle2Queue, &angle2, portMAX_DELAY)){
      servo2.write(angle2); // Set servo 2 angle
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 ms
  }
}

// Task for handling serial commands
void SerialTask(void *pvParameter){
  String command; // Variable to store received command
  SerialCommand serialCommand; // Serial command structure

  while(1){
    if(Serial.available() > 0){
      command = Serial.readStringUntil('\n'); // Read command from serial input
      command.trim(); // Trim whitespace from command
      serialCommand.command = command; // Store command in structure

      if(command.equalsIgnoreCase("blueOn")){
        bool state = true; // Set state to true
        xQueueSend(BlueLedStateQueue, &state, portMAX_DELAY); // Send state to blue LED queue
      }else if(command.equalsIgnoreCase("blueOff")){
        bool state = false; // Set state to false
        xQueueSend(BlueLedStateQueue, &state, portMAX_DELAY); // Send state to blue LED queue
      }else if(command.equalsIgnoreCase("redOn")){
        bool state = true; // Set state to true
        xQueueSend(RedLedStateQueue, &state, portMAX_DELAY); // Send state to red LED queue
      }else if(command.equalsIgnoreCase("redOff")){
        bool state = false; // Set state to false
        xQueueSend(RedLedStateQueue, &state, portMAX_DELAY); // Send state to red LED queue
      }else if(command.startsWith("servo1.")){
        int angle = command.substring(7).toInt(); // Parse angle from command
        if(angle >= 0 && angle <= 180){
          xQueueSend(ServoAngle1Queue, &angle, portMAX_DELAY); // Send angle to servo 1 queue
        }else{
          Serial.println("Angle should be between 0 to 180"); // Print error if angle is out of range
        }
      }else if(command.startsWith("servo2.")){
        int angle = command.substring(7).toInt(); // Parse angle from command
        if(angle >= 0 && angle <= 180){
          xQueueSend(ServoAngle2Queue, &angle, portMAX_DELAY); // Send angle to servo 2 queue
        }else{
          Serial.println("Angle should be between 0 to 180"); // Print error if angle is out of range
        }
      }else{
        Serial.println("Not a Command"); // Print error if command is not recognized
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 10 ms
  }
}

// Task for updating the OLED display
void OLEDDisplayTask(void *pvParameters) {
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed")); // Print error if display initialization fails
    for(;;); // Infinite loop to halt execution
  }
  display.display(); // Initialize display
  delay(2000); // Delay for 2 seconds

  // Variables to store last received values
  float lastTemperature = -1000;
  float lastHumidity = -1000;
  int lastServo1Angle = 90;
  int lastServo2Angle = 90;
  bool lastBlueLedState = false;
  bool lastRedLedState = false;

  while (1) {
    float temperature; // Variable to store temperature
    float humidity; // Variable to store humidity
    int servo1Angle; // Variable to store servo 1 angle
    int servo2Angle; // Variable to store servo 2 angle
    bool blueLedState; // Variable to store blue LED state
    bool redLedState; // Variable to store red LED state

    // Use portMAX_DELAY to ensure we wait for the latest data
    if (xQueueReceive(TemperatureQueue, &temperature, 0)) {
      lastTemperature = temperature; // Update last temperature value
    }
    if (xQueueReceive(HumidityQueue, &humidity, 0)) {
      lastHumidity = humidity; // Update last humidity value
    }
    if (xQueueReceive(ServoAngle1Queue, &servo1Angle, 0)) {
      lastServo1Angle = servo1Angle; // Update last servo 1 angle
    }
    if (xQueueReceive(ServoAngle2Queue, &servo2Angle, 0)) {
      lastServo2Angle = servo2Angle; // Update last servo 2 angle
    }
    if (xQueueReceive(BlueLedStateQueue, &blueLedState, 0)) {
      lastBlueLedState = blueLedState; // Update last blue LED state
    }
    if (xQueueReceive(RedLedStateQueue, &redLedState, 0)) {
      lastRedLedState = redLedState; // Update last red LED state
    }

    display.clearDisplay(); // Clear display
    display.setTextSize(1); // Set text size to 1
    display.setTextColor(SSD1306_WHITE); // Set text color to white

    display.setCursor(0, 0); // Set cursor to top-left corner
    display.print("Temp: "); // Print label
    display.print(lastTemperature); // Print temperature value
    display.println(" C"); // Print unit

    display.setCursor(0, 10); // Set cursor to next line
    display.print("Humidity: "); // Print label
    display.print(lastHumidity); // Print humidity value
    display.println(" %"); // Print unit

    display.setCursor(0, 20); // Set cursor to next line
    display.print("Servo 1: "); // Print label
    display.print(lastServo1Angle); // Print servo 1 angle
    display.println(" deg"); // Print unit

    display.setCursor(0, 30); // Set cursor to next line
    display.print("Servo 2: "); // Print label
    display.print(lastServo2Angle); // Print servo 2 angle
    display.println(" deg"); // Print unit

    display.setCursor(0, 40); // Set cursor to next line
    display.print("Blue LED: "); // Print label
    display.println(lastBlueLedState ? "ON" : "OFF"); // Print blue LED state

    display.setCursor(0, 50); // Set cursor to next line
    display.print("Red LED: "); // Print label
    display.println(lastRedLedState ? "ON" : "OFF"); // Print red LED state

    display.display(); // Update display with new data

    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ms
  }
}

// Setup function, called once when the program starts
void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud

  servo1.attach(SERVO_1); // Attach servo 1 to pin
  servo2.attach(SERVO_2); // Attach servo 2 to pin

  dht.setup(DHT_PIN, DHTesp::DHT22); // Setup DHT22 sensor

  // Create queues with capacity of 1 for each type of data
  TemperatureQueue = xQueueCreate(1, sizeof(float));
  HumidityQueue = xQueueCreate(1, sizeof(float));
  ServoAngle1Queue = xQueueCreate(1, sizeof(int));
  ServoAngle2Queue = xQueueCreate(1, sizeof(int));
  BlueLedStateQueue = xQueueCreate(1, sizeof(bool));
  RedLedStateQueue = xQueueCreate(1, sizeof(bool));
  SerialQueue = xQueueCreate(10, sizeof(SerialCommand)); // Create serial queue with capacity of 10

  // Create tasks and assign them to specific cores
  xTaskCreatePinnedToCore(blinkBlue, "blinkBlue", 1024, NULL, 1, &BlueLedHandle, 0);
  xTaskCreatePinnedToCore(blinkRed, "blinkRed", 1024, NULL, 1, &RedLedHandle, 0);
  xTaskCreatePinnedToCore(DHT22SensorTask, "DHT22SensorTask", 2048, NULL, 1, &DHT22SensorTaskHandle, 1);
  xTaskCreatePinnedToCore(Servo1Task, "servo1", 1024, NULL, 1, &ServoAngle1Handle, 1);
  xTaskCreatePinnedToCore(Servo2Task, "servo2", 1024, NULL, 1, &ServoAngle2Handle, 1);
  xTaskCreatePinnedToCore(OLEDDisplayTask, "OLEDDisplayTask", 2048, NULL, 1, &OLEDDisplayTaskHandle, 1);
  xTaskCreatePinnedToCore(SerialTask, "SerialTask", 2048, NULL, 1, &SerialTaskHandle, 1);
}

// Main loop function, not used in this project as tasks run independently
void loop() {
}
