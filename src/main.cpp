#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <cmath>
#include <STM32FreeRTOS.h>
#include <Wire.h>
#include <Print.h>
#include <stdlib.h>



// Constants
const uint32_t interval = 100; // Display update interval
volatile uint32_t currentStepSize = 0;
HardwareTimer sampleTimer(TIM1);
const std::string notelist[]{"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"}; 

// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

constexpr uint32_t fs = 22000;

constexpr uint32_t getstepsize(int f)
{
  uint64_t out = (((1 << 31) / fs) << 1) * f;
  return out;
}
#include <stdint.h>
struct
{
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
  uint8_t prevnobA  ;
  uint8_t prevnobB ;
  int knobrotate;
  int lastRotationDirection;
} sysState;

int Quad(uint8_t previousA, uint8_t previousB, uint8_t currentA, uint8_t currentB) {
  int rotationVariable = 0;

  Serial.print("Prev A,B: ");
  Serial.print(previousA);
  Serial.print(",");
  Serial.print(previousB);
  Serial.print(" -> ");
  Serial.print("Curr A,B: ");
  Serial.print(currentA);
  Serial.print(",");
  Serial.println(currentB);

  // Combine previous and current states into a 2-bit value
  uint8_t prevState = (previousA << 1) | previousB;
  uint8_t currState = (currentA << 1) | currentB;

  // Define valid state transitions
  switch (prevState) {
      case 0b00: // Previous state 00
          if (currState == 0b01) rotationVariable = +1; // 00 -> 01 (Clockwise)
          else if (currState == 0b11) rotationVariable = sysState.lastRotationDirection;
          break;
      case 0b01: // Previous state 01
          if (currState == 0b10) rotationVariable = sysState.lastRotationDirection;
          else if (currState == 0b00) rotationVariable = -1; // 01 -> 00 (Counterclockwise)
          break;
      case 0b10: // Previous state 10
          if (currState == 0b01) rotationVariable = sysState.lastRotationDirection; // 10 -> 00 (Clockwise)
          else if (currState == 0b11) rotationVariable = -1; // 10 -> 11 (Counterclockwise)
          break;
      case 0b11: // Previous state 11
          if (currState == 0b10) rotationVariable = +1; // 11 -> 10 (Clockwise)
          else if (currState == 0b00) rotationVariable =sysState.lastRotationDirection;
          break;
  }

  // Update the last direction after a valid transition
  if (rotationVariable != 0) {
      sysState.lastRotationDirection = rotationVariable;
  }

  return rotationVariable;
}





// constant step size
constexpr uint32_t stepSizes[] = {
    getstepsize(262),
    getstepsize(277),
    getstepsize(294),
    getstepsize(311),
    getstepsize(330),
    getstepsize(349),
    getstepsize(370),
    getstepsize(392),
    getstepsize(415),
    getstepsize(440),
    getstepsize(466),
    getstepsize(494)};

void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - sysState.knobrotate);
  analogWrite(OUTR_PIN, Vout + 128);
  
}

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

void setRow(uint8_t rowIdx)
{
  digitalWrite(REN_PIN, 0);
  std::bitset<3> enables(rowIdx);
  digitalWrite(RA0_PIN, enables[0]);
  digitalWrite(RA1_PIN, enables[1]);
  digitalWrite(RA2_PIN, enables[2]);
  digitalWrite(REN_PIN, 1);
}

std::bitset<4> readCols()
{
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

void scanKeysTask(void* pvParameters) {
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;  // Scan the keys every 50 ms
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int localrotation = 0;
  while (1) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait for the next scan cycle
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);  // Access shared resources safely

      // Scan the keypad for all rows (3 rows in total)
      for (int i = 0; i < 4; i++) {
          setRow(i);
          delayMicroseconds(40);  // Delay to stabilize readings
          std::bitset<4> inputcol = readCols();
          sysState.inputs[i * 4] = inputcol[0];
          sysState.inputs[i * 4 + 1] = inputcol[1];
          sysState.inputs[i * 4 + 2] = inputcol[2];
          sysState.inputs[i * 4 + 3] = inputcol[3];
      }

      // Update step size based on key presses (if any)
      uint32_t localCurrentStepSize = 0;
      for (int i = 0; i < 12; i++) {
          if (!sysState.inputs[i]) {
              localCurrentStepSize = stepSizes[i];
          }
      }
  
      uint8_t currentA = sysState.inputs[12];
      uint8_t currentB = sysState.inputs[13];

      

      sysState.knobrotate = sysState.knobrotate + Quad(sysState.prevnobA, sysState.prevnobB, currentA, currentB);
      if (sysState.knobrotate >8) sysState.knobrotate=8;
      sysState.prevnobA = currentA;
      sysState.prevnobB = currentB;
      Serial.print(sysState.knobrotate);
     

      xSemaphoreGive(sysState.mutex);  // Release mutex

      // Update the global current step size
      __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}


void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(2, 10);
    u8g2.drawStr(2, 10, "Hello World!");

    xSemaphoreTake(sysState.mutex, portMAX_DELAY); // Take mutex
    u8g2.setCursor(2, 20);
    u8g2.print(sysState.inputs.to_ulong(), HEX);

    int pressed = -1;
    for (int i = 0; i < 12; i++)
    {
      if (!sysState.inputs[i])
        pressed = i;
    }

    if (pressed != -1)
    {
      std::string message = notelist[pressed];
      u8g2.setCursor(2, 30);
      u8g2.print(message.c_str());
    }

    u8g2.setCursor(20, 30);
    
    u8g2.print(sysState.knobrotate);


    xSemaphoreGive(sysState.mutex);  // Release mutex
    u8g2.sendBuffer();

    digitalToggle(LED_BUILTIN);
  }
}

void setup()
{
  // put your setup code here, to run once:
  sysState.mutex = xSemaphoreCreateMutex();
  
  
  // Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
 Serial.println("Hello World");

  TaskHandle_t displayUpdateTaskHandle = NULL;
  xTaskCreate(
      displayUpdateTask,         /* Function that implements the task */
      "displayUpdateTask",       /* Text name for the task */
      256,                       /* Stack size in words, not bytes */
      NULL,                      /* Parameter passed into the task */
      1,                         /* Task priority */
      &displayUpdateTaskHandle); /* Pointer to store the task handle */

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      64,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      2,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */

  

  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
  vTaskStartScheduler();
}

void loop()
{
}