#include <Arduino.h>
#include <stdint.h>

#define DATA_PIN  4      // SER (Serial Input Data) 14
#define LATCH_PIN 3      // RCLK (Register Clock / Latch) 12
#define CLOCK_PIN 2      // SRCLK (Shift Register Clock) 11 

// Patterns for characters EMPTY, r, n, 1, 2, 3, 4, 5, 6, 7, 8, 9
uint8_t gearArray[12] = {
  0b00000000, // 0
  0b00001010, // REVERSE
  0b00000010, // NEUTRAL
  0b01100000, // 1
  0b11011010, // 2
  0b11110010, // 3  
  0b01100110, // 4
  0b10110110, // 5
  0b10111110, // 6
  0b11100000, // 7
  0b11111110, // 8
  0b11110110  // 9
};

uint8_t speedArray[11] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111, // 9
  0b00000000  // EMPTY
};

uint8_t digitToRender[4] = {
  0b0111, // 0
  0b1011, // 1
  0b1101, // 2
  0b1110, // 3
};

uint32_t replaceBits(uint32_t original, uint8_t replacement, int offset,
                     int width) {
  // Create a mask for the exact number of bits to replace
  uint32_t mask = ((1UL << width) - 1) << offset;

  // Clear the specified bits in `original` where the `replacement` will go
  original &= ~mask;

  // Insert the replacement bits in the cleared area
  original |= ((uint32_t)(replacement & ((1 << width) - 1)) << offset);

  return original;
}

uint32_t getDigit(int value, int digit) {
  int modolus = 10;
  
  // Calculate the total number of digits in the value
  int numDigits = 0;
  int temp = value;
  while (temp > 0) {
    temp /= 10;
    numDigits++;
  }

  // If the requested digit is out of range, return 10
  if (digit >= numDigits) {
    return 10;
  }

  // Find the modulus to get the requested digit
  for (int k = 0; k < digit; k++) {
    modolus *= 10;  
  }

  return (value % modolus) / (modolus / 10);
}

class Display {
private:
  int dataPin;
  int latchPin;
  int clockPin;

public:
  int gear = -2;
  int rpm = 0;
  //                      nth 595         1st 595
  //                      |       |       |                          
  uint32_t dataToSend = 0b000000000000000000000000;
  //                          .GFEDCBA~~~~ABCDEFG.
  //                                  | Controls the ones that are on 
  //                                  | 1 is OFF, 0 is ON

  Display(int d, int l, int c) {
    dataPin = d;
    latchPin = l;
    clockPin = c;
  }

  void setup() {
    pinMode(latchPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(dataPin, OUTPUT);
  }

  void draw(uint32_t data) {
    // ST_CP LOW to keep LEDs from changing while reading serial data
    digitalWrite(latchPin, LOW);

    shiftOut(dataPin, clockPin, MSBFIRST, (data >> 16) & 0xFF); // Last 595 in Chain
    shiftOut(dataPin, clockPin, MSBFIRST, (data >> 8) & 0xFF); // Last 595 in Chain
    shiftOut(dataPin, clockPin, MSBFIRST, (data) & 0xFF); // Last 595 in Chain

    // ST_CP HIGH change LEDs
    digitalWrite(latchPin, HIGH);

    delay(1);
  }

  void render() {

    dataToSend = replaceBits(dataToSend, gearArray[gear + 2], 0, 8);
    draw(dataToSend);

    for(int k = 0; k < 4; k++) {
      int dig = getDigit(rpm, k);
      dataToSend = replaceBits(dataToSend, digitToRender[k], 8, 4);
      dataToSend = replaceBits(dataToSend, speedArray[dig], 12, 8);  // Use `digit` instead of `k`

      draw(dataToSend);
    }
  }
};

Display d(DATA_PIN, LATCH_PIN, CLOCK_PIN);

void setup () {
  // Setup the baudrate or whatever
  Serial.begin(9600);

  // Setup the display
  d.setup();
}

uint32_t gearTick = 0;
uint32_t rpmTick = 0;

void loop() {
  // uint32_t curTime = millis();
  // if(curTime - gearTick > 1000) {
  //   if (d.gear == 11) {
  //     d.gear = -2;
  //   }
  //   d.gear++;
  //   gearTick = curTime;
  // }
  //
  // if(curTime - rpmTick > 10) {
  //   if (d.rpm == 1111) {
  //     d.rpm = 0;
  //   }
  //   d.rpm++;
  //   rpmTick = curTime;
  // }

  d.render();
  
  static char message[16];

  while (Serial.available() > 0) {
    //Create a place to hold the incoming message
    static unsigned int message_pos = 0;
    //Read the next available byte in the serial receive buffer
    char inByte = Serial.read();

    if(inByte != '\n') {
      message[message_pos] = inByte;
      message_pos++;
    } else {
      message[message_pos] = '\0';
      message_pos = 0;
    }
  }

  char results[2][32] = {};

  char s[256];
  strcpy(s, message);
  char* token = strtok(s, ",");
  int iter = 0;
  while(token) {
    strcpy(results[iter], token);
    token = strtok(NULL, ",");
    iter++;
  }

  d.gear = atoi(results[0]);
  d.rpm = atoi(results[1]);
}
