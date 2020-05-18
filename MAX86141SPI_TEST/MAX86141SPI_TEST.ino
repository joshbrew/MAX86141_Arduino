#include "MAX86141.h"

#include <esp_timer.h>

static int spiClk = 1000000; // 8 MHz Maximum

//
// Pin Definitions.
//
#define MISO_PIN              19
#define MOSI_PIN              23
#define SCK_PIN               18
#define SS_PIN                5

#define VSPI_MISO             MISO
#define VSPI_MOSI             MOSI
#define VSPI_SCLK             SCK
#define VSPI_SS               SS_PIN
  
#define INT_PIN               17

#define GPIO1_PIN             16
#define GPIO2_PIN             4

//uninitalised pointers to SPI objects
MAX86141 pulseOx1;
//MAX86141 pulseOx2;

void setup() {
  Serial.begin(115200);

  //
  // Configure IO.
  //
  pinMode(SS_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT_PULLUP);
  //pinMode(GPIO1_PIN, OUTPUT);
  //pinMode(GPIO2_PIN, INPUT_PULLUP);

  digitalWrite(SS_PIN, LOW);
  //digitalWrite(GPIO1_PIN, HIGH);

  //initialise SPI
  pulseOx1.spi = new SPIClass(VSPI);

  pulseOx1.SS = 5;
  //clock miso mosi ss

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(5, OUTPUT); //VSPI SS

  Serial.println("Init SPI Port...");
  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5, INT = 17
  pulseOx1.spi->begin();
  delay(100);
  Serial.println("Init Pulse Ox 1...");
  pulseOx1.setDebug(false);
  pulseOx1.init(spiClk);

}

// the loop function runs over and over again until power down or reset
void loop() {

  //Serial.println(uint8_t(0b00000001));
  uint8_t output1;
  //Serial.println("pulseOx1 Output: ");
  //output = pulseOx1.read_reg(REG_PART_ID);
  //Serial.println(output1);

  pulseOx1.device_data_read();
  unsigned long currentMicros = esp_timer_get_time();

  int led1A = pulseOx1.led1A[0] - pulseOx1.ambA[0];
  int led2A = pulseOx1.led2A[0] - pulseOx1.ambA[0];
  int led1B = pulseOx1.led1B[0] - pulseOx1.ambB[0];
  int led2B = pulseOx1.led2B[0] - pulseOx1.ambB[0];
  
  Serial.println(currentMicros);
  Serial.print("P1: LED1: ");
  Serial.print(led1A);
  Serial.print(" | LED2: ");
  Serial.print(led2A);
  Serial.print(" | AMB: ");
  Serial.println(pulseOx1.ambA[0]);
  Serial.print("P2: LED1: ");
  Serial.print(led1B);
  Serial.print(" | LED2: ");
  Serial.print(led2B);
  Serial.print(" | AMB: ");
  Serial.println(pulseOx1.ambB[0]);

  Serial.print("P2 - P1: LED1: ");
  Serial.print(led1B - led1A);
  Serial.print(" | LED2: ");
  Serial.print(led2B - led2A);
  Serial.print(" | AMB: ");
  Serial.println(pulseOx1.ambB[0] - pulseOx1.ambA[0]);
    
  delay(100);
}
