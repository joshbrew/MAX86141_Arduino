
#include "MAX86141.h"

int spiClk = 200000; // 8 MHz Maximum

//uninitalised pointers to SPI objects
MAX86141 pulseOx1;
//MAX86141 pulseOx2;

void setup() {
  Serial.begin(115200);
  
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  pulseOx1.spi = new SPIClass(VSPI);

  pulseOx1.SS = 5;
  //clock miso mosi ss

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(5, OUTPUT); //VSPI SS

  Serial.println("Init SPI Port...");
  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  pulseOx1.spi->begin();
  //alternatively route through GPIO pins of your choice
  //vspi->begin(0, 2, 4, 33); //SCLK, MISO, MOSI, SS
  delay(100);
  Serial.println("Init Pulse Ox 1...");
  pulseOx1.init(spiClk, true);

}

// the loop function runs over and over again until power down or reset
void loop() {

  //Serial.println(uint8_t(0b00000001));
  uint8_t output1;
  Serial.println("pulseOx1 Output: ");
  pulseOx1.read_reg(REG_PART_ID, &output1, true);
  //Serial.println(output1);
  /*
  uint8_t count;
  
  pulseOx1.read_reg(REG_FIFO_DATA_COUNT, &count); 
  if (count& 0x80) //indicates full FIFO
  { 
    pulseOx1.device_data_read();
 
    Serial.print("LED1 P1: ");
    Serial.println(pulseOx1.led1A[0]);
    Serial.print("LED1 P2: ");
    Serial.println(pulseOx1.led1B[0]);
    Serial.print("LED2 P1: ");
    Serial.println(pulseOx1.led2A[0]);
    Serial.print("LED2 P2: ");
    Serial.println(pulseOx1.led2B[0]);    
  }
  */
  delay(1000);
}
