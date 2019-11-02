

/* The ESP32 has four SPi buses, however as of right now only two of
 * them are available to use, HSPI and VSPI. Simply using the SPI API 
 * as illustrated in Arduino examples will use VSPI, leaving HSPI unused.
 * 
 * However if we simply intialise two instance of the SPI class for both
 * of these buses both can be used. However when just using these the Arduino
 * way only will actually be outputting at a time.
 * 
 * Logic analyser capture is in the same folder as this example as
 * "multiple_bus_output.png"
 * 
 * created 30/04/2018 by Alistair Symonds
 */
#include "MAX86141.h"

#define DAC1 25
#define DAC2 26

static const int spiClk = 8000000; // 8 MHz

//uninitalised pointers to SPI objects
MAX86141 pulseOx1;
MAX86141 pulseOx2;

void setup() {
  Serial.begin(115200);
  dacWrite(DAC1, 139); // 3.3v = 255, 1.8v ~= 139 (low voltage chip logic)
  dacWrite(DAC2, 139);
  
  //initialise two instances of the SPIClass attached to VSPI and HSPI respectively
  pulseOx1.spi = new SPIClass(HSPI);
  pulseOx2.spi = new SPIClass(VSPI);

  pulseOx1.SS = 15;
  pulseOx2.SS = 5;
  //clock miso mosi ss

  //set up slave select pins as outputs as the Arduino API
  //doesn't handle automatically pulling SS low
  pinMode(15, OUTPUT); //HSPI SS
  pinMode(5, OUTPUT); //VSPI SS

  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  pulseOx1.spi->begin();
  //alternatively route through GPIO pins of your choice
  //vspi->begin(0, 2, 4, 33); //SCLK, MISO, MOSI, SS
  
  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  pulseOx2.spi->begin(); 
  //alternatively route through GPIO pins
  //hspi->begin(25, 26, 27, 32); //SCLK, MISO, MOSI, SS

  Serial.println("Init Pulse Ox 1");
  pulseOx1.init();
  Serial.println("Init Pulse Ox 2");
  pulseOx2.init();

}

// the loop function runs over and over again until power down or reset
void loop() {

  //Serial.println(uint8_t(0b00000001));
  uint8_t output1;
  uint8_t output2;
  Serial.println("pulseOx1 Output: ");
  pulseOx1.read_reg(REG_PART_ID, &output1);
  //Serial.println(output1);
  Serial.println("pulseOx2 Output: ");
  pulseOx2.read_reg(REG_PART_ID, &output2);
  //Serial.println(output2);
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
