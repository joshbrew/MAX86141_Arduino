#include "MAX86141.h"

#define WRITE_EN 0x00
#define READ_EN 0xFF

/*write to register function*/
void MAX86141::write_reg(uint8_t address, uint8_t data_in)
{

/**< A buffer with data to transfer. */
    m_tx_buf[0] = address;  //Target Register
    m_tx_buf[1] = WRITE_EN; //Set Write mode
    m_tx_buf[2] = data_in;  //Byte to Write

    if(debug == true) {
        Serial.println("TX Buffer contents");
        Serial.print(m_tx_buf[0]);
        Serial.print("|");
        Serial.print(m_tx_buf[1]);
        Serial.print("|")
        Serial.println(m_tx_buf[2]);
    }
/**< A buffer for incoming data. */    

    m_rx_buf[0] = 0;
    m_rx_buf[1] = 0;
    m_rx_buf[2] = 0;

    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(SS, LOW);
    spi->transfer(m_tx_buf,3);
    //m_rx_buf[0] = spi->transfer(m_tx_buf[0]);
    //m_rx_buf[1] = spi->transfer(m_tx_buf[1]);
    //m_rx_buf[2] = spi->transfer(m_tx_buf[2]);
    digitalWrite(SS, HIGH);
    m_rx_buf[0] = m_tx_buf[0];
    m_rx_buf[1] = m_tx_buf[0];
    m_rx_buf[2] = m_tx_buf[0];

    if(debug == true) {
        Serial.println("RX Buffer contents");
        Serial.print(m_rx_buf[0]);
        Serial.print("|");
        Serial.print(m_rx_buf[1]);
        Serial.print("|");
        Serial.println(m_rx_buf[2]);
    }
    /*
    digitalWrite(SS, LOW);
    spi->transfer(m_rx_buf[0]);
    spi->transfer(m_rx_buf[1]);
    spi->transfer(m_rx_buf[2]);;
    digitalWrite(SS, HIGH);
    */
    spi->endTransaction();

}

/*read register function*/
void MAX86141::read_reg(uint8_t address, uint8_t *data_out)
{

    /**< A buffer with data to transfer. */
    m_tx_buf[0] = address;  //Target Address
    m_tx_buf[1] = READ_EN;  //Set Read mode
    m_tx_buf[2] = 0x00;     //

/**< A buffer for incoming data. */ 
    m_rx_buf[0] = 0;
    m_rx_buf[1] = 0;
    m_rx_buf[2] = 0;

    spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE3));
    digitalWrite(SS, LOW);
    spi->transfer(m_tx_buf,3);

    //m_rx_buf[0] = spi->transfer(m_tx_buf[0]);
    //m_rx_buf[1] = spi->transfer(m_tx_buf[1]);
    //m_rx_buf[2] = spi->transfer(m_tx_buf[2]);
    digitalWrite(SS, HIGH);
    m_rx_buf[0] = m_tx_buf[0];
    m_rx_buf[1] = m_tx_buf[0];
    m_rx_buf[2] = m_tx_buf[0];

    if(debug == true){
        Serial.println("RX Buffer contents");
        Serial.print(m_rx_buf[0]);
        Serial.print("|");
        Serial.print(m_rx_buf[1]);
        Serial.print("|");
        Serial.println(m_rx_buf[2]);
    }
/*
    digitalWrite(SS, LOW);
    spi->transfer(m_rx_buf[0]);
    spi->transfer(m_rx_buf[1]);
    spi->transfer(m_rx_buf[2]);
    digitalWrite(SS, HIGH);
  */
    spi->endTransaction();

    *data_out = m_rx_buf[2];
}


/*inspired by pseudo-code available on MAX86141 datasheet for initialisation*/
void MAX86141::init(int newSpiClk=8000000)
{
    setSpiClk(newSpiClk);

    uint8_t value; // byte format 0b1010101
    write_reg(REG_MODE_CONFIG, 0x01 ); //soft reset, bit turns to 0 after reset
    delayMicroseconds(1000);
    write_reg(REG_MODE_CONFIG, 0x02 ); //shutdown
    delayMicroseconds(1000);
    read_reg(REG_INTR_STATUS_1, &value ); // clear interrupt 1
    delayMicroseconds(1000);
    read_reg(REG_INTR_STATUS_2, &value );  // clear interrupt 2
    delayMicroseconds(1000);
    write_reg(REG_PPG_CONFIG_1, 0xBE );
    delayMicroseconds(1000);
    write_reg(REG_PPG_CONFIG_2, 0x60 );
    delayMicroseconds(1000);
    write_reg(REG_PPG_CONFIG_3, 0xC0 );
    delayMicroseconds(1000);
    write_reg(REG_PDIODE_BIAS, 0x11 ); //65Pf PD CAP
    delayMicroseconds(1000);
    //write_reg(REG_PICKET_FENCE, 0b11011010, debug); //Set error correction (prediction) mode.
    //delayMicroseconds(1000);
    write_reg(REG_LED1_PA, 0x80 ); //LED1 drive current percentages of maximum 123mA. 255 == 100%
    delayMicroseconds(1000);
    write_reg(REG_LED2_PA, 0x80 );//LED2 drive current
    delayMicroseconds(1000);
    write_reg(REG_LED_RANGE_1, 0xF );
    delayMicroseconds(1000);
    write_reg(REG_LED_SEQ_1, 0x00 ); //exposure settings for led1 and 2
    delayMicroseconds(1000);
    write_reg(REG_LED_SEQ_2, 0x00 ); //led3 can be direct ambient exposure, led4 off
    delayMicroseconds(1000);
    write_reg(REG_LED_SEQ_3, 0x00 ); //led5 and 6 off
    delayMicroseconds(1000);
    write_reg(REG_FIFO_CONFIG_2, 0xA ); // Set interrupt and rollover modes, bit 3 (in MSB mode) adds a sample count if set to 1
    delayMicroseconds(1000);
    write_reg(REG_MODE_CONFIG, 0x00 ); //start sampling
}

/* inspired by pseudo-code available on MAX86141 datasheet */
void MAX86141::device_data_read(void)
{
    uint8_t sample_count;
    uint8_t reg_val;
    uint8_t dataBuf[128*2*2*3]; ///128 FIFO samples, 2 channels, 2 PDs, 3 bytes/channel
      
    read_reg(REG_FIFO_DATA_COUNT, &sample_count); //number of items available in FIFO to read 

    read_fifo(dataBuf, sample_count*3);

    /*suitable formatting of data for 2 LEDs*/
    int i = 0;
    for (i=0; i<sample_count; i++)
    {
    led1A[i] = ((dataBuf[i*12+0] << 16 ) | (dataBuf[i*12+1] << 8) | (dataBuf[i*12+2])) & 0x7FFFF; // LED1, PD1
    led1B[i] = ((dataBuf[i*12+3] << 16 ) | (dataBuf[i*12+4] << 8) | (dataBuf[i*12+5])) & 0x7FFFF; // LED1, PD2
    led2A[i] = ((dataBuf[i*12+6] << 16 ) | (dataBuf[i*12+7] << 8) | (dataBuf[i*12+8])) & 0x7FFFF; // LED2, PD1
    led2B[i] = ((dataBuf[i*12+9] << 16 ) | (dataBuf[i*12+10] << 8) | (dataBuf[i*12+11])) & 0x7FFFF; // LED2, PD2
    } 

}

void MAX86141::fifo_intr()
{
    uint8_t count;
    read_reg(REG_FIFO_DATA_COUNT, &count); 

    if (count == 0x80) //indicates full FIFO
    { 
        device_data_read();
    }
 }


/*read FIFO*/
void MAX86141::read_fifo(uint8_t data_buffer[], uint8_t count)
{
    uint8_t output;
    for(int i =0; i<count; i++){

        read_reg(REG_FIFO_DATA, &output);
        data_buffer[i] = output;
    }
}

void MAX86141::setSS(int pin){
    SS = pin;
}

void MAX86141::setSPI(SPIClass * newspi){
    spi = newspi;
}

void MAX86141::setSpiClk(int newSpiClk) {
    spiClk = newSpiClk;
}

void MAX86141::setDebug(bool setdebug) {
    debug = setdebug;
}