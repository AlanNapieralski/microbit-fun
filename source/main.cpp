#include "MicroBit.h"

// Get a serial port running over USB back to the host PC
NRF52Pin    usbTx(ID_PIN_USBTX, MICROBIT_PIN_UART_TX, PIN_CAPABILITY_DIGITAL);
NRF52Pin    usbRx(ID_PIN_USBRX, MICROBIT_PIN_UART_RX, PIN_CAPABILITY_DIGITAL);
NRF52Serial serial(usbTx, usbRx, NRF_UARTE0);

// Create an I2C driver for the micro:bit's internal I2C bus that includes the accelerometer
NRF52Pin    i2cSdaInt(ID_PIN_SDA, MICROBIT_PIN_INT_SDA, PIN_CAPABILITY_DIGITAL);
NRF52Pin    i2cSclInt(ID_PIN_SCL, MICROBIT_PIN_INT_SCL, PIN_CAPABILITY_DIGITAL);
NRF52I2C    i2cInt(i2cSdaInt, i2cSclInt, NRF_TWIM0);

// Create an I2C driver for the micro:bit's external I2C bus exposed on the edge connector
NRF52Pin    i2cSdaExt(ID_PIN_P20, MICROBIT_PIN_EXT_SDA, PIN_CAPABILITY_DIGITAL);
NRF52Pin    i2cSclExt(ID_PIN_P19, MICROBIT_PIN_EXT_SCL, PIN_CAPABILITY_DIGITAL);
NRF52I2C    i2cExt(i2cSdaExt, i2cSclExt, NRF_TWIM1);

extern void initMicroBitDisplay(void);
extern void clearMicroBitDisplay(void);
extern void setMicroBitPixel(uint8_t, uint8_t);
extern void clearMicroBitPixel(uint8_t, uint8_t);

extern void initOledDisplay(void);
extern void clearOledDisplay(void);
extern void setOledPixel(uint8_t, uint8_t);
extern void clearOledPixel(uint8_t, uint8_t);
extern void drawOledLine(uint8_t, uint8_t, uint8_t, uint8_t);

extern void graphData(uint8_t);


int main() {

    while (1) {
        // display instructions
        serial.printf("\r\nEnter a number 1-n to run a CW3 subtask: ");
        int in = serial.getChar(SYNC_SPINWAIT);   // get a character from serial

        switch (in) {            // call a Subtask based on the character typed
            case '1':
                serial.printf("\r\nPart 1\r\n");

                initMicroBitDisplay();  // initialise the micro:bit LED display
                setMicroBitPixel(1,1);  // set seven pixels to display an icon
                setMicroBitPixel(3,1); 
                setMicroBitPixel(0,3); 
                setMicroBitPixel(4,3); 
                setMicroBitPixel(1,4); 
                setMicroBitPixel(2,4);
                setMicroBitPixel(3,4); 

                // loop to create a simple animation
                for (int i = 0; i<3; i++) {
                    NRFX_DELAY_MS(300);         // delay 300ms between animating
                    clearMicroBitPixel(1,1);    // toggle the centre pixel
                    NRFX_DELAY_MS(300);         // delay 300ms between animating
                    setMicroBitPixel(1,1);      // toggle the centre pixel
                }
      
                // return execution to the main loop that checks for user input
                break;
                
            case '2':
                serial.printf("\r\nPart 2\r\n");

                initOledDisplay();              // initialise the I2C OLED display
                
                // loop 16 times to light up a 4x4 block of 16 pixels 
                // in the middle of the 128x64 OLED display
                // i.e. top-left is at coordinate 62,30
                for (uint8_t i = 0; i < 16; i++) {          // for each of the 16 pixels
                    setOledPixel((i % 4)+62, (i / 4)+30);   // calc position and set it
                }

                // draw some lines on the OLED display
                drawOledLine(0, 0, 127, 0);     // top-left to top-right
                drawOledLine(0, 63, 127, 63);   // bottom-left to bottom-right
                drawOledLine(0, 0, 127, 63);    // top-left to bottom-right
                drawOledLine(0, 63, 127, 0);    // bottom-left to top-right

                // return execution to the main loop that checks for user input
                break;

            case '3':
                serial.printf("\r\nPart 3\r\n");

                graphData(20);                    // call the data graphing application, request 20 fps

                break;

            case 'a':
                serial.printf("\r\nTest a\r\n");

                // test code to show how to use CODAL I2C object to talk to on-board accelerometer 

                {
                    uint8_t i2cByteRead;
                    
                    // enable accelerometer by setting subadress 0x20 to 0x57, see LSM303AGR datasheet 
                    // then reada high part of X value from subaddress 0x29
                    // remember these functions need the I2C address lwft-shifted by one bit
                    i2cInt.writeRegister(0x19<<1, 0x20, 0x57);              
                    i2cInt.readRegister(0x19<<1, 0x29, &i2cByteRead, 1);
                    serial.printf("Accelerometer X high byte: %d\r\n", ((int8_t) i2cByteRead));
                }

                // return execution to the main loop that checks for user input
                break;
        }

    }

}
