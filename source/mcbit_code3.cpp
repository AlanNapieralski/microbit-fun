#include "MicroBit.h"

extern NRF52Serial serial;  // serial object for displaying type of sensor and debugging 
extern NRF52I2C    i2cInt;  // internal I2C object for talking to accelerometer
extern NRF52I2C    i2cExt;  // external I2C object for talking to OLED display


#define DISPLAY_X_SIZE 5
#define DISPLAY_Y_SIZE 5

// the bitmap in memory for the LED pixels
// you can change the type and the size if necessary, but keep the name the same
uint8_t microBitDisplayFrameBuffer[5][5] = {0}; // buffer[row][column]

// count of the row that is to be updated at the moment (min: 0, max: 4)
uint8_t CURRENT_ROW = 0;

// arrays containing the configuration for each column and row (left to right)
const uint32_t cols[DISPLAY_Y_SIZE] = {(1UL << MICROBIT_PIN_COL1), (1UL << MICROBIT_PIN_COL2), (1UL << MICROBIT_PIN_COL3), (1UL << (MICROBIT_PIN_COL4 - 32)), (1UL << MICROBIT_PIN_COL5)};
const uint32_t rows[DISPLAY_X_SIZE] = {(1UL << MICROBIT_PIN_ROW1), (1UL << MICROBIT_PIN_ROW2), (1UL << MICROBIT_PIN_ROW3), (1UL << MICROBIT_PIN_ROW4), (1UL << MICROBIT_PIN_ROW5)};


/***
 * @param timerType choose between 4 timers that are available
 * @param callback the isr function to be executed by the timer's interrupt
 */
static void enableTimerISR(IRQn_Type timerType, void (*callback)(void)) {
  // setting up a periodic timer that results in an interrupt
  NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
  // 2 to the power of 5 (prescaler) == 32 ... 16Mhz / 32 / 1000 ticks == 500hz
  NRF_TIMER0->CC[0] = 1000; // 500hz is 5 times as fast as 100hz, so going through 5 rows of the matrix gives us the 100hz refresh rate of the whole screen
  NRF_TIMER0->PRESCALER = 5;
  NRF_TIMER0->SHORTS = 1; // automatically resets the timer's counter
  NRF_TIMER0->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);

  // configuring that timer’s interrupt handler to show the correct row of the LED display.
  NVIC_SetVector(timerType, (uint32_t) callback);
  NVIC_SetPriority(timerType, 1); // highest priority as we don't want the screen to stop from refreshing
  NVIC_EnableIRQ(timerType);

  NRF_TIMER0->TASKS_START = 1;
}

// displays the whole row based on what values are flipped on in a buffer
static void displayRow(int row) {
  // disable the interrupt
  NRF_TIMER0->EVENTS_COMPARE[0] = 0; // clear events to prevent stopping the code execution
  NVIC_DisableIRQ(TIMER0_IRQn);

  // reset everything
  NRF_P0->OUTCLR = rows[0] | rows[1] | rows[2] | rows[3] | rows[4];
  NRF_P1->OUTCLR = rows[0] | rows[1] | rows[2] | rows[3] | rows[4];
  NRF_P0->OUTSET =    (1 << MICROBIT_PIN_COL1) |
                      (1 << MICROBIT_PIN_COL2) |
                      (1 << MICROBIT_PIN_COL3) |
                      (1 << MICROBIT_PIN_COL5);
  NRF_P1->OUTSET =    (1 << (MICROBIT_PIN_COL4 - 32));

  // using a binary mask that we can use to enable proper pixels in a row
  uint32_t colMask = 0;
  bool col4Used = false;

  for (int col = 0; col < DISPLAY_Y_SIZE; col++) {
    if (microBitDisplayFrameBuffer[row][col]) {
      if (col == 3) {
        col4Used = true;
      } else {
        colMask |= cols[col];
      }
    }
  }
  
  // edge case of column 4 as it's in a different pin
  if (col4Used) {
    NRF_P1->OUTCLR = (1UL << (MICROBIT_PIN_COL4 - 32));
  }

  NRF_P0->OUTSET = rows[row];
  NRF_P1->OUTSET = rows[row];
  NRF_P0->OUTCLR = colMask;

  NVIC_EnableIRQ(TIMER0_IRQn);
}


// change the row that is being displayed after an interrupt is thrown. (After 100hz)
static void microBitDisplayIsr() {
  displayRow(CURRENT_ROW);

  CURRENT_ROW++;
  // loop back to 0 upon reaching the last row
  if (CURRENT_ROW >= 5) {
    CURRENT_ROW = 0;
  }
}

static void clearMicroBitDisplayBuffer() {
  for (int y = 0; y < DISPLAY_Y_SIZE; y++) {
    for (int x = 0; x < DISPLAY_X_SIZE; x++) {
      microBitDisplayFrameBuffer[x][y] = 0;
    }
  }
}

//
void initMicroBitDisplay() {
  // clearing the display frame buffer
  clearMicroBitDisplayBuffer();

  // setting each row and column GPIO as output
  NRF_P0->DIRSET =    (1 << MICROBIT_PIN_ROW1) |
                      (1 << MICROBIT_PIN_ROW2) |
                      (1 << MICROBIT_PIN_ROW3) |
                      (1 << MICROBIT_PIN_ROW4) |
                      (1 << MICROBIT_PIN_ROW5) |

                      (1 << MICROBIT_PIN_COL1) |
                      (1 << MICROBIT_PIN_COL2) |
                      (1 << MICROBIT_PIN_COL3) |
                      (1 << MICROBIT_PIN_COL5);
  NRF_P1->DIRSET =    (1 << (MICROBIT_PIN_COL4 - 32)); // column 4 is 37, so 37 - 32 = 5

  enableTimerISR(TIMER0_IRQn, microBitDisplayIsr); // passing in the callback function
}

//
void clearMicroBitDisplay() {
  clearMicroBitDisplayBuffer();
}

//
void setMicroBitPixel(uint8_t x, uint8_t y) {
  microBitDisplayFrameBuffer[y][x] = 1;
}

//
void clearMicroBitPixel(uint8_t x, uint8_t y) {
  microBitDisplayFrameBuffer[y][x] = 0;
}

#define NUMBER_OF_COLUMNS 128
#define NUMBER_OF_PAGES 8
#define NUMBER_OF_ROWS 64

// the bitmap in memory for the OLED pixels
// you must specify the type and the size, but keep the name the same
uint8_t oledDisplayFrameBuffer[NUMBER_OF_PAGES][NUMBER_OF_COLUMNS];

#define DEVICE_ADDR 0x3C << 1

#define COMMAND_ADDR 0x80
#define DATA_ADDR 0x40

#define MUX_RATIO_REG 0xA8
#define MUX_RATIO_VALUE 0x3F

#define DISPLAY_OFFSET_REG 0xD3
#define DISPLAY_OFFSET_VALUE 0x00

#define DISPLAY_START_LINE_REG 0x40

#define SEGMENT_REMAP_REG 0xA1

#define COM_OUTPUT_SCAN_DIR_REG 0xC8

#define COM_PINS_HARDWARE_CONFIG_REG 0x12
#define COM_PINS_HARDWARE_CONFIG_VALUE 0x02

#define CONTRAST_CONTROL_REG 0x81
#define CONTRAST_CONTROL_VALUE 0x7F

#define DISABLE_ENTIRE_DISPLAY_ON_REG 0xA4

#define NORMAL_DISPLAY_REG 0xA6

#define OSC_FREQUENCY_REG 0xD5
#define OSC_FREQUENCY_VALUE 0x80

#define ENABLE_CHARGE_PUMP_REGULATOR_REG 0x8D
#define ENABLE_CHARGE_PUMP_REGULATOR_VALUE 0x14

#define DISPLAY_ON_REG 0xAF

#define ADDRESSING_MODE_REG 0x20
#define HORIZONTAL_ADDRESSING_MODE 0x0
#define VERTICAL_ADDRESSING_MODE 0x1

#define COLUMN_ADDRESS_REG 0x21

#define PAGE_ADDRESS_REG 0x22

static uint8_t controlCommands[] = {
  DEVICE_ADDR, // shifting left one bit as the least significant byte is deciding on whether the address is for the write or read
  COMMAND_ADDR,
  DATA_ADDR,
};

static uint8_t initCommands[] = {
    MUX_RATIO_REG, MUX_RATIO_VALUE,
    DISPLAY_OFFSET_REG, DISPLAY_OFFSET_VALUE,
    DISPLAY_START_LINE_REG,
    SEGMENT_REMAP_REG,
    COM_OUTPUT_SCAN_DIR_REG,
    COM_PINS_HARDWARE_CONFIG_REG, COM_PINS_HARDWARE_CONFIG_VALUE,
    CONTRAST_CONTROL_REG, CONTRAST_CONTROL_VALUE,
    DISABLE_ENTIRE_DISPLAY_ON_REG,
    NORMAL_DISPLAY_REG,
    OSC_FREQUENCY_REG, OSC_FREQUENCY_VALUE,
    ENABLE_CHARGE_PUMP_REGULATOR_REG, ENABLE_CHARGE_PUMP_REGULATOR_VALUE,
    DISPLAY_ON_REG,
    ADDRESSING_MODE_REG, HORIZONTAL_ADDRESSING_MODE
};

// function to write all the init commands with one I2C transaction preceding each register with a 0x80 control byte
void writeCommandBytes(uint8_t *buffer, int len) {
  int newBufferLength = len * 2; // accounting for control bytes added before each command sent
  uint8_t newBuffer[newBufferLength];
  int count = 0; // count for the new buffer (double the size of the buffer passed in)
  for (int i = 0; i < len; i++) {
    // add a control byte and then the command
    newBuffer[count] = COMMAND_ADDR;
    newBuffer[count + 1] = buffer[i];
    count += 2;
  }

  i2cExt.write(DEVICE_ADDR, newBuffer, newBufferLength);
}

// updates the boundaries of where the pixels are going to start being written into the display
void updateDisplayDrawBoundaries(uint8_t page_start, uint8_t page_end, uint8_t column_start, uint8_t column_end) {
  uint8_t columnAddressCommands[] = {COLUMN_ADDRESS_REG, column_start, column_end};
  writeCommandBytes(columnAddressCommands, 3);

  uint8_t pageAddressCommands[] = {PAGE_ADDRESS_REG, page_start, page_end};
  writeCommandBytes(pageAddressCommands, 3);
}

// iterate through the entire buffer and set each value to 0
void clearOledDisplay() {
  // In horizontal mode, pointer will automatically move through all pages
  int length = NUMBER_OF_COLUMNS * NUMBER_OF_PAGES + 1; // add 1 for the command at the start of the array
  uint8_t buffer[length]; // flattened version of the oledDisplayBuffer
  int count = 0;
  for (uint8_t col = 0; col < NUMBER_OF_COLUMNS; col++) {
    for(uint8_t page = 0; page < NUMBER_OF_PAGES; page++) {
      oledDisplayFrameBuffer[page][col] = 0;
      buffer[count + 1] = 0;
      count++;
    }
  }
  // making sure the boundaries are correct
  updateDisplayDrawBoundaries(0, NUMBER_OF_PAGES - 1, 0, NUMBER_OF_COLUMNS - 1); // full display
  buffer[0] = DATA_ADDR; // start the stream of bytes with a control byte
  i2cExt.write(DEVICE_ADDR, buffer, length);
}

//
void initOledDisplay() {
  writeCommandBytes(initCommands, sizeof(initCommands)); // initialisation
  clearOledDisplay();  // clearing the noise
}

// sets and writes one pixel at a time
void setOledPixel(uint8_t x, uint8_t y) {
  uint8_t page = y / 8; // gets the page number pased on y axis value
  uint8_t bitPosition = y % 8; // gets the bit position of the pixel in the column of the page

  oledDisplayFrameBuffer[page][x] |= (1 << bitPosition); // OR the bit to the buffer so that it doesn't delete other bits in each coordinate's byte

  updateDisplayDrawBoundaries(page, page, x, x);

  uint8_t buff[] = {DATA_ADDR, oledDisplayFrameBuffer[page][x]};
  i2cExt.write(DEVICE_ADDR, buff, 2);
}

// writes the whole buffer content on the display
void writeBuffer() {
  int length = NUMBER_OF_COLUMNS * NUMBER_OF_PAGES + 1; // add 1 for the command at the start of the array
  uint8_t buffer[length]; // flattened buffer
  buffer[0] = DATA_ADDR;
  int count = 0;
  for (uint8_t page = 0; page < NUMBER_OF_PAGES; page++) {
    for(uint8_t col = 0; col < NUMBER_OF_COLUMNS; col++) {
      buffer[count + 1] = oledDisplayFrameBuffer[page][col];
      count++;
    }
  }

  updateDisplayDrawBoundaries(0, NUMBER_OF_PAGES - 1, 0, NUMBER_OF_COLUMNS - 1); // full display
  i2cExt.write(DEVICE_ADDR, buffer, length);
}

// clears and writes one pixel at a time
void clearOledPixel(uint8_t x, uint8_t y) {
  uint8_t page = y / 8;
  uint8_t bitPosition = y % 8;

  oledDisplayFrameBuffer[page][x] &= ~(1 << bitPosition);

  updateDisplayDrawBoundaries(page, page, x, x);

  uint8_t buff[] = {DATA_ADDR, oledDisplayFrameBuffer[page][x]};
  i2cExt.write(DEVICE_ADDR, buff, 2);
}

void putHorizontalLineToBuffer(uint8_t x_start, uint8_t x_end, uint8_t y) {
  uint8_t lineLength = (x_end - x_start) + 1;
  uint8_t page = y / 8;
  uint8_t bitPosition = y % 8;

  for(uint8_t i = 0; i < lineLength; i++) {
    oledDisplayFrameBuffer[page][x_start + i] = (1 << bitPosition);
  }
}

void putVerticalLineToBuffer(uint8_t y_start, uint8_t y_end, uint8_t x) {
  uint8_t lineLength = (y_end - y_start) + 1;

  uint8_t startPage = y_start / 8;
  uint8_t startBitPosition = y_start % 8;

  uint8_t endPage = y_end / 8;
  uint8_t endBitPosition = y_end % 8;

  // double loop starting with the starting page of the vertical line to be written and ending with the endPage
  for (uint8_t i = startPage; i <= endPage; i++) {
    for (uint8_t bitPos = 0; bitPos < 8; bitPos++) {
      if (i == startPage) {
        // need to compute the byte value for the column of the start page
        uint8_t value = 0;
        // we go through each bit and OR to get the full value of the byte (column of a page)
        for (int bit = startBitPosition; bit < 8; bit++) {
          oledDisplayFrameBuffer[i][x] |= (1 << bit);
        }
      } else if (i == endPage) {
        // need to compute the value for the column of the end page
        uint8_t value = 0;
        // we go through each bit and OR to get the full value of the byte (column of a page)
        for (int bit = 0; bit < endBitPosition; bit++) {
          oledDisplayFrameBuffer[i][x] |= (1 << bit);
        }
      } else {
        oledDisplayFrameBuffer[i][x] |= 0xFF; // full line down (full page in one column)
      }
    }
  }
}

void putDiagonalLineToBuffer(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end) {
  int16_t differenceX = abs(x_end - x_start);  // difference in x
  int16_t differenceY = abs(y_end - y_start);  // difference in y

  int8_t stepX = (x_start < x_end) ? 1 : -1;  // step direction for x
  int8_t stepY = (y_start < y_end) ? 1 : -1;  // step direction for y
  int16_t err = differenceX - differenceY;  // starting error term

  int16_t x = x_start;
  int16_t y = y_start;

  while (1) {
    // making sure the value is in the bounds, then or it with the existing value in a buffer
    if (x >= 0 && x < NUMBER_OF_COLUMNS && y >= 0 && y < NUMBER_OF_ROWS) {
      // calculate the page and bit for the current (x, y) position
      uint8_t page = y / 8;
      uint8_t bitPosition = y % 8;

      // Set the bit in the framebuffer
      oledDisplayFrameBuffer[page][x] |= (1 << bitPosition);
    }

    // if we've reached both x_end and y_end, break the loop
    if (x == x_end && y == y_end) {
      break;
    }

    // Bresenham's line algorithm: calculate the error and then decide where to step
    int16_t e2 = err * 2; // scaled-up version of the current error term err, ensures that the calculations involving e2 remain in integer arithmetic

    // if the error term is large enough in x, move in its direction
    if (e2 > -differenceY) {
      err -= differenceY;
      x += stepX;
    }

    // if the error term is large enough in y, move in its direction
    if (e2 < differenceX) {
      err += differenceX;
      y += stepY;
    }
  }
}

//
void drawOledLine(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end) {
  // if we're drawing horizontal lines
  if (y_start == y_end) {
    putHorizontalLineToBuffer(x_start, x_end, y_start);
    writeBuffer();
  // if we're drawing vertical lines
  }  else if (x_start == x_end) {
    putVerticalLineToBuffer(y_start, y_end, x_start);
    writeBuffer();
  } else {
    putDiagonalLineToBuffer(x_start, y_start, x_end, y_end);
    writeBuffer();
  }
}


#define ACCELERATOR_ADDRESS 0x19 << 1

// (starting with LSB)
// bit 0 == enables X axis
// bit 3 == enables low power mode 
// bit 4-5 == Power Mode Selection of 25hz
#define CTRL_REG_A_CONFIG 0b111001

// gets one X value of the accelerometer
static int getAccelerometerXValue() {
    // acceleromter initialisation
    i2cInt.writeRegister(ACCELERATOR_ADDRESS, LSM303_CTRL_REG1_A, CTRL_REG_A_CONFIG);

    // reading the high and low registers of x axis
    int high_reg = LSM303_OUT_X_H_A;
    int low_reg = LSM303_OUT_X_L_A;

    // reading from the set registers
    uint8_t high, low;
    i2cInt.readRegister(ACCELERATOR_ADDRESS, high_reg, &high, 1);
    i2cInt.readRegister(ACCELERATOR_ADDRESS, low_reg, &low, 1);

    // shifting and merging 10 bit high and 2 bit low into 16 bit signed value
    int16_t signedValue = ((int16_t(high << 8) | low) >> 6);

    return signedValue;
}

// maps the values of the accelerometer to the values of the column pixels on the display
static int mapValueToDisplay(float value, float old_min, float old_max) {
  // minimum values based on the size of the screen
  float new_min = 0;
  float new_max = NUMBER_OF_ROWS - 1;

  // normalising algorithm that's offsetting the input, scaling the value to the new range, normalising the valuei n the old range and shifting to the new range
  int mappedValue = (int) ((value + old_max) * (new_max - new_min)) / (old_max - old_min) + new_min;

  return mappedValue;
}

// shifts the whole array to the left, sets zeros to the last column and writes it to the buffer
static void scrollHorizontally(uint8_t newXValue) {
  for (uint8_t page = 0; page < NUMBER_OF_PAGES; page++) {
    for(uint8_t column = 0; column < NUMBER_OF_COLUMNS; column++) {
        oledDisplayFrameBuffer[page][column] = oledDisplayFrameBuffer[page][column + 1];
    }
    oledDisplayFrameBuffer[page][NUMBER_OF_COLUMNS - 1] = 0; // clear the last column 
  }

  writeBuffer();
}

#define middleYAxis 31

// draws the vertical lines representing the acceleration value of the accelerometer
// high value will result in a high column on the screen
static void runAcceleratorMode() {
  clearOledDisplay();

  int currentColumn = 0;
  while (1) {
    // stop this mode if any button has triggered an event
    if (NRF_GPIOTE->EVENTS_IN[0] == 1 || NRF_GPIOTE->EVENTS_IN[1] == 1) {
      break;
    }

    int x = getAccelerometerXValue();
    int mappedX = mapValueToDisplay((float) x, -512.0, 511.0);

    // locks in the currentColumn on the last column of the screen and triggers the scroll upon reaching the end
    if (currentColumn < NUMBER_OF_COLUMNS - 1) {
      currentColumn++;
    } else {
      scrollHorizontally(mappedX);
    }

    drawOledLine(currentColumn, (NUMBER_OF_ROWS - 1) - mappedX, currentColumn, NUMBER_OF_ROWS - 1);
  }
}

// draws the horizontal line in the middle and vertical lines showing the jerk value of the accelerometer
// big difference in acceleration between the current reading of accelerometer and the previous will result with a long, positive (or negative) line
// positive is represented by a line going from the middle up and the negative by a line going from the middle down
static void runJerkMode() {
  int jerk;
  int previousAcc = 0;
  clearOledDisplay();

  // draw the middle line 
  drawOledLine(0, middleYAxis, 127, middleYAxis);
  
  int currentColumn = 0;
  while (1) {
    // stop this mode if any button has triggered an event
    if (NRF_GPIOTE->EVENTS_IN[0] == 1 || NRF_GPIOTE->EVENTS_IN[1] == 1) {
      break;
    }
    int acc = getAccelerometerXValue();
    jerk = acc - previousAcc;
    previousAcc = acc;
    int mappedX = mapValueToDisplay(jerk, -1023.0, 1023.0);

    // locks in the currentColumn on the last column of the screen and triggers the scroll upon reaching the end
    if (currentColumn < NUMBER_OF_COLUMNS - 1) {    
      currentColumn++;
    } else {
      scrollHorizontally(mappedX);
      setOledPixel(NUMBER_OF_COLUMNS - 1, middleYAxis);
    }
    // draw the value of jerk
    if (jerk > 0) { // if it's positive
      drawOledLine(currentColumn, (NUMBER_OF_ROWS - 1) - mappedX, currentColumn, middleYAxis);
    } else {
      drawOledLine(currentColumn, middleYAxis, currentColumn, (NUMBER_OF_ROWS - 1) - mappedX);
    }
  }
}

static void displayLeftMicrobitIndicator() {
  clearMicroBitDisplay();
  setMicroBitPixel(0, 0);
  setMicroBitPixel(0, 1);
  setMicroBitPixel(0, 2);
  setMicroBitPixel(0, 3);
  setMicroBitPixel(0, 4);
  setMicroBitPixel(2, 2); // middle pixel
}

static void displayRightMicrobitIndicator() {
  clearMicroBitDisplay();
  setMicroBitPixel(4, 0);
  setMicroBitPixel(4, 1);
  setMicroBitPixel(4, 2);
  setMicroBitPixel(4, 3);
  setMicroBitPixel(4, 4);
  setMicroBitPixel(2, 2); // middle pixel
}

// triggered by isr
static void handleButtons() {   
  if (NRF_GPIOTE->EVENTS_IN[0] == GPIOTE_EVENTS_IN_EVENTS_IN_Generated) { // if button A is pressed
    NRF_GPIOTE->EVENTS_IN[0] = GPIOTE_EVENTS_IN_EVENTS_IN_NotGenerated; // clear the event
    displayLeftMicrobitIndicator();
    runAcceleratorMode();
  } else if (NRF_GPIOTE->EVENTS_IN[1] == GPIOTE_EVENTS_IN_EVENTS_IN_Generated) { // if button B is pressed
    NRF_GPIOTE->EVENTS_IN[1] = GPIOTE_EVENTS_IN_EVENTS_IN_NotGenerated; // clear the event
    displayRightMicrobitIndicator();
    runJerkMode();
  }
}


// graphs based on the data of the accelerometer depending on the button pressed
// Button A == accelerometer graph
// Button B == jerk graph
void graphData(uint8_t refreshRate) {
  // set the buttons to input
  NRF_P0->PIN_CNF[BUTTONA] =   (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
  NRF_P0->PIN_CNF[BUTTONB] =   (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

  // enable and configure the events for each button
  // enable event, set the PSEL of the buttons and set the polirity from high (button not pressed) to low (button pressed)
  NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                          (BUTTONA << GPIOTE_CONFIG_PSEL_Pos) |
                          (p0 << GPIOTE_CONFIG_PORT_Pos) |
                          (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);

  NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
                          (BUTTONB << GPIOTE_CONFIG_PSEL_Pos) |
                          (p0 << GPIOTE_CONFIG_PORT_Pos) |
                          (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos);
  
  // configure the interrupts 
  NRF_GPIOTE->INTENSET =  (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos) |
                          (GPIOTE_INTENSET_IN1_Enabled << GPIOTE_INTENSET_IN1_Pos);

  // initialising the displays
  initOledDisplay();
  initMicroBitDisplay();

  // configure the isr to interrupt every time an event is set
  NVIC_SetVector(GPIOTE_IRQn, (uint32_t) handleButtons);
  NVIC_SetPriority(GPIOTE_IRQn, NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY); // low priority for buttons (must be lower than the timer isr)
  NVIC_EnableIRQ(GPIOTE_IRQn);
}
