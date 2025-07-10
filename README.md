# microbic-fun
Exploring the possibilities of the micro-bit.

## Micro:bit Bare-Metal Display and Graphing System

This project showcases a comprehensive bare-metal implementation for both the built-in Micro:bit 5x5 LED matrix and an external OLED display over I2C. All functionality was implemented from scratch in C, without relying on high-level display libraries or abstraction layers—interacting directly with hardware registers and peripheral control logic. This low-level, interrupt-driven design achieves efficient and precise control, demonstrating an in-depth understanding of embedded systems.

## Features

### ✅ Micro:bit 5x5 LED Matrix Driver

Implemented a complete LED display driver by directly interacting with GPIOs and hardware timers. A custom framebuffer and interrupt-driven row scanning algorithm were developed to emulate simultaneous pixel lighting, updating the entire display at **100Hz**, mimicking persistence of vision.

- **Functionality:**
  - `initMicroBitDisplay(void)`: Configures GPIO pins, sets up the timer interrupt for display scanning, and clears the framebuffer.
  - `clearMicroBitDisplay(void)`: Resets all pixel data.
  - `setMicroBitPixel(uint8_t x, uint8_t y)`: Lights up a pixel at specified coordinates.
  - `clearMicroBitPixel(uint8_t x, uint8_t y)`: Turns off a pixel at specified coordinates.
  - `microBitDisplayIsr()`: Interrupt handler that updates one row of the display on each timer interrupt.

> **Note:** No busy-wait loops or delay-based timing. All updates are managed via hardware interrupts, ensuring responsiveness and low power consumption.

---

### 🟦 OLED Display Driver (SSD1306)

Developed a custom display controller for an external OLED screen connected via I2C. The SSD1306 was initialized and managed manually over the I2C_EXT bus using the CODAL I2C object, but all higher-level logic (pixel handling, drawing, framebuffer management) was custom-implemented from scratch.

- **Functionality:**
  - `initOledDisplay(void)`: Sends initialization commands over I2C to the OLED controller.
  - `clearOledDisplay(void)`: Clears both the framebuffer and display.
  - `setOledPixel(uint8_t x, uint8_t y)`: Sets a pixel in the framebuffer and updates the corresponding byte on the display.
  - `clearOledPixel(uint8_t x, uint8_t y)`: Clears a specific pixel.
  - `drawOledLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)`: Implements line drawing using a rasterization algorithm (Bresenham's or similar).

> Direct manipulation of bit patterns and minimal RAM usage were prioritized, achieving fast rendering with minimal I2C traffic.

---

### 📈 Real-Time Accelerometer Graphing Application

Built a real-time graphing system using the external OLED display, capable of plotting **live accelerometer readings** and **jerk (first derivative of acceleration)** over time. Graph data scrolls horizontally with new values plotted at the right edge of the screen.

- **Functionality:**
  - `graphData(uint8_t refreshRate)`: Infinite loop that reads accelerometer X-axis data and displays:
    - **Mode A:** Raw acceleration data plotted as vertical bars from the bottom up.
    - **Mode B:** Jerk values plotted relative to a mid-screen baseline (y=31).
  - Data scrolling, display refresh, and visual scaling were all manually controlled with no third-party drawing utilities.
  - Efficient frame shifting was implemented to allow smooth horizontal scrolling of data.

- **Mode Switching:**
  - Implemented using GPIOTE-generated interrupts (not polling).
  - Pressing Button A: switches to acceleration mode.
  - Pressing Button B: switches to jerk mode.
  - Middle LED and side columns on the Micro:bit matrix were used as visual indicators for active mode.

---

## Key Technical Highlights

- **Bare-metal implementation** with direct register manipulation.
- **No display libraries** or built-in abstraction layers used.
- Efficient **interrupt-driven rendering** on the Micro:bit display.
- Custom **framebuffer management** for both displays.
- Optimized **I2C communication** to minimize overhead and flicker.
- Real-time data visualization with **graph scrolling** and **mode switching**.

---

## What I Learned

- ⚙️ How to configure and handle **hardware interrupts** for real-time tasks.
- 💡 Techniques for **scanning LED matrices** with persistence of vision using timers.
- 🔧 How to communicate with an **I2C peripheral (SSD1306)** at a low level.
- 🧠 Implemented custom drawing algorithms including **pixel plotting** and **line rasterization**.
- 📉 Real-time data processing and **graphical representation of sensor data**.
- 🖥️ Memory-efficient framebuffer manipulation.
- ⏱️ Performance tuning for **interrupts, I2C**, and refresh rates.
- 🧵 Using **GPIOTE peripheral** to generate and handle button press interrupts.
- 🔍 Debugging and verifying embedded applications without access to standard IO.
- 🧼 Writing maintainable, well-documented, and **scalable low-level C code**.

---

## Summary

This project demonstrates a thorough and low-level understanding of embedded systems design on the Micro:bit platform. From direct GPIO configuration to I2C device initialization and interrupt-driven scheduling, all core display and sensor functionality was built from first principles. The result is a responsive and efficient embedded graphing system that integrates real-time data capture with visually intuitive display output.





# microbit-v2-usage
[![Native Build Status](https://github.com/lancaster-university/microbit-v2-samples/actions/workflows/build.yml/badge.svg)](https://github.com/lancaster-university/microbit-v2-samples/actions/workflows/build.yml) [![Docker Build Status](https://github.com/lancaster-university/microbit-v2-samples/actions/workflows/docker-image.yml/badge.svg)](https://github.com/lancaster-university/microbit-v2-samples/actions/workflows/docker-image.yml)

This repository provides the necessary tooling to compile a C/C++ CODAL program for the micro:bit V2 and generate a HEX file that can be downloaded to the device.

## Raising Issues
Any issues regarding the micro:bit are gathered on the [lancaster-university/codal-microbit-v2](https://github.com/lancaster-university/codal-microbit-v2) repository. Please raise yours there too.

# Installation
You need some open source pre-requisites to build this repo. You can either install these tools yourself, or use the docker image provided below.

- [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
- [Git](https://git-scm.com)
- [CMake](https://cmake.org/download/)
- [Python 3](https://www.python.org/downloads/)

We use Ubuntu Linux for most of our tests. You can also install these tools easily through the package manager:

```
    sudo apt install gcc
    sudo apt install git
    sudo apt install cmake
    sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi
```

## Yotta
For backwards compatibility with [microbit-samples](https://github.com/lancaster-university/microbit-samples) users, we also provide a yotta target for this repository.

## Docker
You can use the [Dockerfile](https://github.com/lancaster-university/microbit-v2-samples/blob/master/Dockerfile) provided to build the samples, or your own project sources, without installing additional dependencies.

Run the following command to build the image locally; the .bin and .hex files from a successful compile will be placed in a new `out/` directory:

```
    docker build -t microbit-tools --output out .
```

To omit the final output stage (for CI, for example) run without the `--output` arguments:

```
    docker build -t microbit-tools .
```

# Building
- Clone this repository
- In the root of this repository type `python build.py`
- The hex file will be built `MICROBIT.hex` and placed in the root folder.

# Developing
You will find a simple main.cpp in the `source` folder which you can edit. CODAL will also compile any other C/C++ header files our source files with the extension `.h .c .cpp` it finds in the source folder.

The `samples` folder contains a number of simple sample programs that utilise you may find useful.

## Developer codal.json

There is an example `coda.dev.json` file which enables "developer builds" (clones dependencies from the latest commits, instead of the commits locked in the `codal-microbit-v2` tag), and adds extra CODAL flags that enable debug data to be printed to serial.
To use it, simply copy the additional json entries into your `codal.json` file, or you can replace the file completely (`mv coda.dev.json codal.json`).

# Debugging
If you are using Visual Studio Code, there is a working debugging environment already set up for you, allowing you to set breakpoints and observe the micro:bit's memory. To get it working, follow these steps:

1. Install either [OpenOCD](http://openocd.org) or [PyOCD](https://github.com/pyocd/pyOCD).
2. Install the [`marus25.cortex-debug` VS Code extension](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug).
3. Build your program.
4. Click the Run and Debug option in the toolbar.
5. Two debugging options are provided: one for OpenOCD, and one for PyOCD. Select the correct one depending on the debugger you installed.

This should launch the debugging environment for you. To set breakpoints, you can click to the left of the line number of where you want to stop.

# Compatibility
This repository is designed to follow the principles and APIs developed for the first version of the micro:bit. We have also included a compatibility layer so that the vast majority of C/C++ programs built using [microbit-dal](https://www.github.com/lancaster-university/microbit-dal) will operate with few changes.

# Documentation
API documentation is embedded in the code using doxygen. We will produce integrated web-based documentation soon.
