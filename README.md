# STM32F4_baremetal_FreeRTOS
 This repository contains all my code and experiments related to studying bare-metal embedded systems and FreeRTOS. It includes low-level peripheral drivers, startup code, and FreeRTOS tasks/examples targeting ARM Cortex-M microcontrollers. The goal is to deepen my understanding of embedded systems by working close to the hardware, without relying on high-level abstractions.

** Under development **

## 🕒 Smart Environmental Clock
---
---
## 📘 **Overview**
---

The Smart Environmental Clock is an embedded project that displays the current time, date, temperature, and humidity on an OLED screen. It uses a real-time clock module for accurate timekeeping and a DHT22 sensor for environmental readings. 

In addition to showing live data, the system allows users to set or adjust the current date and time. The project demonstrates a complete, self-contained embedded system with multiple sensors, a visual display, and user-interactive features, making it both a practical and educational example of bare-metal microcontroller programming.

---
## ✨ Features
---

- **Real-Time Clock**: Reads and maintains accurate time and date using the DS1307 RTC module.  
- **Temperature & Humidity Monitoring**: Periodic readings from the DHT22 sensor.  
- **OLED Display**: Visualizes time, date, and sensor data on an SH1106 display.  
- **FreeRTOS-Based Scheduling**: Uses multiple tasks for sensor acquisition, display updates, and peripheral management.  
- **Interrupt-Driven I2C Communication**: Ensures efficient, non-blocking communication with peripherals.  
- **BSP Abstraction Layer**: Board-specific drivers provide a reusable interface for sensors and displays.  
- **Thread-Safe Resource Management**: Queues and semaphores coordinate access to shared peripherals like the I2C bus.  
- **Error Handling**: Detects sensor or communication failures and maintains system stability without full resets.  
- **Modular Design**: Clean separation between HAL, BSP, and application logic for portability and maintainability.

---
## 🛠️ **Hardware Used**
---

 StM32 NUCLEO-F446RE
 DS1307 RTC Module
 DHT22 Temperature and Humidity Sensor
 SH1106 1.3" OLED Display
 
---
## ⚙️ **Software Stack**
---
### Application Layer
Handles high-level logic:
 * Retrieves time, temperature, humidity
 * Processes data
 * Triggers OLED updates
 * Manages system behavior

### Middleware
Lightweight utility modules sitting above drivers:
 * OLED rendering utilities
 * Sensor communication helpers
 * Formatting and display functions

### FreeRTOS Kernel
Provides real-time scheduling and synchronization:
 * Task Scheduler
 * Queues and Event Groups
 * Software Timers for periodic sensor/time updates
 * Mutexes for safe I2C access
<p style="text-align: justify;">
FreeRTOS manages timing for periodic reading of time, temperature, and humidity. Software timers trigger sensor acquisition tasks, while event groups or queues notify the OLED display task when new data is available. The display task uses mutex-protected I2C access through the CMSIS-based custom drivers to update the screen. This task structure ensures responsive, event-driven updates with deterministic timing.

### Custom Low-Level Driver Layer (CMSIS-based)
Custom LL drivers built directly on ARM CMSIS-Core and CMSIS-Device headers:
 * GPIO Driver using CMSIS register definitions
 * I2C Driver for sensor and OLED communication
 * Timer Driver for FreeRTOS tick and timing tasks
 * RCC/Clock Setup
 * Interrupt Handlers using NVIC CMSIS interfaces
<p style="text-align: justify;">
The low-level drivers used in this project were developed directly on top of the ARM CMSIS-Core and CMSIS-Device layers. Instead of using the STM32 HAL/LL libraries, the drivers interact with peripherals through CMSIS register definitions, providing fine-grained control and reduced overhead. Drivers implemented include GPIO, I2C, timers, UART, RCC configuration, and NVIC interrupt handlers. These drivers form the foundation of the system and are used by FreeRTOS tasks and middleware modules.


---
## 🧱 **Project Structure**
---
```text
STM32F4_baremetal_FreeRTOS/
│
├── BSP/
│   ├── Inc/
│   │   ├── DHT22.h
│   │   ├── DS1307.h
│   │   ├── fonts.h
│   │   └── SH1106.h
│   └── Src/
│       ├── DHT22.c
│       ├── DS1307.c
│       ├── fonts.c
│       └── SH1106.c
├── CMSIS/
│   ├── Device/
│   │   ├── stm32f4xx.h
│   │   ├── stm32f446xx.h
│   │   └── system_stm32f4xx.h
│   └── Include/
│       ├── core_cm4.h
│       ├── cmsis_gcc.h
│       ├── cmsis_compiler.h
│       └── ...
├── Core/
│   ├── Inc/
│   │   ├── dclk_init.h
│   │   ├── dclk_tasks.h
│   │   ├── dht22_wrappers.h
│   │   ├── ds1307_wrappers.h
│   │   ├── FreeRTOSConfig.h
│   │   ├── main.h
│   │   ├── sh1106_wrapper.h
│   │   └── stm32f4xx_it.h
│   ├── Src/
│   │   ├── dclk_init.c
│   │   ├── dclk_tasks.c
│   │   ├── dht22_wrappers.c
│   │   ├── ds1307_wrappers.c
│   │   ├── main.c
│   │   ├── sh1106_wrapper.c
│   │   ├── stm32f4xx_it.c
│   │   ├── syscalls.c
│   │   ├── sysmem.c
│   │   └── system_stm32f4xx.c
│   └── Startup/
│       └── startup_stm32f446retx.s
├── Drivers/
│   ├── Inc/
│   │   ├── stm32f44xx_gpio.h
│   │   ├── stm32f44xx_i2c.h
│   │   ├── stm32f44xx_rcc.h
│   │   ├── stm32f44xx_spi.h
│   │   ├── stm32f44xx_timer.h
│   │   └── stm32f44xx_usart.h
│   └── Src/
│       ├── stm32f44xx_gpio.c
│       ├── stm32f44xx_i2c.c
│       ├── stm32f44xx_rcc.c
│       ├── stm32f44xx_spi.c
│       ├── stm32f44xx_timer.c
│       └── stm32f44xx_usart.c
└── FreeRTOS/
    └── FreeRTOS-Kernel/
        ├── include/
        │       ├── FreeRTOS.h
        │       ├── task.h
        │       ├── queue.h
        │       ├── semphr.h
        │       ├── timers.h
        │       └── ...
        ├── portable/
        │   ├── GCC/
        │   │   └── ARM_CM4F/
        │   │       ├── port.c
        │   │       └── portmacro.h
        │   └── MemMang/
        │       └── heap_4.c
        ├── croutine.c
        ├── event_groups.c
        ├── list.c
        ├── queue.c
        ├── stream_buffer.c
        ├── tasks.c
        └── timers.c
```
---
## 🧠 Implementation Details
---

The firmware is implemented using a layered architecture that separates low-level hardware access, board-specific drivers, and application logic. Core MCU functionality is handled through CMSIS, while external devices are isolated within a BSP layer to keep hardware assumptions localized.

The BSP layer abstracts local driver implementations through wrapper functions, enabling portability across different MCU platforms.

The system runs on FreeRTOS, where functionality is decomposed into small, focused tasks. Sensor acquisition and timekeeping are performed periodically, while display updates are event-driven to avoid unnecessary CPU usage. Task execution is coordinated using lightweight RTOS primitives to ensure deterministic behavior.

All I2C-based peripherals share a single bus managed through an interrupt-driven state machine. Access to the bus is serialized using synchronization primitives to prevent concurrent transactions. I2C interrupts are enabled only during active transfers, reducing unintended ISR activity and simplifying error recovery.

Data transfer and synchronization between tasks are handled using FreeRTOS primitives. Queues are used to safely pass sensor and time data between producer and consumer tasks, while semaphores and mutexes ensure exclusive access to shared resources such as the I2C bus.

Board-specific drivers expose minimal, well-defined interfaces and do not depend on FreeRTOS APIs, allowing them to remain reusable. Error conditions such as communication timeouts and invalid sensor readings are detected at the driver level and propagated upward, enabling the system to continue operating without requiring a full reset.

---
## 🔌 Logic Analyzer Logs
---
### System Runtimme I/O Trace

<img width="1868" height="413" alt="image" src="https://github.com/user-attachments/assets/1ea28c56-94ee-4146-bd5a-9afeb01e3690" />

| Channel | Description        |
|--------|---------------------|
| **D0** | DHT22 Sampling Line |
| **D1** | I²C SDA             |
| **D2** | I²C SCL             |

### Description
<p style="text-align: justify;">
This logic-analyzer capture shows the system’s normal runtime behavior. Channel 1 displays the DHT22 sensor update pulse occurring roughly every 2 seconds. Channels 2 and 3 (I²C SDA/SCL) show large bursts of data corresponding to 
OLED display updates, while the smaller recurring I²C transactions represent periodic time/second reads performed every ~100 ms. Overall, the trace illustrates the coordinated timing between sensor polling, display refresh cycles, and    
lightweight background I²C activity.
 
### DS1307 Sampling - Timing Marker P0 (Channel 2 - D1)
<img width="1801" height="317" alt="image" src="https://github.com/user-attachments/assets/48a8ab14-79c0-4945-bfc3-2164571b5ac2" />

<p style="text-align: justify;">
Timing Marker displays the 100ms interval for the sampling of the external RTC (DS1307 module). The scheduler from the FreeRTOS maintains a fairly accurate sampling time at maintaining a fairly accurate sampling time with just an average discrepancy of around ~400us. This is crucial since the displayed time must be updated at every second. 
<img width="1341" height="278" alt="image" src="https://github.com/user-attachments/assets/aaa4fa9f-a71b-4ea5-b6b3-d13e8d865841" />

 ### DHT22 Sampling - Timing Marker P1 (Channel 1 - D0)

---    
## 💡 **Skills & Learnings**
---
Skills acquired and applied include, but are not limited to:
* Baremetal C Programing
* Understanding Data Sheets
* HW and SW Debugging
* Systems Design
* Utilizing CMSIS, FreeRTOS, I2C, Timers, Interrupts
---
## 🚀 **Future Improvements**
---

- Implement **DMA-based I2C communication** to further reduce CPU usage and improve efficiency.  
- Add **power-saving modes** to extend battery life for portable applications.  
- Expand the **user interface** to include alarms, timers, or menu navigation.  
- Integrate additional **environmental sensors** (e.g., air quality, light, or pressure).  
- Add **data logging** capabilities to store historical sensor readings.  
- Enhance **error handling and diagnostics**, such as logging I2C errors or sensor failures.  

---
