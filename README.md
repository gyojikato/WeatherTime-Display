# STM32F4_baremetal_FreeRTOS
 This repository contains all my code and experiments related to studying bare-metal embedded systems and FreeRTOS. It includes low-level peripheral drivers, startup code, and FreeRTOS tasks/examples targeting ARM Cortex-M microcontrollers. The goal is to deepen my understanding of embedded systems by working close to the hardware, without relying on high-level abstractions.

** Under development **

## 🕒 Smart Environmental Clock
---
---
## 📘 **Overview**
---
---
## 🧩 **Features**
---
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
---
## 🧠 **Implementation Details**
---
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

### Timing Marker P0
<img width="1801" height="317" alt="image" src="https://github.com/user-attachments/assets/48a8ab14-79c0-4945-bfc3-2164571b5ac2" />

<p style="text-align: justify;">
Timing Marker displays the 100ms interval for the sampling of the external RTC (DS1307 module). The scheduler from the FreeRTOS makes a great job at maintaining a fairly accurate sampling time with just an average discrepancy of around ~400us. This is crucial since the displayed time must be updated at every second. 
<img width="1341" height="278" alt="image" src="https://github.com/user-attachments/assets/aaa4fa9f-a71b-4ea5-b6b3-d13e8d865841" />

---    
## 💡 **Skills & Learnings**
---
Skills acquired and applied include, but are not limited to:
* Baremmetal C Programming
* Understanding Data Sheets
* HW and SW Debugging
* Systems Design
* Utilizing CMSIS, FreeRTOS, I2C, Timers, Interrupts
---
## 🚀 **Future Improvements**
---
---
