# STM32 Sensor Interface & Binary Protocol

A high-performance C implementation for **STM32F401RE** (Nucleo) to interface with **MPU-6050** accelerometer. Focuses on non-blocking data acquisition and reliable serial transmission.

## 🛠 Features
- **Non-blocking I2C**: Uses **DMA (Direct Memory Access)** to acquire sensor data without loading the CPU.
- **Custom Binary Protocol**: Implemented frame synchronization with **Byte-stuffing** and **CRC-32** for integrity.
- **Efficiency**: Utilizes **Hardware Timers** for precise sampling and **Ring Buffers** to handle high-speed USART streams.
- **Architecture**: State-machine based parser for asynchronous data processing.

## 🔧 Tech Stack
- **Language**: C
- **Hardware**: STM32F401RE (ARM Cortex-M4)
- **Peripherals**: I2C, USART, DMA, TIM
- **IDE**: STM32CubeIDE / HAL Library