This driver provides a complete interface for the NAU7802 24-bit ADC, commonly used with load cells and other precision analogue sensors. It is built for the ESP32/ESP-IDF platform using the legacy I²C driver (driver/i2c.h), though it can be adapted to the newer I²C master driver if needed.



This library was inspired by the SparkFun NAU7802 Qwiic Arduino library and was created for my project, where I used an ESP32-S3 with a 4.3" LCD controller, integrated as an ESP-IDF component in VS Code.



### **Adding the umar\_nau7802\_adc ESP-IDF Component to Your Project**



This library can be easily added to your ESP-IDF project (VS Code) as a component. Follow these steps carefully:



Step 1: Open your ESP-IDF project in VS Code. Press F1 to open the command palette. select “ESP-IDF: Open ESP-IDF Terminal”. This opens a new terminal inside VS Code, usually at the root folder of your project.



Step 2: In the terminal, type the following command:



**git clone https://github.com/umarf0450/umar-nau7802-adc.git umar\_nau7802\_adc**



Press Enter. This will clone the library directly into the root folder of your project. Move the entire folder umar-nau7802-adc into your project’s **components** folder.





### **Details**

The library automatically detects the NAU7802 on the I²C bus and supports software reset to initialise the device. It allows enabling or disabling the internal voltage regulator (LDO) and configuring the PGA LDO mode, with proper ramp timing for stable readings. The programmable gain amplifier (PGA) and ADC sample rates can be configured, and the library automatically recalibrates to maintain accurate measurements.



It supports switching between input channels and accessing the internal temperature sensor for calibration or monitoring. Functions are provided for internal offset calibration, system offset and gain adjustments, and converting raw ADC readings into scaled weight units. Helper functions are available for calculating zero offsets, calibration factors, and averaging multiple samples for stable readings.



Advanced users have full access to NAU7802 registers and individual bits for fine-tuning or extending functionality. Non-blocking reads with configurable timeouts and task delays allow smooth operation in multitasking environments.



I have also included example files demonstrating basic readings, calculating the calibration factor, and averaging samples, etc. In most cases, simply calling nau7802\_start to initialize the ADC and nau7802\_get\_reading to read the values is sufficient for getting started. These examples can be used as a reference to quickly integrate the NAU7802 into your project.

