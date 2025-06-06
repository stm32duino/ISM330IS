# ISM330IS
Arduino library to support the ISM330IS 3D accelerometer and 3D gyroscope

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    ISM330ISSensor AccGyr(&dev_i2c);
    AccGyr.begin();
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    ISM330ISSensor AccGyr(&dev_spi, CS_PIN);
    AccGyr.begin();	
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

The access to the sensor values is done as explained below:  

  Read accelerometer and gyroscope.

    ISM330IS_Axes_t accelerometer;
    ISM330IS_Axes_t gyroscope;
    AccGyr.Get_X_Axes(accelerometer);  
    AccGyr.Get_G_Axes(gyroscope);

## Examples

* ISM330IS_DataLog_Terminal: This application shows how to get data from ISM330IS accelerometer and gyroscope and print them on terminal.

* ISM330IS_ISPU_Sensor_Fusion: This application implements the sensor fusion of the accelerometer and gyroscope, configured in high-performance mode at 104 Hz. The configuration generates an interrupt on INT1 when the quaternion for the new sample is computed and available in the output registers.

* ISM330IS_SPU_Tap: This application implements the tap detection solution based on the accelerometer data. The configuration generates an interrupt on INT1 when the tap event for the new sample is computed and available in the output registers.
## Documentation

You can find the source files at  
https://github.com/stm32duino/ISM330IS

The ISM330IS datasheet is available at  
https://www.st.com/en/mems-and-sensors/ism330is.html