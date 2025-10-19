# BME280 Zephyr I2C Device Driver (Experimental)

This is an experimental I2C device driver for the BME280 sensor, written as a learning exercise. However, please note that Zephyr already includes a fully-featured BME280 driver in the `zephyr/drivers/sensor/bosch/bme280` directory, which I recommend using instead.
Writing this driver was an insightful experience in understanding how device drivers are constructed in Zephyr. However, this implementation has several limitations:

1. Supports only I2C, with no SPI support.
2. Lacks integration with the Zephyr driver interface.
3. Does not include a Kconfig file (though one could be added easily).
4. Does not support asynchronous operations.

For production use, rely on the official Zephyr BME280 driver. This experimental driver may still be useful for educational purposes if you're interested in learning about device driver development.
