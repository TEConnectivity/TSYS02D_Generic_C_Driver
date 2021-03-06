# TSYS02D Generic C Driver
Generic C driver for the [TSYS02D sensor](http://www.te.com/usa-en/product-G-NIMO-003.html)

![tsys02d](http://www.te.com/content/dam/te-com/catalog/part/0GN/IMO/003/G-NIMO-003-t1.jpg/jcr:content/renditions/product-details.png)

The TSYS02D sensor is a self-contained temperature sensor that is  fully calibrated during manufacture. The sensor can operate from 1.5V to 3.6V.  The TSYS02D has a low power stand-by mode for power-sensitive applications.

### Specifications
*	Measures temperature from -40°C to 125°C
*	High Accuracy Temperature Sensor
*	16 bit Resolution
*	High Speed, low Response Time
*	Low Power Consumption
*	I2C Interface
*	Small TDFN8 Package 


### Driver features
* Connection test
* Reset
* Select I2C master mode
* Read serial number
* Temperature measurement


**NB:** This driver is intended to provide an implementation example of the sensor communication protocol, in order to be usable you have to implement a proper I2C layer for your target platform.
