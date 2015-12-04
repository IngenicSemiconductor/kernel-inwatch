linux Kernel driver MMA955X, and be compatible with Android

Supported chips:
  * Freescale Digital Accelerometer,Pedometer and Motion Detect sensor
    Addresses: I2C 0x4c
    Datasheet: http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA9553L


Author: Rick Zhang <Rick.Zhang@freescale.com>

Driver Features
---------------

1. create a MMA955X input devices FreescaleAccelerometer to report sensor rase data,Step-Count    and Active-level events.

   FreescaleAccelerometer -- report Acclerometer raw data, step count and Active level

   input device event type:
   ABS_X  --  Acclerometer X-axis raw data
   ABS_Y  --  Acclerometer Y-axis raw data
   ABS_Z  --  Acclerometer Z-axis raw data

   ABS_RX --  pedometer step count.
   ABS_RY --  motion detect, active level

3. create a sysfs dir /sys/class/input/inputX/ to   configure the sensor, includes:

   poll   --  set sensor polling interval to read raw data.

