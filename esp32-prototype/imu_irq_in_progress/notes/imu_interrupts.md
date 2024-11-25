# Register 56 - Interrupt Enable
- Clear bits [7:1], set bit 0 (RAW_RDY_EN)
- also configure register 36 - I2C_MST_CTRL, bit[6] WAIT_FOR_ES
    - We should clear bit[6] WAIT_FOR_ES, we don't want to wait for the external sensor to load data before triggering an interrupt (we have no external sensor connected)

# In IDF
For now just adding to `mpu9250.h` and `mpu9250.c`. I think it can be similar to the lora interrupt, but simplier. The interrupt is only triggered when raw sensor data is ready.
- We should measure how frequently this occurs to get an idea if we want to run it at a lower data rate by polling/scheduling
- So far I have the outline for interrupt enable, but still need to modify interrupt config (registers 55 and 58, see the [register map](../../../datasheets/mpu9250_register_map.pdf))
- Need to add the isr function which should just queue up a task to run get_accel_gyro_mag to read data over i2c (and potentially clear interrupt status)
    - We can create a task that is similar to imu_task, but modify imu_run function to use interrupts instead of pausing. So it should block on some queue defined in main i guess.
    - This motivates moving the initialization of the interrupt into a separate file. This will probably be my final approach, then we can trim unecessary library code from the mpu components.
