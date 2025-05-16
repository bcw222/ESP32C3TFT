# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()

# Init WLAN device
import network
wlan = network.WLAN(network.STA_IF)
wlan.active(False)
from wlan_cfg import wlan_cfg


# Init BLE device
from ubluetooth import BLE
ble = BLE()
ble.active(False)


from lib.ADXL345_spi import ADXL345
import machine

# Make sure the display cs is not enabled
display_cs = machine.Pin(10, machine.Pin.OUT)
display_cs.value(1)

# Init ADXL345 as acc
int1 = machine.Pin(3, machine.Pin.IN)
acc = ADXL345(cs_pin=8, scl_pin=6, sda_pin=2, sdo_pin=7, spi_freq=5_000_000).init_spi()

def clear_acc_int(int_pin):
    while int_pin.value():
        acc.clear_fifo()  # Clear the interrupt with less reads
        print(acc.get_int_source())

# Set up acc
acc.set_sampling_rate(6.25)
acc.set_g_range(2)
acc.set_int_enable()
acc.set_threshold_activity(0.75)  # Not used yet
acc.set_threshold_inactivity(0.5)
acc.set_act_inact_ctl(*((True,)*8))
acc.set_time_inactivity(255)
acc.set_int_enable(inactivity=True)
acc.set_measure_mode(True)

clear_acc_int(int1)

acc.deinit_spi() # Release acc.spi

# Run main
from main import main
display_spi = machine.SPI(1, baudrate=40_000_000, sck=machine.Pin(6), mosi=machine.Pin(2), miso=machine.Pin(7))
main(display_spi, display_cs, int1, wlan, ble, wlan_cfg)
display_spi.deinit()
display_cs.value(1)

acc.init_spi() # Reinit acc.spi for mode change
acc.set_measure_mode(False)
acc.deinit_spi() # Release acc.spi

machine.deepsleep()
