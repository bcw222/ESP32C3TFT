from micropython import const
import time
import ustruct
import gc
from machine import SPI, Pin

class ADXL345:
  def __init__(self, spi_bus=1, cs_pin=5, scl_pin=18, sda_pin=23, sdo_pin=19, spi_freq=5000000, debug=False):
    """
    Class for fast SPI comunications between a MCU flashed with MicroPython and
    an Analog Devices ADXL345 accelerometer.  
    
    :param spi_bus: SPI bus number at which accelerometer is connected  
    :param cs_pin: MCU pin number at which accelerometer's CS wire is connected  
    :param scl_pin: MCU pin number at which accelerometer's SCL wire is connected (SCK)  
    :param sda_pin: MCU pin number at which accelerometer's SDA wire is connected (MOSI)  
    :param sdo_pin: MCU pin number at which accelerometer's SDO wire is connected (MISO)  
    :param spi_freq: frequency of SPI comunications  
    """

    # Sanity check
    if spi_freq > 5000000:
      spi_freq = 5000000
      print('Warning: Max spi clock frequency for ADXL345 is 5Mhz. Forcing to 5Mhz.')
    
    # Constants
    self.standard_g         = const(9.80665)  # m/s2
    self.read_mask          = const(0x80)
    self.multibyte_mask     = const(0x40)
    self.bytes_per_3axes    = const(6)  # 2 bytes * 3 axes
    self.device_id          = const(0xE5) # Octal 345

    # register addresses
    self.addr_device            = const(0x53)
    self.regaddr_DEVID          = const(0x00)
    self.regaddr_THRESH_TAP     = const(0x1D)
    self.regaddr_OFSX           = const(0x1E)
    self.regaddr_OFSY           = const(0x1F)
    self.regaddr_OFSZ           = const(0x20)
    self.regaddr_DUR            = const(0x21)
    self.regaddr_Latent         = const(0x22)
    self.regaddr_Window         = const(0x23)
    self.regaddr_THRESH_ACT     = const(0x24)
    self.regaddr_THRESH_INACT   = const(0x25)
    self.regaddr_TIME_INACT     = const(0x26)
    self.regaddr_ACT_INACT_CTL  = const(0x27)
    self.regaddr_THRESH_FF      = const(0x28)
    self.regaddr_TIME_FF        = const(0x29)
    self.regaddr_TAP_AXES       = const(0x2A)
    self.regaddr_ACT_TAP_STATUS = const(0x2B)
    self.regaddr_BW_RATE        = const(0x2C)
    self.regaddr_POWER_CTL      = const(0x2D)
    self.regaddr_INT_ENABLE     = const(0x2E)
    self.regaddr_INT_MAP        = const(0x2F)
    self.regaddr_INT_SOURCE     = const(0x30)
    self.regaddr_DATA_FORMAT    = const(0x31)
    self.regaddr_DATAX0         = const(0x32)
    self.regaddr_DATAX1         = const(0x33)
    self.regaddr_DATAY0         = const(0x34)
    self.regaddr_DATAY1         = const(0x35)
    self.regaddr_DATAZ0         = const(0x36)
    self.regaddr_DATAZ1         = const(0x37)
    self.regaddr_FIFO_CTL       = const(0x38)
    self.regaddr_FIFO_STATUS    = const(0x39)
    
    # SPI pins
    self.spi_bus = spi_bus
    self.cs_pin = cs_pin
    self.scl_pin = scl_pin
    self.sdo_pin = sdo_pin
    self.sda_pin = sda_pin
    self.spi_freq = spi_freq
  
  # == General Purpose ==
  def init_spi(self):
    self.spi = SPI(self.spi_bus, sck=Pin(self.scl_pin, Pin.OUT), mosi=Pin(self.sda_pin, Pin.OUT), miso=Pin(self.sdo_pin), baudrate=self.spi_freq, polarity=1, phase=1, bits=8, firstbit=SPI.MSB)
    time.sleep(0.2)
    self.cs = Pin(self.cs_pin, Pin.OUT, value=1)
    time.sleep(0.2)
    self.check_spi()
    return self

  def deinit_spi(self):
    self.spi.deinit()
    self.cs = None
    return self
  
  def __del__(self): # might not be called due to limitations of MicroPython. See https://docs.micropython.org/en/latest/genrst/core_language.html#special-method-del-not-implemented-for-user-defined-classes
    self.deinit_spi()

  def write(self, regaddr:int, the_byte:int):
    """
    Write a single byte into a register by address.  
    
    :param regaddr: register address to write  
    :param bt: byte to write  
    """
    self.cs.value(0)
    self.spi.write(bytearray((regaddr, the_byte)))
    self.cs.value(1)
    return self

  
  def read(self, regaddr: int) -> int:
    """
    read a single byte from register by address.  
    
    :param regaddr: register address to read  
    """
    return self.read_bytes(regaddr, 1)[0]
    

  def read_bytes(self, regaddr: int, nbytes: int) -> bytearray or int: # type: ignore
    """
    Read byte(s) from registers.  
    
    :param regaddr: register address to begin read  
    :param nbytes: number of bytes to read  
    :return: byte or bytes read  
    """
    wbyte = regaddr | self.read_mask
    if nbytes > 1:
      wbyte = wbyte | self.multibyte_mask
    self.cs.value(0)
    value = self.spi.read(nbytes + 1, wbyte)[1:]
    self.cs.value(1)
    return value
  
  def read_into(self, buf: bytearray, regaddr: int) -> bytearray:
    """
    Read bytes from register into an existing bytearray, generally faster than normal read.  
    
    :param rbuf: bytearray where read values will be assigned to  
    :param regaddr: register address to read  
    :return: modified input bytearray  
    """
    wbyte = regaddr | self.read_mask | self.multibyte_mask
    self.cs.value(0)
    self.spi.readinto(buf, wbyte)
    self.cs.value(1)
    return buf

  
  def remove_first_bytes_from_bytearray_of_many_transactions(self, buf:bytearray) -> bytearray:
    """
    Remove first byte of SPI transaction (which is irrelevant) from a buffer read through spi.readinto  
    
    :param buf: bytearray of size multiple of (self.bytes_per_3axes + 1)  
    :return: bytearray of size multiple of self.bytes_per_3axes  
    """
    bytes_per_3axes = self.bytes_per_3axes
    return bytearray([b for i, b in enumerate(buf) if i % (bytes_per_3axes + 1) != 0])


  # get_ methods aim to provide a simple interface to get the status of the accelerometer.
  # set_ methods aim to provide a simple interface to set the settings of the accelerometer.
  # To do that, we'll convert setting value to corresponding register value,
  # read current settings and write it back to the register.
  # To have a better control of the settings, you can just read/write the corresponding register directly.
  
  def check_spi(self) -> bool:
    """
    Read the device ID to check if SPI connection is working correctly.  
    
    :return: True if SPI connection is working correctly, False otherwise.  
    """
    try:
      is_spi_working = self.read(self.regaddr_DEVID) == self.device_id
    except:
      is_spi_working = False
    return is_spi_working
  
  def get_tap_threshold(self) -> float:
    """
    Get the threshold for tap detection.  
    
    :return: threshold value in g  
    """
    return self.read(self.regaddr_THRESH_TAP) * 0.0625
  
  def set_tap_threshold(self, threshold:int or float): # type: ignore
    """
    Set the threshold for tap detection.  
    
    :param threshold: threshold value in g  
    """
    self.write(self.regaddr_THRESH_TAP, int(threshold / 0.0625))
  
  def get_offset(self) -> tuple[float, float, float]:
    """
    Get the offset of the accelerometer.  
    
    :return: tuple of x, y, z offset in g  
    """
    return self.read(self.regaddr_OFSX) * 0.015625, self.read(self.regaddr_OFSY) * 0.015625, self.read(self.regaddr_OFSZ) * 0.015625
  
  def set_offset(self, x_offset:int or float, y_offset:int or float, z_offset:int or float): # type: ignore
    """
    Set the offset of the accelerometer.  
    
    :param x_offset: x offset in g  
    :param y_offset: y offset in g  
    :param z_offset: z offset in g  
    """
    self.write(self.regaddr_OFSX, int(x_offset / 0.015625))
    self.write(self.regaddr_OFSY, int(y_offset / 0.015625))
    self.write(self.regaddr_OFSZ, int(z_offset / 0.015625))
  
  def get_duration(self) -> int:
    """
    Get the duration of the tap detection.  
    
    :return: duration value in ms, 0 means no activity detection  
    """
    return self.read(self.regaddr_DUR) * 0.625
  
  def set_duration(self, duration:int or float): # type: ignore
    """
    Set the duration of the tap detection.  
    
    :param duration: duration value in ms, 0 means no activity detection  
    """
    self.write(self.regaddr_DUR, int(duration / 0.625))
  
  def get_latent(self) -> int:
    """
    Get the latency of the second tap detection.  
    
    :return: latency value in ms  
    """
    return self.read(self.regaddr_Latent) * 1.25
  
  def set_latent(self, latent:int or float): # type: ignore
    """
    Set the latency of the second tap detection.  
    
    :param latent: latency value in ms  
    """
    self.write(self.regaddr_Latent, int(latent / 1.25))
  
  def get_window(self) -> int:
    """
    Get the window of the second tap detection.  
    
    :return: window value in ms  
    """
    return self.read(self.regaddr_Window) * 1.25
  
  def set_window(self, window:int or float): # type: ignore
    """
    Set the window of the second tap detection.  
    
    :param window: window value in ms  
    """
    self.write(self.regaddr_Window, int(window / 1.25))
  
  def get_threshold_activity(self) -> float:
    """
    Get the threshold for activity detection.  
    
    :return: threshold value in g  
    """
    return self.read(self.regaddr_THRESH_ACT) * 0.0625
  
  def set_threshold_activity(self, threshold:int or float): # type: ignore
    """
    Set the threshold for activity detection.  
    
    :param threshold: threshold value in g  
    """
    self.write(self.regaddr_THRESH_ACT, int(threshold / 0.0625))
  
  def get_threshold_inactivity(self) -> float:
    """
    Get the threshold for inactivity detection.  
    
    :return: threshold value in g  
    """
    return self.read(self.regaddr_THRESH_INACT) * 0.0625
  
  def set_threshold_inactivity(self, threshold:int or float): # type: ignore
    """
    Set the threshold for inactivity detection.  
    
    :param threshold: threshold value in g  
    """
    self.write(self.regaddr_THRESH_INACT, int(threshold / 0.0625))
  
  def get_time_inactivity(self) -> int:
    """
    Get the time for inactivity detection.  
    
    :return: time value in s  
    """
    return self.read(self.regaddr_TIME_INACT)
  
  def set_time_inactivity(self, time:int):
    """
    Set the time for inactivity detection.  
    
    :param time: time value in s  
    """
    self.write(self.regaddr_TIME_INACT, time)
  
  def get_act_inact_ctl(self) -> bool:
    """
    Get activity and inactivity detection configuration.  
    
    :return: dict with the configuration of activity and inactivity detection.  
    :return['act_acdc']: True if activity detection is in relative mode  
    :return['act_x']: True if x axis is used for activity detection  
    :return['act_y']: True if y axis is used for activity detection  
    :return['act_z']: True if z axis is used for activity detection  
    :return['inact_acdc']: True if inactivity detection is in relative mode  
    :return['inact_x']: True if x axis is used for inactivity detection  
    :return['inact_y']: True if y axis is used for inactivity detection  
    :return['inact_z']: True if z axis is used for inactivity detection  
    """
    act_inact_ctl = self.read(self.regaddr_ACT_INACT_CTL)
    return {
      'act_acdc': bool(act_inact_ctl >> 7 & 1),
      'act_x': bool(act_inact_ctl >> 6 & 1),
      'act_y': bool(act_inact_ctl >> 5 & 1),
      'act_z': bool(act_inact_ctl >> 4 & 1),
      'inact_acdc': bool(act_inact_ctl >> 3 & 1),
      'inact_x': bool(act_inact_ctl >> 2 & 1),
      'inact_y': bool(act_inact_ctl >> 1 & 1),
      'inact_z': bool(act_inact_ctl & 1)
    }
  
  def get_activity_acdc(self) -> bool:
    """
    Get whether the activity detection threshold is in relative or absolute mode.  
    
    :return: True if relative mode, False if absolute mode  
    """
    return bool(self.read(self.regaddr_ACT_INACT_CTL) >> 7 & 1)
  
  def get_inactivity_acdc(self) -> bool:
    """
    Get whether the inactivity detection threshold is in relative or absolute mode.  
    
    :return: True if relative mode, False if absolute mode  
    """
    return bool(self.read(self.regaddr_ACT_INACT_CTL) >> 3 & 1)
  
  def get_activity_axes(self) -> tuple[bool, bool, bool]:
    """
    Get the axes used for activity detection.  
    
    :return: tuple of x, y, z axes used for activity detection  
    """
    act_inact_ctl = self.read(self.regaddr_ACT_INACT_CTL)
    return (
      bool(act_inact_ctl >> 6 & 1),
      bool(act_inact_ctl >> 5 & 1),
      bool(act_inact_ctl >> 4 & 1)
    )
  
  def get_inactivity_axes(self) -> tuple[bool, bool, bool]:
    """
    Get the axes used for inactivity detection.  
    
    :return: tuple of x, y, z axes used for inactivity detection  
    """
    act_inact_ctl = self.read(self.regaddr_ACT_INACT_CTL)
    return (
      bool(act_inact_ctl >> 2 & 1),
      bool(act_inact_ctl >> 1 & 1),
      bool(act_inact_ctl & 1)
    )
  
  def set_act_inact_ctl(self, act_acdc:bool, act_x:bool, act_y:bool, act_z:bool, inact_acdc:bool, inact_x:bool, inact_y:bool, inact_z:bool):
    """
    Set the activity and inactivity detection configuration.  
    
    :param act_acdc: True if activity detection is in relative mode  
    :param act_x: True if x axis is used for activity detection  
    :param act_y: True if y axis is used for activity detection  
    :param act_z: True if z axis is used for activity detection  
    :param inact_acdc: True if inactivity detection is in relative mode  
    :param inact_x: True if x axis is used for inactivity detection  
    :param inact_y: True if y axis is used for inactivity detection  
    :param inact_z: True if z axis is used for inactivity detection  
    """
    act_inact_ctl = (
      (int(act_acdc) << 7) |
      (int(act_x) << 6) |
      (int(act_y) << 5) |
      (int(act_z) << 4) |
      (int(inact_acdc) << 3) |
      (int(inact_x) << 2) |
      (int(inact_y) << 1) |
      int(inact_z)
    )
    return self.write(self.regaddr_ACT_INACT_CTL, act_inact_ctl)
    
  def set_activity_acdc(self, acdc:bool):
    """
    Set whether the activity detection threshold is in relative or absolute mode.  
    
    :param acdc: True if relative mode, False if absolute mode  
    """
    current_ACT_INACT_CTL = self.read(self.regaddr_ACT_INACT_CTL)
    new_ACT_INACT_CTL = (current_ACT_INACT_CTL & 0x7F) | (int(acdc) << 7)
    return self.write(self.regaddr_ACT_INACT_CTL, new_ACT_INACT_CTL)
  
  def set_inactivity_acdc(self, acdc:bool):
    """
    Set whether the inactivity detection threshold is in relative or absolute mode.  
    
    :param acdc: True if relative mode, False if absolute mode  
    """
    current_ACT_INACT_CTL = self.read(self.regaddr_ACT_INACT_CTL)
    new_ACT_INACT_CTL = (current_ACT_INACT_CTL & 0xF7) | (int(acdc) << 3)
    return self.write(self.regaddr_ACT_INACT_CTL, new_ACT_INACT_CTL)
  
  def set_activity_axes(self, x:bool, y:bool, z:bool):
    """
    Set the axes used for activity detection.  
    
    :param x: True if x axis is used for activity detection  
    :param y: True if y axis is used for activity detection  
    :param z: True if z axis is used for activity detection  
    """
    current_ACT_INACT_CTL = self.read(self.regaddr_ACT_INACT_CTL)
    new_ACT_INACT_CTL = (current_ACT_INACT_CTL & 0x3F) | (int(x) << 6) | (int(y) << 5) | (int(z) << 4)
    return self.write(self.regaddr_ACT_INACT_CTL, new_ACT_INACT_CTL)
  
  def set_inactivity_axes(self, x:bool, y:bool, z:bool):
    """
    Set the axes used for inactivity detection.  
    
    :param x: True if x axis is used for inactivity detection  
    :param y: True if y axis is used for inactivity detection  
    :param z: True if z axis is used for inactivity detection  
    """
    current_ACT_INACT_CTL = self.read(self.regaddr_ACT_INACT_CTL)
    new_ACT_INACT_CTL = (current_ACT_INACT_CTL & 0xC1) | (int(x) << 2) | (int(y) << 1) | int(z)
    return self.write(self.regaddr_ACT_INACT_CTL, new_ACT_INACT_CTL)
    
  def get_free_fall_threshold(self) -> float:
    """
    Get the threshold for free fall detection.  
    
    :return: threshold value in g  
    """
    return self.read(self.regaddr_THRESH_FF) * 0.0625
  
  def set_free_fall_threshold(self, threshold:int or float): # type: ignore
    """
    Set the threshold for free fall detection.  
    0 may result in undesirable behavior if the freefall interrupt is enabled.  
    Values between 0.3 g and 0.6 g (0x05 to 0x09) are recommended.  
    
    :param threshold: threshold value in g  
    """
    return self.write(self.regaddr_THRESH_FF, int(threshold / 0.0625))
  
  def get_free_fall_time(self) -> int:
    """
    Get the time for free fall detection.  
    
    :return: time value in ms  
    """
    return self.read(self.regaddr_TIME_FF) * 5
  
  def set_free_fall_time(self, time:int):
    """
    Set the time for free fall detection.  
    0 may result in undesirable behavior if the free-fall interrupt is enabled.  
    Values between 100 ms and 350 ms are recommended.  
    
    :param time: time value in ms  
    """
    return self.write(self.regaddr_TIME_FF, time // 5)
  
  def get_is_double_tap_suppressd_on_high_g_during_latent(self) -> bool:
    """
    Get whether the double tap detection is suppressed if acceleration greater
    than the value in THRESH_TAP is present between taps.  
    
    :return: True if suppressed, False if not  
    """
    return bool(self.read(self.regaddr_TAP_AXES) >> 3 & 1)
  
  def get_tap_axes(self) -> tuple[bool, bool, bool]:
    """
    Get the axes used for tap detection.  
    
    :return: tuple of x, y, z axes used for tap detection  
    """
    tap_axes = self.read(self.regaddr_TAP_AXES)
    return (
      bool(tap_axes >> 2 & 1),
      bool(tap_axes >> 1 & 1),
      bool(tap_axes & 1)
    )
    
  def set_is_double_tap_suppressd_on_high_g_during_latent(self, suppress:bool):
    """
    Set whether the double tap detection is suppressed if acceleration greater
    than the value in THRESH_TAP is present between taps.  
    
    :param suppress: True if suppressed, False if not  
    """
    current_TAP_AXES = self.read(self.regaddr_TAP_AXES)
    new_TAP_AXES = (current_TAP_AXES & 0xF7) | (int(suppress) << 3)
    return self.write(self.regaddr_TAP_AXES, new_TAP_AXES)
  
  def set_tap_axes(self, x:bool, y:bool, z:bool):
    """
    Set the axes used for tap detection.  
    
    :param x: True if x axis is used for tap detection  
    :param y: True if y axis is used for tap detection  
    :param z: True if z axis is used for tap detection  
    """
    tap_axes = (
      (int(x) << 2) |
      (int(y) << 1) |
      int(z)
    )
    return self.write(self.regaddr_TAP_AXES, tap_axes)
  
  def get_link_activity_inactivity(self) -> bool:
    """
    Get the link between activity and inactivity detection.  
    
    :return: True if linked, False if independent  
    """
    return bool(self.read(self.regaddr_ACT_INACT_CTL) >> 7 & 1)
  
  def get_act_tap_status(self) -> dict[bool]:
    """
    Get the status of the activity and tap detection.  
    
    :return: dict with the status of the activity and tap detection.  
    :return['act_x']: True if x axis is involved in detected activities before last read  
    :return['act_y']: True if y axis is involved in detected activities before last read  
    :return['act_z']: True if z axis is involved in detected activities before last read  
    :return['asleep']: True if this accelometer is asleep. only toggles when AUTO_SLEEP is enabled.  
    :return['tap_x']: True if x axis is involved in detected taps before last read  
    :return['tap_y']: True if y axis is involved in detected taps before last read  
    :return['tap_z']: True if z axis is involved in detected taps before last read  
    """
    act_tap_status = self.read(self.regaddr_ACT_TAP_STATUS)
    return {
      'act_x': bool(act_tap_status >> 6 & 1),
      'act_y': bool(act_tap_status >> 5 & 1),
      'act_z': bool(act_tap_status >> 4 & 1),
      'asleep': bool(act_tap_status >> 3 & 1),
      'tap_x': bool(act_tap_status >> 2 & 1),
      'tap_y': bool(act_tap_status >> 1 & 1),
      'tap_z': bool(act_tap_status & 1)
    }
  
  def clear_act_tap_status(self):
    """
    Clear the status of the activity and tap detection.  
    Useful since the ACT_TAP_STATUS register should be read before clearing the interrupt.  
    """
    self.read(self.regaddr_ACT_TAP_STATUS)
    gc.collect()
    return self
    
  def get_low_power_mode(self) -> int:
    """
    Get the current low-power mode of the accelerometer.  
    
    :return: 0 for normal mode, 1 for low-power mode  
    """
    return self.read(self.regaddr_BW_RATE) & 0x10
  
  def get_sampling_rate(self) -> int or float: # type: ignore
    """
    Get the sampling rate setting of the accelerometer.  
    Could be one of the following values:
    {0.10, 0.20, 0.39, 0.78, 1.56, 3.13, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600, 3200}  
    
    :return: sampling rate in Hz  
    """
    sampling_rates = [0.10, 0.20, 0.39, 0.78, 1.56, 3.13, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600, 3200]
    
    bw_rate = self.read(self.regaddr_BW_RATE)
    return sampling_rates[bw_rate & 0x0F]
  
  def set_low_power_mode(self, mode:int):
    """
    Set the low-power mode of the accelerometer.  
    
    :param mode: 0 for normal mode, 1 for low-power mode  
    """
    current_BW_RATE = self.read(self.regaddr_BW_RATE)
    new_BW_RATE = (current_BW_RATE & 0x0F) | (mode << 4)
    return self.write(self.regaddr_BW_RATE, new_BW_RATE)
  
  def set_sampling_rate(self, sr:int or float): # type: ignore
    """
    Set the sampling rate of the accelerometer.  
    
    :param sr: sampling rate in Hz. Should be one of the following values:
    {0.10, 0.20, 0.39, 0.78, 1.56, 3.13, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600, 3200}  
    """
    sampling_settings = {0.10: 0x00, 0.20: 0x01, 0.39: 0x02, 0.78: 0x03, 1.56: 0x04, 3.13: 0x05, 6.25: 0x06, 12.5: 0x07, 25: 0x08, 50: 0x09, 100: 0x0a, 200: 0x0b, 400: 0x0c, 800: 0x0d, 1600: 0x0e, 3200: 0x0f}
    
    current_BW_RATE = self.read(self.regaddr_BW_RATE)
    new_BW_RATE = (current_BW_RATE & 0xF0) | sampling_settings[sr]
    return self.write(self.regaddr_BW_RATE, new_BW_RATE)
  
  def get_power_ctl(self) -> dict[bool]:
    """
    Get the power configuration.  
    
    :return: dict with the configuration of the power configuration.  
    :return['link']: True if activity and inactivity detection is linked, False if independent  
    :return['auto_sleep']: True if AUTO_SLEEP is enabled. If the link mode is
    not set, the AUTO_SLEEP feature is disabled and setting the AUTO_SLEEP bit
    does not have an impact on device operation.  
    :return['measure_mode']: True if the accelerometer is in measure mode, False if in standby mode  
    :return['sleep_mode']: True if the accelerometer is set in sleep mode  
    :return['wakeup_rate']: frequency of readings in Hz. Could be one of the following values: {1, 2, 4, 8}  
    """
    power_ctl = self.read(self.regaddr_POWER_CTL)
    return {
      'link': bool(power_ctl >> 5 & 1),
      'auto_sleep': bool(power_ctl >> 4 & 1),
      'measure_mode': bool(power_ctl >> 3 & 1),
      'sleep_mode': bool(power_ctl >> 2 & 1),
      'wakeup_rate': 1 << (power_ctl & 0x03 ^ 3)
    }
    
  def set_link_activity_inactivity(self, link:bool):
    """
    Set the link between activity and inactivity detection.  
    
    :param link: True if linked, False if independent  
    """
    current_POWER_CTL = self.read(self.regaddr_POWER_CTL)
    new_POWER_CTL = (current_POWER_CTL & 0xDF) | (link << 5)
    return self.write(self.regaddr_POWER_CTL, new_POWER_CTL)
  
  def set_auto_sleep(self, enable:bool):
    """
    Set whether the accelerometer is in AUTO_SLEEP mode.  
    When clearing the AUTO_SLEEP bit, it is recommended that the part be placed
    into standby mode and then set back to measurement mode with a subsequent
    write. This is done to ensure that the device is properly biased if sleep
    mode is manually disabled; otherwise, the first few samples of data after
    the AUTO_SLEEP bit is cleared may have additional noise, especially if the
    device was asleep when the bit was cleared.  
    
    :param enable: True if AUTO_SLEEP is enabled, False if not  
    """
    current_POWER_CTL = self.read(self.regaddr_POWER_CTL)
    # use bit operation instead of calling get/set_measure_mode() since they share the same register
    auto_to_standby = not enable and bool(current_POWER_CTL >> 3 & 1)
    if auto_to_standby:
      self.write(self.regaddr_POWER_CTL, current_POWER_CTL & 0xF7) # set to standby mode
      new_POWER_CTL = (current_POWER_CTL & 0xE7) | (int(enable) << 4) # clear two bits at once to avoid unnecessary operation
    else:
      new_POWER_CTL = (current_POWER_CTL & 0xEF) | (int(enable) << 4)
    self.write(self.regaddr_ACT_INACT_CTL, new_POWER_CTL)
    if auto_to_standby:
      self.write(self.regaddr_POWER_CTL, new_POWER_CTL | 0x08) # set back to measure mode
    return self
  
  def set_measure_mode(self, enabled:bool):
    """
    Set the measure mode of the accelerometer.  
    
    :param enabled: True if the accelerometer is in measure mode, False if in standby mode  
    """
    current_POWER_CTL = self.read(self.regaddr_POWER_CTL)
    new_POWER_CTL = (current_POWER_CTL & 0x0F) | (int(enabled) << 3)
    return self.write(self.regaddr_POWER_CTL, new_POWER_CTL)
  
  def set_sleep_mode(self, enabled:bool):
    """
    Set whether the accelerometer is set in sleep mode.  
    When clearing the sleep bit, it is recommended that the part be placed into
    standby mode and then set back to measurement mode with a subsequent write.
    This is done to ensure that the device is properly biased if sleep mode is
    manually disabled; otherwise, the first few samples of data after the sleep
    bit is cleared may have additional noise, especially if the device was asleep
    when the bit was cleared.  
    
    :param enabled: True if the accelerometer is in sleep mode  
    """
    current_POWER_CTL = self.read(self.regaddr_POWER_CTL)
    # use bit operation instead of calling get/set_measure_mode() since they share the same register
    auto_to_standby = not enabled and bool(current_POWER_CTL >> 3 & 1)
    if auto_to_standby:
      self.write(self.regaddr_POWER_CTL, current_POWER_CTL & 0xF7) # set to standby mode
      new_POWER_CTL = (current_POWER_CTL & 0xF3) | (int(enabled) << 2) # clear two bits at once to avoid unnecessary operation
    else:
      new_POWER_CTL = (current_POWER_CTL & 0xFB) | (int(enabled) << 2)
    self.write(self.regaddr_ACT_INACT_CTL, new_POWER_CTL)
    if auto_to_standby:
      self.write(self.regaddr_POWER_CTL, new_POWER_CTL | 0x08) # set back to measure mode
    return self
  
  def set_wakeup_rate(self, rate:int):
    """
    Set the the frequency of readings in sleep mode.  
    
    :param rate: frequency of readings in Hz. Should be one of the following
    values: {1, 2, 4, 8}  
    """
    wakeup_rates = {1: 0x00, 2: 0x01, 4: 0x02, 8: 0x03}
    
    current_POWER_CTL = self.read(self.regaddr_POWER_CTL)
    new_POWER_CTL = (current_POWER_CTL & 0xFC) | wakeup_rates[rate]
    return self.write(self.regaddr_POWER_CTL, new_POWER_CTL)
  
  def get_int_enable(self) -> int:
    """
    Get the interrupt enable register.  
    The DATA_READY, watermark, and overrun bits enable only the interrupt output;
    the functions are always enabled.  
    It is recommended that interrupts be configured before enabling their outputs.  
    
    :return: dict with the enable configuration of each interrupt.  
    :return['data_ready']: True if data_ready interrupt is enabled  
    :return['single_tap']: True if single_tap interrupt is enabled  
    :return['double_tap']: True if double_tap interrupt is enabled  
    :return['activity']: True if activity interrupt is enabled  
    :return['inactivity']: True if inactivity interrupt is enabled  
    :return['free_fall']: True if free_fall interrupt is enabled  
    :return['watermark']: True if watermark interrupt is enabled  
    :return['overrun']: True if overrun interrupt is enabled  
    """
    int_enable = self.read(self.regaddr_INT_ENABLE)
    return {
      'data_ready': bool(int_enable >> 7 & 1),
      'single_tap': bool(int_enable >> 6 & 1),
      'double_tap': bool(int_enable >> 5 & 1),
      'activity': bool(int_enable >> 4 & 1),
      'inactivity': bool(int_enable >> 3 & 1),
      'free_fall': bool(int_enable >> 2 & 1),
      'watermark': bool(int_enable >> 1 & 1),
      'overrun': bool(int_enable & 1)
    }
  
  def set_int_enable(self, data_ready:bool=False, single_tap:bool=False, double_tap:bool=False, activity:bool=False, inactivity:bool=False, free_fall:bool=False, watermark:bool=False, overrun:bool=False):
    """
    Set the interrupt enable register.  
    The DATA_READY, watermark, and overrun bits enable only the interrupt output;
    the functions are always enabled.  
    It is recommended that interrupts be configured before enabling their outputs.  
    
    :param data_ready: True if data_ready interrupt is enabled  
    :param single_tap: True if single_tap interrupt is enabled  
    :param double_tap: True if double_tap interrupt is enabled  
    :param activity: True if activity interrupt is enabled  
    :param inactivity: True if inactivity interrupt is enabled  
    :param free_fall: True if free_fall interrupt is enabled  
    :param watermark: True if watermark interrupt is enabled  
    :param overrun: True if overrun interrupt is enabled  
    """
    int_enable = (
      (data_ready << 7) |
      (single_tap << 6) |
      (double_tap << 5) |
      (activity << 4) |
      (inactivity << 3) |
      (free_fall << 2) |
      (watermark << 1) |
      overrun
    )
    return self.write(self.regaddr_INT_ENABLE, int_enable)
  
  def get_int_map(self) -> int:
    """
    Get the interrupt mapping.  
    
    :return: dict with the mapping of each interrupt to the INT1 or INT2 pin.  
    :return['data_ready']: False if data_ready interrupt is mapped to INT1 pin  
    :return['single_tap']: False if single_tap interrupt is mapped to INT1 pin  
    :return['double_tap']: False if double_tap interrupt is mapped to INT1 pin  
    :return['activity']: False if activity interrupt is mapped to INT1 pin  
    :return['inactivity']: False if inactivity interrupt is mapped to INT1 pin  
    :return['free_fall']: False if free_fall interrupt is mapped to INT1 pin  
    :return['watermark']: False if watermark interrupt is mapped to INT1 pin  
    :return['overrun']: False if overrun interrupt is mapped to INT1 pin  
    """
    int_map = self.read(self.regaddr_INT_MAP)
    return {
      'data_ready': bool(int_map >> 7 & 1),
      'single_tap': bool(int_map >> 6 & 1),
      'double_tap': bool(int_map >> 5 & 1),
      'activity': bool(int_map >> 4 & 1),
      'inactivity': bool(int_map >> 3 & 1),
      'free_fall': bool(int_map >> 2 & 1),
      'watermark': bool(int_map >> 1 & 1),
      'overrun': bool(int_map & 1)
    }
  
  def set_int_map(self, data_ready:bool=False, single_tap:bool=False, double_tap:bool=False, activity:bool=False, inactivity:bool=False, free_fall:bool=False, watermark:bool=False, overrun:bool=False):
    """
    Set the interrupt mapping. Set the bit to False to map corresponding
    interrupt on INT1 pin, True to INT2 pin.  
    
    :param data_ready: False if data_ready interrupt is mapped to INT1 pin  
    :param single_tap: False if single_tap interrupt is mapped to INT1 pin  
    :param double_tap: False if double_tap interrupt is mapped to INT1 pin  
    :param activity: False if activity interrupt is mapped to INT1 pin  
    :param inactivity: False if inactivity interrupt is mapped to INT1 pin  
    :param free_fall: False if free_fall interrupt is mapped to INT1 pin  
    :param watermark: False if watermark interrupt is mapped to INT1 pin  
    :param overrun: False if overrun interrupt is mapped to INT1 pin  
    """
    int_map = (
      (data_ready << 7) |
      (single_tap << 6) |
      (double_tap << 5) |
      (activity << 4) |
      (inactivity << 3) |
      (free_fall << 2) |
      (watermark << 1) |
      overrun
    )
    return self.write(self.regaddr_INT_MAP, int_map)
  
  def get_int_source(self) -> int:
    """
    Get interrupts from SPI.  
    
    :return: dict with the status of each interrupt.  
    :return['data_ready']: True if a new measure is available  
    :return['single_tap']: True if a single tap is detected  
    :return['double_tap']: True if a double tap is detected  
    :return['activity']: True if the activity detection is triggered  
    :return['inactivity']: True if the inactivity detection is triggered  
    :return['free_fall']: True if a free fall is detected  
    :return['watermark']: True if the watermark level is reached  
    :return['overrun']: True if the fifo overrun is detected  
    """
    int_status = self.read(self.regaddr_INT_SOURCE)
    return {
      'data_ready': bool(int_status >> 7 & 1),
      'single_tap': bool(int_status >> 6 & 1),
      'double_tap': bool(int_status >> 5 & 1),
      'activity': bool(int_status >> 4 & 1),
      'inactivity': bool(int_status >> 3 & 1),
      'free_fall': bool(int_status >> 2 & 1),
      'watermark': bool(int_status >> 1 & 1),
      'overrun': bool(int_status & 1)
    }
  
  def clear_int_source(self):
    """
    Clear interrupts from SPI. Might need to be called multiple times to clear all interrupts.
    Useful to initialize the status after a interrupt has been handled.  
    """
    self.read(self.regaddr_INT_SOURCE)
    gc.collect()
    return self
  
  def get_data_format(self) -> int:
    """
    Get the data format configuration.  
    
    :return: dict with the data format configuration.  
    :return['self_test']: True if self test is enabled
    :return['spi_interface']: True if SPI interface is 3-wire
    :return['int_invert']: True if interrupt pins are active low
    :return['full_res']: True if device is in 13-bit resolution mode, False if in 10-bit resolution mode
    :return['justify']: True if data is left-justified. May result in wrong values from built-in coversation.
    :return['range']: the scale of output acceleration data
    """
    data_format = self.read(self.regaddr_DATA_FORMAT)
    return {
      'self_test': bool(data_format >> 7 & 1),
      'spi_interface': bool(data_format >> 6 & 1),
      'int_invert': bool(data_format >> 5 & 1),
      'full_res': bool(data_format >> 3 & 1),
      'justify': bool(data_format >> 2 & 1),
      'range': 1 << (data_format & 0x03 + 1)
    }
  
  def get_g_range(self) -> int:
    """
    Get the scale of output acceleration data.  
    
    :return: the scale of output acceleration data. One of {2, 4, 8, 16}  
    """
    return 1 << (self.read(self.regaddr_DATA_FORMAT) & 0x03 + 1)
  
  def set_self_test(self, enabled:bool):
    """
    Set whether the accelerometer is in self-test mode.  
    
    :param enabled: True if self-test is enabled, False if not  
    """
    current_DATA_FORMAT = self.read(self.regaddr_DATA_FORMAT)
    new_DATA_FORMAT = (current_DATA_FORMAT & 0x7F) | (int(enabled) << 7)
    return self.write(self.regaddr_DATA_FORMAT, new_DATA_FORMAT)
  
  def set_spi_interface(self, three_wire:bool):
    """
    Set whether the accelerometer is in SPI 3-wire mode.  
    
    :param three_wire: True if SPI interface is 3-wire, False if 4-wire  
    """
    current_DATA_FORMAT = self.read(self.regaddr_DATA_FORMAT)
    new_DATA_FORMAT = (current_DATA_FORMAT & 0xBF) | (int(three_wire) << 6)
    return self.write(self.regaddr_DATA_FORMAT, new_DATA_FORMAT)
  
  def set_int_invert(self, active_low:bool):
    """
    Set whether the interrupt pins are active low or active high.  
    
    :param active_low: True if interrupt pins are active low, False if active high  
    """
    current_DATA_FORMAT = self.read(self.regaddr_DATA_FORMAT)
    new_DATA_FORMAT = (current_DATA_FORMAT & 0xDF) | (int(active_low) << 5)
    return self.write(self.regaddr_DATA_FORMAT, new_DATA_FORMAT)
  
  def set_full_res(self, full_res:bool):
    """
    Set whether the accelerometer is in 13-bit or 10-bit resolution mode.  
    
    :param full_res: True if device is in 13-bit resolution mode, False if in 10-bit resolution mode  
    """
    current_DATA_FORMAT = self.read(self.regaddr_DATA_FORMAT)
    new_DATA_FORMAT = (current_DATA_FORMAT & 0xF7) | (int(full_res) << 3)
    return self.write(self.regaddr_DATA_FORMAT, new_DATA_FORMAT)
  
  def set_justify(self, left_justify:bool):
    """
    Set whether the data is right-justified or left-justified.  
    
    :param left_justify: True if data is left-justified. May result in wrong values from built-in coversation.  
    """
    current_DATA_FORMAT = self.read(self.regaddr_DATA_FORMAT)
    new_DATA_FORMAT = (current_DATA_FORMAT & 0xFB) | (int(left_justify) << 2)
    return self.write(self.regaddr_DATA_FORMAT, new_DATA_FORMAT)
  
  def set_g_range(self, grange:int):
    """
    Set the scale of output acceleration data.  
    
    :param grange: {2, 4, 8, 16}
    """
    granges = {2: 0x00, 4: 0x01, 8: 0x02, 16: 0x03}
    if grange not in granges:
      raise ValueError(f'Invalid range value. Should be one of: {granges}')
    
    current_DATA_FORMAT = self.read(self.regaddr_DATA_FORMAT)
    new_DATA_FORMAT = (current_DATA_FORMAT & 0xFC) | granges[grange]
    return self.write(self.regaddr_DATA_FORMAT, new_DATA_FORMAT)
  
  def get_data(self) -> bytearray:
    """
    Get raw acceleration data.  
    """
    return self.read_bytes(self.regaddr_DATAX0, self.bytes_per_3axes)
  
  def get_fifo_ctl(self) -> dict[int, bool]:
    """
    Get the FIFO status.  
    
    :return: dict with the FIFO configuration and sample count.  
    :return['fifo_mode']: mode of
      FIFO operation:
      
      - 0 (Bypass): FIFO is bypassed  
      - 1 (FIFO): FIFO collects up to 32 values and then stops collecting
       data, collecting new data only when FIFO is not full  
      - 2 (Stream): FIFO holds the last 32 data values. When FIFO is full,
        the oldest data is overwritten with newer data  
      - 3 (Trigger): When triggered by the trigger bit, FIFO holds the last
        data samples before the trigger event and then continues to collect
        data until full. New data is collected only when FIFO is not full  
    :return['trigger']: True if the trigger pin is set to INT2
    :return['samples']: the number of samples
      based on the FIFO mode:
      
      - For Bypass mode: None
      - For FIFO and Stream modes: Specifies how many FIFO entries are needed
        to trigger a watermark interrupt
      - For Trigger mode: Specifies how many FIFO samples are retained in the
        FIFO buffer before a trigger event
    """
    fifo_ctl = self.read(self.regaddr_FIFO_CTL)
    return {
      'fifo_mode': fifo_ctl >> 6 & 0x03,
      'trigger': bool(fifo_ctl >> 5 & 1),
      'samples': fifo_ctl & 0x1F
    }
  
  def set_fifo_ctl(self, fifo_mode:int, trigger:bool=False, samples:int=32):
    """
    Set the FIFO configuration.

    :param fifo_mode: mode of
      FIFO operation:
      
      - 0 (Bypass): FIFO is bypassed  
      - 1 (FIFO): FIFO collects up to 32 values and then stops collecting
       data, collecting new data only when FIFO is not full  
      - 2 (Stream): FIFO holds the last 32 data values. When FIFO is full,
        the oldest data is overwritten with newer data  
      - 3 (Trigger): When triggered by the trigger bit, FIFO holds the last
        data samples before the trigger event and then continues to collect
        data until full. New data is collected only when FIFO is not full  
    :param trigger: True if the trigger pin is set to INT2  
    :param samples: the number of samples
      based on the FIFO mode:
      
      - For Bypass mode: None
      - For FIFO and Stream modes: Specifies how many FIFO entries are needed
        to trigger a watermark interrupt
      - For Trigger mode: Specifies how many FIFO samples are retained in the
        FIFO buffer before a trigger event
    """
    fifo_ctl = ((fifo_mode & 0x03) << 6) | ((int(trigger) & 1) << 5) | (samples & 0x1F)
    return self.write(self.regaddr_FIFO_CTL, fifo_ctl)
  

  def set_fifo_mode(self, mode:int):
    """
    Set the FIFO mode.  
    
    :param mode: mode of
      FIFO operation:
      
      - 0 (Bypass): FIFO is bypassed  
      - 1 (FIFO): FIFO collects up to 32 values and then stops collecting
       data, collecting new data only when FIFO is not full  
      - 2 (Stream): FIFO holds the last 32 data values. When FIFO is full,
        the oldest data is overwritten with newer data  
      - 3 (Trigger): When triggered by the trigger bit, FIFO holds the last
        data samples before the trigger event and then continues to collect
        data until full. New data is collected only when FIFO is not full  
    """
    current_FIFO_CTL = self.read(self.regaddr_FIFO_CTL)
    new_FIFO_CTL = (current_FIFO_CTL & 0x3F) | ((mode & 0x03) << 6)
    return self.write(self.regaddr_FIFO_CTL, new_FIFO_CTL)
  
  def set_fifo_trigger(self, INT2:bool):
    """
    Set whether the FIFO trigger pin is INT1 or INT2.  
    
    :param INT2: True if the trigger pin is set to INT2, False if INT1  
    """
    current_FIFO_CTL = self.read(self.regaddr_FIFO_CTL)
    new_FIFO_CTL = (current_FIFO_CTL & 0xDF) | (int(INT2) << 5)
    return self.write(self.regaddr_FIFO_CTL, new_FIFO_CTL)
  
  def set_fifo_samples(self, samples:int):
    """
    Set the number of FIFO samples.
    
    :param samples: the number of samples
      based on the FIFO mode:
      
      - For Bypass mode: None
      - For FIFO and Stream modes: Specifies how many FIFO entries are needed
        to trigger a watermark interrupt
      - For Trigger mode: Specifies how many FIFO samples are retained in the
        FIFO buffer before a trigger event
    """
    current_FIFO_CTL = self.read(self.regaddr_FIFO_CTL)
    new_FIFO_CTL = (current_FIFO_CTL & 0xE0) | (samples & 0x1F)
    return self.write(self.regaddr_FIFO_CTL, new_FIFO_CTL)

  def clear_fifo(self):
    """
    Clears all values in fifo by setting the bypass mode and then the stream mode again.  
    Useful to start reading FIFO when expected, otherwise the first values were
    recorded before actually starting the measurement.  
    """
    # use bit opration instead of get_fifo_ctl() and set_fifo_mode() to avoid unnecessary read/write
    current_FIFO_CTL = self.read(self.regaddr_FIFO_CTL)
    self.write(self.regaddr_FIFO_CTL, current_FIFO_CTL & 0xCF)
    return self.write(self.regaddr_FIFO_CTL, current_FIFO_CTL)
  
  def get_fifo_status(self) -> dict[int, bool]:
    """
    Get the FIFO status.  
    
    :return: dict with the FIFO status.  
    :return['fifo_trig']: True if a FIFO trigger event has occurred  
    :return['samples']: the number of samples in the FIFO buffer  
    """
    fifo_status = self.read(self.regaddr_FIFO_STATUS)
    return {
      'fifo_trig': bool(fifo_status >> 7 & 1),
      'samples': fifo_status & 0x3F
    }


  # == Continous reading able to reach 3.2 kHz ==
  
  def read_many_xyz(self, n: int=1) -> tuple:
    """
    :param n: number of xyz accelerations to read from the accelerometer
    return: (
        bytearray containing 2 bytes for each of the 3 axes multiplied by the fractions of the sampling rate contained in the acquisition time,
        array of times at which each sample was recorded in microseconds
    )
    """
    
    # local variables and functions are MUCH faster, so copy them inside the function
    regaddr_data = self.regaddr_DATAX0 | self.read_mask | self.multibyte_mask
    regaddr_intsource = self.regaddr_INT_SOURCE | self.read_mask
    spi_readinto = self.spi.readinto
    cs = self.cs
    ticks_us = time.ticks_us
    bytes_per_3axes = self.bytes_per_3axes
    read_bytes = self.read_bytes
    
    # definitions
    n_exp_meas = n
    n_exp_bytes = (self.bytes_per_3axes + 1) * n_exp_meas
    times = [0] * (int(n_exp_meas * 1.5))
    buf = bytearray(int(n_exp_bytes * 1.5))
    m = memoryview(buf)
    n_act_meas = 0
    
    # set up device
    last_not_measure_mode = not self.get_power_ctl()['measure_mode']
    self.set_fifo_mode(0)
    if last_not_measure_mode:
      self.set_measure_mode(True)
    gc.collect()
    
    # measure
    while n_act_meas < n_exp_meas:
      start_index = n_act_meas * (bytes_per_3axes + 1)
      stop_index = n_act_meas * (bytes_per_3axes + 1) + (bytes_per_3axes + 1)
      cs.value(0)
      is_data_ready = read_bytes(regaddr_intsource, 2)[1] >> 7 & 1
      cs.value(1)
      if not is_data_ready:
        continue
      cs.value(0)
      spi_readinto(m[start_index: stop_index], regaddr_data)
      cs.value(1)
      times[n_act_meas] = ticks_us()
      n_act_meas += 1
    if last_not_measure_mode:
      self.set_measure_mode(False)
    
    # tail data cleaning
    buf = self.remove_first_bytes_from_bytearray_of_many_transactions(buf)
    buf = buf[:n_exp_meas * bytes_per_3axes]  # remove exceeding values
    times = times[:n_exp_meas]  # remove exceeding values

    gc.collect()
    return buf, times

  
  def read_many_xyz_fromfifo(self, n: int = 1) -> tuple:
    """
    read many measures of accaleration on the 3 axes from the fifo register
    :param n: number of measures to read (xyz counts 1)
    return: (
        bytearray containing 2 bytes for each of the 3 axes multiplied by the fractions of the sampling rate contained in the acquisition time,
        array of times at which each sample was recorded in microseconds
    )
    """
    
    # local variables and functions are MUCH faster so copy them inside the function
    regaddr_data = self.regaddr_DATAX0 | self.read_mask | self.multibyte_mask
    spi_readinto = self.spi.readinto
    cs = self.cs
    get_fifo_status = self.get_fifo_status
    bytes_per_3axes = self.bytes_per_3axes
    
    # definitions
    n_exp_meas = n
    n_exp_bytes = (bytes_per_3axes + 1) * n_exp_meas
    buf = bytearray(int(n_exp_bytes * 1.5))
    m = memoryview(buf)
    n_act_meas = 0
    
    # set up device
    last_not_measure_mode = not self.get_power_ctl()['measure_mode']
    self.set_fifo_mode(1)
    if last_not_measure_mode:
      self.set_measure_mode(True)
    self.clear_fifo()
    gc.collect()
    
    # measure
    t_start = time.ticks_us()
    while n_act_meas < n_exp_meas:
      samples_infifo = get_fifo_status()['samples']
      for _ in range(samples_infifo):  # it is impossible to read a block of measures from fifo
        cs.value(0)
        spi_readinto(m[n_act_meas * (bytes_per_3axes + 1): n_act_meas * (bytes_per_3axes + 1) + (bytes_per_3axes + 1)], regaddr_data)
        cs.value(1)
        n_act_meas += 1
    t_stop = time.ticks_us()
    if last_not_measure_mode:
      self.set_measure_mode(False)
    
    # tail data cleaning
    buf = self.remove_first_bytes_from_bytearray_of_many_transactions(buf)
    buf = buf[:n_exp_meas * bytes_per_3axes]  # remove exceeding values
    actual_acq_time = (t_stop - t_start) / 1000000
    actual_sampling_rate = n_act_meas / actual_acq_time
    times = [(i+1) / actual_sampling_rate for i in range(n_exp_meas)]

    gc.collect()
    return buf, times

  
  def read_continuos_xyz(self, acquisition_time:int) -> tuple:
    """
    Read for the provided amount of time from the acceleration register, saving
    the value only if a new measure is available since last reading.
    
    :param acquisition_time: seconds the acquisition should last
    :return: (
        bytearray containing 2 bytes for each of the 3 axes multiplied by the fractions of the sampling rate contained in the acquisition time,
        array of times at which each sample was recorded in microseconds
    )
    """
    n_exp_meas = int(acquisition_time * self.get_sampling_rate())
    buf, times = self.read_many_xyz(n_exp_meas)
    return buf, times

  
  def read_continuos_xyz_fromfifo(self, acquisition_time: int) -> tuple:
    """
    Read for the provided amount of time all the values contained in the fifo register (if any).
    
    :param acquisition_time:
    :return: (
        bytearray containing 2 bytes for each of the 3 axes multiplied by the fractions of the sampling rate contained in the acquisition time,
        array of times at which each sample was recorded in microseconds
    )
    """
    n_exp_meas = int(acquisition_time * self.get_sampling_rate())
    buf, times = self.read_many_xyz_fromfifo(n_exp_meas)
    return buf, times


  # == Conversion utitlities ==
  def xyzbytes2g(self, buf: bytearray) -> tuple:
    """
    convert a bytearray of measures on the three axes xyz in three lists where the acceleration is in units of
        gravity on the sealevel (g)
    :param buf: bytearray of 2 bytes * 3 axes * nvalues
    :return: 3 lists of ints corresponding to x, y, z values of acceleration in units of g
    """
    gc.collect()
    acc_x, acc_y, acc_z = zip(*[ustruct.unpack('<HHH', buf[i:i + self.bytes_per_3axes]) for i in range(0,len(buf),self.bytes_per_3axes)])
    # negative values rule
    acc_x = [x if x <= 32767 else x - 65536 for x in acc_x]
    acc_y = [y if y <= 32767 else y - 65536 for y in acc_y]
    acc_z = [z if z <= 32767 else z - 65536 for z in acc_z]
    gc.collect()
    return acc_x, acc_y, acc_z
