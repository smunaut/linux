import os
import subprocess
import re
from time import sleep

class IIO:
	_ip='10.48.65.111'
	# _ip='localhost'

	def write_reg(self, dev_name, reg, val):
		value = os.popen('iio_reg -u ip:' + self._ip + ' ' + dev_name + ' ' + str(reg) + ' ' + str(val)).read()
		if value == '':
			return ''
		return int(value, 16)

	def read_reg(self, dev_name, reg):
		value = os.popen('iio_reg -u ip:' + self._ip + ' ' + dev_name + ' ' + str(reg)).read()
		if value == '':
			return ''
		return int(value, 16)

	def write_dev_attr(self, dev_name, attr, val):
		value = os.popen('iio_attr -u ip:' + self._ip + ' -d ' + dev_name + ' ' + str(attr) + ' ' + str(val)).read()
		if value == '':
			return ''
		return int(value, 10)

	def read_dev_attr(self, dev_name, attr):
		value = os.popen('iio_attr -u ip:' + self._ip + ' -d ' + dev_name + ' ' + str(attr)).read()
		if value == '':
			return ''
		return int(value, 10)

	def write_dev_ch_attr(self, dev_name, channel, attr, val):
		value = os.popen('iio_attr -u ip:' + self._ip + ' -c ' + dev_name + ' ' + str(channel) + ' ' + str(attr) + ' ' + str(val)).read()
		if value == '':
			return ''
		return int(value, 10)

	def read_dev_ch_attr(self, dev_name, channel, attr, val):
		value = os.popen('iio_attr -u ip:' + self._ip + ' -c ' + dev_name + ' ' + str(channel) + ' ' + str(attr)).read()
		if value == '':
			return ''
		return int(value, 10)

class Ltc2992:
	""" Class for Stingray board control. """
	_dev_name='ltc2992'
	_gpio=0
	def __init__(self):
		self.export_gpios()

	def export_gpios(self):
		# devices = os.popen('grep "" /sys/class/gpio/gpiochip*/device/name').read
		devices = os.popen('grep "" ~/adar1000/gpio/gpiochip*/device/name').read()
		devices = devices.split(sep="\n")
		for line in devices:
			if self._dev_name in line:
				device = line
				break

		device = device.split(sep="/")
		for line in device:
			if 'gpiochip' in line:
				device = line
				break
		device = device.replace('gpiochip', '')
		gpio = int(device, 10)
		os.system('echo ' + str(gpio + 0) + ' > /sys/class/gpio/export')
		os.system('echo ' + str(gpio + 1) + ' > /sys/class/gpio/export')
		os.system('echo ' + str(gpio + 2) + ' > /sys/class/gpio/export')
		os.system('echo ' + str(gpio + 3) + ' > /sys/class/gpio/export')
		self._gpio = gpio

	def power_sequencer_enable(self):
		# value = os.popen('/sys/class/gpio/ltc2992-6a-GPIO1/value').read()
		value = os.popen('grep "" ~/adar1000/gpio/ltc2992-6a-GPIO1/value').read()
		if value == '':
			return 0
		val = int(value, 10)
		return val == 0

	def p5v_enable(self):
		# value = os.popen('/sys/class/gpio/ltc2992-6a-GPIO2/value').read()
		value = os.popen('grep "" ~/adar1000/gpio/ltc2992-6a-GPIO2/value').read()
		if value == '':
			return 0
		val = int(value, 10)
		return val == 0

	def power_sequencer_power_good(self):
		# value = os.popen('/sys/class/gpio/ltc2992-6a-GPIO3/value').read()
		value = os.popen('grep "" ~/adar1000/gpio/ltc2992-6a-GPIO3/value').read()
		if value == '':
			return 0
		val = int(value, 10)
		return val == 0

	def p5v_power_good(self):
		# value = os.popen('/sys/class/gpio/ltc2992-6a-GPIO2/value').read()
		value = os.popen('grep "" ~/adar1000/gpio/ltc2992-6a-GPIO4/value').read()
		if value == '':
			return 0
		val = int(value, 10)
		return val == 0

class GPIOS:
	# take this values from dt stingray_control
	P5V_CTRL_PIN = 4
	PWR_UP_DOWN_PIN = 5

	def get_device(self, dev_label):
		# value = os.popen('grep "" /sys/bus/iio/devices/iio\:device*/label').read()
		devices = os.popen('grep "" ~/adar1000/iio/iio\:device*/label').read()
		devices = devices.split(sep="\n")
		for line in devices:
			if dev_label in line:
				device = line
				break

		device = device.split(sep="/")
		for line in device:
			if 'iio:device' in line:
				device = line
				break

		return device

	def gpio_pulse(self, iio, ch):

		device = self.get_device(dev_label='stingray_control')
		iio.write_dev_ch_attr(dev_name = device, channel = 'voltage' + str(ch), attr = 'raw', val = 1)
		iio.write_dev_ch_attr(dev_name = device, channel = 'voltage' + str(ch), attr = 'raw', val = 0)

class Stingray:
	""" Class for Stingray board control. """
	_revision = 'B'
	_POWER_DELAY = 0.2
	_fully_powered = False
	_devices = list()

	def __init__(self):
		self._iio = IIO()
		self._ltc = Ltc2992()
		self._gpio = GPIOS()

	class Adar1000:
		_channels = 4
		_BIAS_CODE_TO_VOLTAGE_SCALE = -0.018824
		def __init__(self, name):
			self._name = name

		def initialize(self, iio, pa_off=-2.5, pa_on=-2.5, lna_off=-2, lna_on=-2):
			dac_code = int(lna_on / self._BIAS_CODE_TO_VOLTAGE_SCALE)
			val = iio.write_dev_attr(self._name, 'lna_bias_on', dac_code)
			if val != dac_code:
				raise SystemError('Cant write device attribute')

			dac_code = int(lna_off / self._BIAS_CODE_TO_VOLTAGE_SCALE)
			val = iio.write_dev_attr(self._name, 'lna_bias_off', dac_code)
			if val != dac_code:
				raise SystemError('Cant write device attribute')

			dac_code = int(pa_on / self._BIAS_CODE_TO_VOLTAGE_SCALE)
			for i in range(self._channels):
				val = iio.write_dev_ch_attr(self._name, 'voltage' + str(i), 'pa_bias_on', dac_code)
				if val != dac_code:
					raise SystemError('Cant write device attribute')

			dac_code = int(pa_off / self._BIAS_CODE_TO_VOLTAGE_SCALE)
			for i in range(self._channels):
				val = iio.write_dev_ch_attr(self._name, 'voltage' + str(i), 'pa_bias_off', dac_code)
				if val != dac_code:
					raise SystemError('Cant write device attribute')

	def pulse_power_pin(self, which):
		if which.lower() == 'pwr_up_down':
			self._gpio.gpio_pulse(self._iio, self._gpio.PWR_UP_DOWN_PIN)
		elif which.lower() == '5v_ctrl':
			self._gpio.gpio_pulse(self._iio, self._gpio.P5V_CTRL_PIN)
		else:
			raise ValueError(f"Can't pulse the {repr(which).upper()} pin")

	def partially_powered(self):
		""" Status of the board's power tree connected to the ADM1186 """

		# If Rev.A, there's no way to check on the rails directly, we have to rely on SPI readback
		if self._revision == 'A':
			# devices = os.popen('grep "" /sys/bus/iio/devices/iio\:device*/label').read()
			devices = os.popen('grep "" ~/adar1000/iio/iio\:device*/name').read()
			devices = devices.split(sep="\n")
			for line in devices:
				if 'adar1000' in line:
					return True

			return False
		else:
			# If Rev.B, we can directly read the sequencer's EN and PG status.
			return bool(self._ltc.power_sequencer_enable() & self._ltc.power_sequencer_power_good())

	def fully_powered(self):
		""" Status of the board's full power tree """

		# If Rev.A, we have to rely on the flags
		if self._revision == 'A':
			return bool(self.partially_powered() & self._fully_powered)

		# If Rev.B, we can directly read the EN and status signals.
		else:
			return bool(self.partially_powered() & self._ltc.p5v_enable() & self._ltc.p5v_power_good())

	def get_devices(self):
		# devices = os.popen('grep "" /sys/bus/iio/devices/iio\:device*/label').read()
		devices = os.popen('grep "" ~/adar1000/iio/iio\:device*/name').read()
		devices = devices.split(sep="\n")
		for line in devices:
			if 'adar1000' in line:
				device = line
				device = device.split(sep="/")
				for line in device:
					if 'iio:device' in line:
						a = self.Adar1000(line)
						self._devices.append(a)

	def initialize_devices(self, pa_off=-2.5, pa_on=-2.5, lna_off=-2, lna_on=-2):
		""" Initialize the devices to allow for safe powerup of the board

		Args:
			pa_off (float): Voltage to set the PA_BIAS_OFF values to during initialization
			pa_on (float): Voltage to set the PA_BIAS_ON values to during initialization
			lna_off (float): Voltage to set the LNA_BIAS_OFF values to during initialization
			lna_on (float): Voltage to set the LNA_BIAS_ON values to during initialization
		"""

		for adar in self._devices:
			adar.initialize(self._iio, pa_off, pa_on, lna_off, lna_on)

	def powerup(self, enable_5v=True, **kwargs):
		""" Power up the Stingray.

		Args:
			enable_5v (bool): Boolean to set whether or not to enable the +5V rail.
		"""

		# Get any keywords that were given
		pa_off = kwargs.get('pa_off', -2.5)
		pa_on = kwargs.get('pa_on', -2.5)
		lna_off = kwargs.get('lna_off', -2)
		lna_on = kwargs.get('lna_on', -2)

		# If the board is powered down
		if not self.partially_powered():
			self.pulse_power_pin('pwr_up_down')
		
			# Wait for supplies to settle
			if self._revision == 'A':
				sleep(self._POWER_DELAY)
			else:
				loops = 0
				while not self._ltc.power_sequencer_power_good():
					sleep(0.01)
					loops += 1

					if loops > 50:
						raise SystemError("Power sequencer PG pin never went high, something's wrong")
			# todo Reload driver

			self.get_devices()
			if len(self._devices) == 0:
				raise SystemError("No ADAR1000 devices found")

			# Initialize all the ADAR1000s
			self.initialize_devices(pa_off=pa_off, pa_on=pa_on, lna_off=lna_off, lna_on=lna_on)

			if not self.partially_powered:
				raise SystemError("Board didn't power up!")

		# Send a signal to power up the +5V rail
		if enable_5v and not self.fully_powered():
			print("POWER UP: pulse_power_pin")
			# self.pulse_power_pin('5v_ctrl')

		if not enable_5v and self.fully_powered():
			print("POWER DOWN: _pulse_power_pin")
			# self.pulse_power_pin('5v_ctrl')

if __name__ == '__main__':
	stingray = Stingray()
	stingray.powerup(enable_5v=False)

# Here’s an updated Power on Sequence with aforementioned GPIO monitoring that we use in our Python code
# •	Enable +12VDC 
# •	Read AND(Power_Sequencer_Enable, Power_Sequencer_Power_Good)
# 	o	If False: Pulse the PWR_UP_DOWN signal to sequence the first RF Power Rails (gpio_pwr_up_down)
# 			ADAR1000s fully powered
# 			ADTR1107 partially powered
# •	Read AND(Power_Sequencer_Enable, Power_Sequencer_Power_Good) again
# 	o	If False: Something wrong with hardware
# •	Initialize ADAR1000 to pinch off gate of ADTR1107
# 	o	ADAR1000 Register writes to set PA_BIAS to -2.5V and LNA_BIAS to -2.0V
# •	Read AND(Power_Sequencer_Enable, Power_Sequencer_Power_Good, p5v_power_good)
# 	o	If False: Pulse +5V_CTRL_IN to Apply +5V rail (gpio_5v_ctrl)
# 			ADTR1107 fully powered
# •	Read AND(Power_Sequencer_Enable, Power_Sequencer_Power_Good, p5v_power_good) again
# 	o	If False: Something wrong with hardware