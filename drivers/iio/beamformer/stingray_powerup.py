import os
import subprocess
import re
from time import sleep

class IIO:
	_ip='10.48.65.109'

	def write_reg(self, dev_name, reg, val):
		os.system('iio_reg -u ip:' + self._ip + ' ' + dev_name + ' ' + str(reg) + ' ' + str(val))

	def read_reg(self, dev_name, reg):
		value = os.popen('iio_reg -u ip:' + self._ip + ' ' + dev_name + ' ' + str(reg)).read()
		if value == '':
			return ''
		return int(value, 16)

	def write_dev_attr(self, dev_name, attr, val):
		os.system('iio_attr -u ip:' + self._ip + ' -d ' + dev_name + ' ' + str(attr) + ' ' + str(val))

	def read_dev_attr(self, dev_name, attr):
		value = os.popen('iio_attr -u ip:' + self._ip + ' -d ' + dev_name + ' ' + str(attr)).read()
		if value == '':
			return ''
		return int(value, 16)

	def write_dev_ch_attr(self, dev_name, channel, attr, val):
		os.system('iio_attr -u ip:' + self._ip + ' -c ' + dev_name + ' ' + str(channel) + ' ' + str(attr) + ' ' + str(val))

	def read_dev_ch_attr(self, dev_name, channel, attr, val):
		value = os.popen('iio_attr -u ip:' + self._ip + ' -c ' + dev_name + ' ' + str(channel) + ' ' + str(attr)).read()
		if value == '':
			return ''
		return int(value, 16)

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
			return 1
		val = int(value, 10)
		return val == 0

	def power_sequencer_power_good(self):
		# value = os.popen('/sys/class/gpio/ltc2992-6a-GPIO3/value').read()
		value = os.popen('grep "" ~/adar1000/gpio/ltc2992-6a-GPIO3/value').read()
		if value == '':
			return 1
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
	_dev_name='adar1000_csb_1_1'
	_POWER_DELAY = 0.2
	# _iio = 0
	# _ltc = 0
	# _gpio = 0
	_devices = list()

	def __init__(self):
		self._iio = IIO()
		self._ltc = Ltc2992()
		self._gpio = GPIOS()
		# Handles for the ADAR and ADTR chips
		# self._devices = list()

	# region Child Classes
	class Adar1000:
		_channels = 4
		def __init__(self, name):
			self._name = name

		def initialize(self, pa_off=-2.5, pa_on=-2.5, lna_off=-2, lna_on=-2):
			print('asd')	

		# region Registers
		INTERFACE_CONFIG_A_REG = 0x0000
		SCRATCHPAD_REG = 0x000A
		LD_WRK_REGS_REG = 0x0028# return self._monitor.gpio1 todo
		TX_ENABLES_REG = 0x002F
		MISC_ENABLES_REG = 0x0030
		SW_CTRL_REG = 0x0031
		ADC_CTRL_REG = 0x0032
		ADC_OUTPUT_REG = 0x0033
		BIAS_CURRENT_RX_LNA_REG = 0x0034
		BIAS_CURRENT_RX_REG = 0x0035
		BIAS_CURRENT_TX_REG = 0x0036
		BIAS_CURRENT_TX_DRV_REG = 0x0037
		MEM_CTRL_REG = 0x0038
		RX_BEAM_COMMON_REG = 0x0039
		TX_BEAM_COMMON_REG = 0x003A
		PA1_BIAS_OFF_REG = 0x0046
		PA2_BIAS_OFF_REG = 0x0047
		PA3_BIAS_OFF_REG = 0x0048
		PA4_BIAS_OFF_REG = 0x0049
		LNA_BIAS_OFF_REG = 0x004A
		TX_TO_RX_DELAY_REG = 0x004B
		RX_TO_TX_DELAY_REG = 0x004C
		TX_BEAM_STEP_START_REG = 0x004D
		TX_BEAM_STEP_STOP_REG = 0x004E
		RX_BEAM_STEP_START_REG = 0x004F
		RX_BEAM_STEP_STOP_REG = 0x0050
		RX_BIAS_RAM_CTL_REG = 0x0051
		TX_BIAS_RAM_CTL_REG = 0x0052
		# endregion


	def _pulse_power_pin(self, which):
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
			write_data = 0xA3
			self._iio.write_reg(dev_name =self._dev_name, reg=self.Adar1000.SCRATCHPAD_REG,val=write_data)
			return self._iio.read_reg(dev_name =self._dev_name, reg=self.Adar1000.SCRATCHPAD_REG) == write_data
			# self.devices[1]._write_spi(self.Adar1000.SCRATCHPAD_REG, write_data)
			# return self.devices[1]._read_spi(self.Adar1000.SCRATCHPAD_REG) == write_data
		else:
			# If Rev.B, we can directly read the sequencer's EN and PG status.
			return bool(self._ltc.power_sequencer_enable() & self._ltc.power_sequencer_power_good())

	def get_devices(self):
		# value = os.popen('grep "" /sys/bus/iio/devices/iio\:device*/label').read()
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

	# def initialize(self, pa_off=-2.5, pa_on=-2.5, lna_off=-2, lna_on=-2):
	# 	for channel in self.channels:

	def initialize_devices(self, pa_off=-2.5, pa_on=-2.5, lna_off=-2, lna_on=-2):
		""" Initialize the devices to allow for safe powerup of the board

		Args:
			pa_off (float): Voltage to set the PA_BIAS_OFF values to during initialization
			pa_on (float): Voltage to set the PA_BIAS_ON values to during initialization
			lna_off (float): Voltage to set the LNA_BIAS_OFF values to during initialization
			lna_on (float): Voltage to set the LNA_BIAS_ON values to during initialization
		"""

		for adar in self._devices:
			print('d')
			adar.initialize(pa_off, pa_on, lna_off, lna_on)

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
			self._pulse_power_pin('pwr_up_down')
		
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
		self.get_devices()
		# Initialize all the ADAR1000s
		self.initialize_devices(pa_off=pa_off, pa_on=pa_on, lna_off=lna_off, lna_on=lna_on)

stingray = Stingray()
stingray.powerup(enable_5v=False)
# stingray.write_reg(reg='0x0A',val='0xea')
# print(stingray.read_reg(reg='0x0A'))

# print(stingray.read_reg(reg=1))


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