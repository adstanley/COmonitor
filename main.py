"""
MQ135/DHT22 example for MicroPython 
"""

from machine import Pin, ADC, UART
from dht import DHT22

import time
import math

class MQ135():
    """ Class for dealing with MQ135 Gas Sensors with improved accuracy """
    # Constants for MQ135 sensor
    # Load resistance in kOhms
    # This is the resistance of the load resistor used in the voltage divider circuit
    RLOAD = 1.0

    # Calibration resistance at atmospheric CO2 level
    # This is the resistance of the sensor at 397.13 ppm CO2
    # This value is used to calculate the RZero value for calibration
    RZERO = 12.25

    # Parameters for calculating ppm of CO2 from sensor resistance
    PARA = 116.6020682
    PARB = 2.769034857

    # Parameters to model temperature and humidity dependence
    CORA = 0.00035
    CORB = 0.02718
    CORC = 1.39538
    CORD = 0.0018
    CORE = -0.003333333
    CORF = -0.001923077
    CORG = 1.130128205

    # Atmospheric CO2 level for calibration purposes
    ATMOCO2 = 397.13

    def __init__(self, analog_pin=0, digital_pin=2, adc_offset=400):
        """
        Initialize MQ135 sensor interface
        
        Args:
            analog_pin: ADC pin object connected to the sensor's analog output
            digital_pin: Digital pin object connected to the sensor's digital output (DO)
            adc_offset: ADC offset value to compensate for (default 400)
        """
        
        self.analog_pin = analog_pin
        self.digital_pin = digital_pin
        
        # ADC configuration for analog reading
        self.adc = ADC(analog_pin)
        self.adc_for_calibration = ADC(4)
        self.adc_offset = adc_offset
        self.AVREF = 3.3  # Reference voltage for ADC
        self.ADC_MAX = 65535  # 16-bit ADC value
        self.oversample = 10  # Number of samples to average for noise reduction
        
        # Dynamic RZero calibration
        self.RZERO = 12.25  # Default starting value
        self.rzero_samples = []
        self.rzero_window = 20  # How many samples to keep
        self.calibration_complete = False

        # Configure Din pin for digital output
        if digital_pin is not None:
            self.do_pin = Pin(digital_pin, Pin.IN)
        else:
            self.do_pin = None
            
        # Debug flag
        self.debug = False
    
    def set_debug(self, enable=True):
        """Enable or disable debug output"""
        self.debug = enable
    
    def calibrate_offset(self, num_samples=100):
        """
        Calibrate the ADC offset by reading the zero point
        Connect the ADC input to ground when running this
        
        Args:
            num_samples: Number of samples to average for calibration
        """
        sum_value = 0
        for _ in range(num_samples):
            sum_value += self.adc_for_calibration.read_u16()
        self.adc_offset = sum_value // num_samples
        return self.adc_offset
    
    def read_raw(self):
        """Read raw ADC value with oversampling and offset compensation"""
        sum_value = 0
        valid_samples = 0
        
        for _ in range(self.oversample):
            value = self.adc.read_u16()
            
            # Skip rail conditions
            if value == 0 or value == self.ADC_MAX:
                continue
                
            # Subtract offset and check for valid range
            compensated = value - self.adc_offset
            if compensated < 0:
                compensated = 0
            elif compensated > self.ADC_MAX:
                compensated = self.ADC_MAX
                
            sum_value += compensated
            valid_samples += 1
                
        if valid_samples == 0:
            # Fall back to a single raw reading if no valid samples
            return self.adc.read_u16()
            
        return sum_value // valid_samples
    
    def get_voltage(self):
        """Read voltage across the sensor with improved accuracy"""
        raw = self.read_raw()
        voltage = (raw / self.ADC_MAX) * self.AVREF
        
        if self.debug:
            print(f"Raw ADC: {raw}, Voltage: {voltage:.4f}V")
            
        return voltage
    
    def _get_resistance(self):
        """Returns the resistance of the sensor in kOhms with improved accuracy"""
        raw = self.read_raw()
                
        # Prevent division by zero
        if raw == 0:
            return float('inf')
        
        # Convert to voltage
        voltage = (raw / self.ADC_MAX) * self.AVREF
        
        if voltage >= self.AVREF or voltage <= 0:
            return float('inf')
            
        # Calculate resistance using formula for sensor on top of voltage divider
        # Rs = Rl * (Vin - Vout) / Vout
        resistance = (self.RLOAD * (self.AVREF - voltage)) / voltage
        
        if self.debug:
            print(f"Calculated resistance: {resistance:.2f} kOhms")
            
        return resistance
    
    def _get_correction_factor(self, temperature, humidity):
        """Calculates the correction factor for ambient air temperature and relative humidity"""
        if temperature < 20:
            return self.CORA * temperature * temperature - self.CORB * temperature + self.CORC - (humidity - 33.) * self.CORD
        return self.CORE * temperature + self.CORF * humidity + self.CORG

    def get_corrected_resistance(self, temperature, humidity):
        """Gets the resistance of the sensor corrected for temperature/humidity"""
        resistance = self._get_resistance()
        correction = self._get_correction_factor(temperature, humidity)
        
        # Avoid division by zero
        if correction <= 0:
            return resistance
            
        return resistance / correction

    def get_rzero(self):
        """Returns the resistance RZero of the sensor (in kOhms) for calibration purposes"""
        resistance = self._get_resistance()
        
        # Check for invalid resistance values
        if resistance <= 0:
            if self.debug:
                print("Error: Invalid resistance for RZero calculation")
            return -1  # Error code
            
        rzero = resistance * math.pow((self.ATMOCO2/self.PARA), (1./self.PARB))
        
        if self.debug:
            print(f"Calculated RZero: {rzero:.2f} kOhms")
            
        return rzero

    def get_corrected_rzero(self, temperature, humidity):
        """Returns the RZero of the sensor (in kOhms) for calibration purposes corrected for temperature/humidity"""
        resistance = self.get_corrected_resistance(temperature, humidity)
        
        # Check for invalid resistance values
        if resistance <= 0:
            if self.debug:
                print("Error: Invalid corrected resistance for RZero calculation")
            return -1  # Error code
            
        return resistance * math.pow((self.ATMOCO2/self.PARA), (1./self.PARB))
    
    def update_rzero(self, temperature, humidity):
        """Update RZERO with a moving average"""
        new_rzero = self.get_corrected_rzero(temperature, humidity)
        
        # Only update if we got a valid reading
        if new_rzero > 0:
            # Add to samples
            self.rzero_samples.append(new_rzero)
            
            # Keep only the last N samples
            if len(self.rzero_samples) > self.rzero_window:
                self.rzero_samples.pop(0)
            
            # Update RZERO with the average
            self.RZERO = sum(self.rzero_samples) / len(self.rzero_samples)
            
            # Mark calibration as complete if we have enough samples
            if len(self.rzero_samples) >= self.rzero_window:
                self.calibration_complete = True
                
        return self.RZERO
    
    def get_ppm(self):
        """Returns the ppm of CO2 sensed (assuming only CO2 in the air)"""
        resistance = self._get_resistance()
        
        # Check for invalid resistance values
        if resistance <= 0:
            if self.debug:
                print("Error: Invalid resistance for PPM calculation")
            return -1  # Error code
            
        ppm = self.PARA * math.pow((resistance / self.RZERO), -self.PARB)
        
        if self.debug:
            print(f"Calculated PPM: {ppm:.2f}")
            
        return ppm

    def get_corrected_ppm(self, temperature, humidity):
        """Returns the ppm of CO2 sensed (assuming only CO2 in the air) corrected for temperature/humidity"""
        resistance = self.get_corrected_resistance(temperature, humidity)
        
        # Check for invalid resistance values
        if resistance <= 0:
            if self.debug:
                print("Error: Invalid corrected resistance for PPM calculation")
            return -1  # Error code
            
        ppm = self.PARA * math.pow((resistance / self.RZERO), -self.PARB)
        
        if self.debug:
            print(f"Corrected PPM: {ppm:.2f}")
            
        return ppm

    def is_danger(self):
        """
        Check if the digital danger pin is indicating dangerous gas levels
        
        Returns:
            True if danger levels detected, False otherwise
            None if digital pin wasn't initialized
        """
        if self.do_pin is None:
            if self.debug:
                print("Warning: Digital pin not initialized")
            return None
            
        # According to the schematic, the digital output goes LOW when 
        # gas levels are high (dangerous). The comparator output triggers 
        # when the gas concentration exceeds the threshold set by the 
        # onboard potentiometer.
        return not self.do_pin.value()  # Return True if pin is LOW (danger)
    
    def get_danger_level(self):
        """
        Get a descriptive danger level based on PPM and digital pin
        
        Returns:
            String describing the danger level
        """
        ppm = self.get_ppm()
        
        # Check digital pin first
        if self.do_pin is not None and not self.do_pin.value():
            return "DANGER - Critical gas levels detected!"
        
        # Then check PPM levels
        if ppm < 0:
            return "ERROR - Invalid sensor reading"
        elif ppm < 700:
            return "SAFE - Normal CO2 levels"
        elif ppm < 1000:
            return "GOOD - Acceptable CO2 levels"
        elif ppm < 2000:
            return "WARNING - Elevated CO2 levels"
        elif ppm < 5000:
            return "HIGH - High CO2 levels, improve ventilation"
        else:
            return "DANGER - Very high CO2 levels, evacuate area!"

    def is_connected(self):
        """Check if sensor appears to be connected properly"""
        resistance = self._get_resistance()
        if resistance is None or resistance <= 0:
            return False
        # Check if resistance is within expected range (MQ sensors have wide range)
        return 0.1 <= resistance <= 10000.0
    
    def get_debug_info(self):
        """Get all sensor readings for debugging"""
        # Get all readings
        raw = self.read_raw()
        voltage = self.get_voltage()
        resistance = self._get_resistance()
        rzero = self.get_rzero()
        ppm = self.get_ppm()
        
        # Check digital danger signal if available
        danger = "Not initialized"
        if self.do_pin is not None:
            danger = "ACTIVE (LOW)" if not self.do_pin.value() else "Normal (HIGH)"
        
        debug_info = {
            'adc_offset': self.adc_offset,
            'raw_adc': raw,
            'voltage': f"{voltage:.4f}V",
            'resistance': f"{resistance:.2f} kOhms",
            'rzero': f"{rzero:.2f}" if rzero > 0 else "INVALID",
            'ppm': f"{ppm:.2f}" if ppm > 0 else "INVALID",
            'connected': self.is_connected(),
            'digital_output': danger,
            'danger_level': self.get_danger_level()
        }   

        print("\n=== MQ135 Debug Information ===")
        # Print in a specific order
        for key in ['adc_offset', 'raw_adc', 'voltage', 'resistance', 'rzero', 
                    'ppm', 'connected', 'digital_output', 'danger_level']:
            if key in debug_info:
                print(f"{key}: {debug_info[key]}")
        print("===============================\n")
        
        return debug_info

class EnhancedDHT22:
    """Enhanced DHT22 sensor implementation with additional features"""
    
    def __init__(self, pin):
        """Initialize the DHT22 sensor"""
        self.sensor = DHT22(Pin(pin, Pin.IN))

        # Default values for temperature and humidity
        self.last_temp = 21.0
        self.last_humidity = 25.0
        
        # Number of read attempts before error
        self.read_attempts = 5
        self.last_error_time = 0
        self.error_cooldown = 60  # Only show errors once per minute
        
    def read(self):
        """Read temperature and humidity with retry logic"""
        for attempt in range(self.read_attempts):
            try:
                time.sleep(0.25)  # Stabilization delay
                self.sensor.measure()
                self.last_temp = self.sensor.temperature()
                self.last_humidity = self.sensor.humidity()
                return True
            except Exception as e:
                if attempt < self.read_attempts - 1:
                    time.sleep(1)  # Wait before retry
                else:
                    current_time = time.time()
                    if current_time - self.last_error_time > self.error_cooldown:
                        print(f"DHT22 read failed after {self.read_attempts} attempts: {e}")
                        self.last_error_time = current_time
                    return False
    
    def temperature(self):
        """Get the last successful temperature reading"""
        return self.last_temp
    
    def temperature_celsius(self):
        """Get the last successful temperature reading in Celsius"""
        return self.last_temp
    
    def temperature_fahrenheit(self):
        """Get the last successful temperature reading in Fahrenheit"""
        return self.last_temp * 9.0 / 5.0 + 32.0
    
    def humidity(self):
        """Get the last successful humidity reading"""
        return self.last_humidity

class Node(MQ135, EnhancedDHT22):
    """ Local class for dealing with sensors """

    def __init__(self, mq135, dht22):
        """Initialize MQ135 and enhanced DHT22 sensors"""
        self.mq135 = mq135
        self.dht22 = dht22
        
        # Default values now come from the DHT22 class
        self.temperature = self.dht22.temperature()
        self.humidity = self.dht22.humidity()

    def update_environment_data(self):
        """Update temperature and humidity from DHT22 sensor"""
        try:
            self.dht22.read()
            self.temperature = self.dht22.temperature()
            self.humidity = self.dht22.humidity()
            return True
        
        except Exception as e:
            print("Error reading DHT22 sensor:", e)
            return False

    def read_mq135_values(self):
        """Read all values from MQ135 sensor"""
        values = {
            'rzero': self.mq135.get_rzero(),
            'corrected_rzero': self.mq135.get_corrected_rzero(self.temperature, self.humidity),
            'resistance': self.mq135._get_resistance(),
            'ppm': self.mq135.get_ppm(),
            'corrected_ppm': self.mq135.get_corrected_ppm(self.temperature, self.humidity)
        }
        return values

    def print_sensor_data(self):
        """Print current temperature, humidity and MQ135 readings"""
        values = self.read_mq135_values()
        
        # Check for error values
        if values['resistance'] <= 0:
            print("Error: Invalid resistance reading")
            return
        if values['ppm'] <= 0:
            print("Error: Invalid PPM reading")
            return
        
        # Print sensor data
        print("=========================================")
        print("DHT22 Sensor Data:")
        print(f"DHT22 Temperature: {self.temperature}C\t Humidity: {self.humidity}%")
        print("=========================================")
        print("MQ135 Sensor Data:")
        print(f"MQ135 RZero: {values['rzero']:.2f}\tCorrected RZero: {values['corrected_rzero']:.2f}")
        print(f"Resistance: {values['resistance']:.2f}\tPPM: {values['ppm']:.2f}")
        print(f"Corrected PPM: {values['corrected_ppm']:.2f} ppm")
        print("=========================================\n")

    def continuous_monitoring(self, update_interval=2, use_dht=True, transmit=False, xbee=None):
        """Continuously monitor and print sensor values, optionally transmit data
        
        Args:
            update_interval: Time between readings in seconds
            use_dht: Whether to use real DHT22 readings or default values
            transmit: Whether to transmit data over XBee
            xbee: TransparentSend object for communication (required if transmit=True)
        """
        if transmit and xbee is None:
            raise ValueError("XBee object required for transmission")
            
        try:
            while True:
                if use_dht:
                    self.update_environment_data()
                
                values = self.read_mq135_values()
                self.print_sensor_data()
                
                if transmit and xbee:
                    # Send data over XBee
                    temperature = self.temperature
                    humidity = self.humidity
                    corrected_ppm = values['corrected_ppm']
                    xbee.send_sensor_data(temperature, humidity, corrected_ppm)
                    
                time.sleep(update_interval)
                
        except KeyboardInterrupt:
            print("Monitoring stopped")

        except Exception as e:
            print("Error during monitoring:", e)
            raise e

class APISend:
    """Class for handling XBee communication in API mode"""
    
    # XBee API frame markers
    START_DELIMITER = 0x7E
    ESCAPE = 0x7D
    XON = 0x11
    XOFF = 0x13
    
    # API frame types
    TX_REQUEST_64 = 0x00  # TX Request: 64-bit address
    TX_REQUEST_16 = 0x01  # TX Request: 16-bit address
    AT_COMMAND = 0x08     # AT Command
    TX_REQUEST = 0x10     # Transmit Request
    
    def __init__(self, uart_id=0, baudrate=9600, tx_pin=0, rx_pin=1, dest_addr=0xFFFF):
        """Initialize UART communication for XBee in API mode
        
        Args:
            uart_id: UART interface ID
            baudrate: Baud rate for communication
            tx_pin: TX pin number
            rx_pin: RX pin number
            dest_addr: Destination address (default: 0xFFFF for broadcast)
        """        
        self.uart = UART(uart_id, 
                       baudrate=baudrate, 
                       bits=8,
                       parity=None,
                       stop=1,
                       tx=tx_pin, 
                       rx=rx_pin,
                       timeout=1000)
        
        self.dest_addr = dest_addr
        
        # Add small delay for XBee power up
        time.sleep(1)
        print("XBee API mode communication initialized")
    
    def _calculate_checksum(self, data):
        """Calculate the checksum for an API frame"""
        # Sum all the bytes in the frame data
        total = sum(data)
        # Return the lowest 8 bits of the sum, then subtract from 0xFF
        return 0xFF - (total & 0xFF)
    
    def _escape_data(self, data):
        """Escape special bytes in API data"""
        escaped = bytearray()
        for byte in data:
            # If the byte needs to be escaped (is a special character)
            if byte in [self.START_DELIMITER, self.ESCAPE, self.XON, self.XOFF]:
                escaped.append(self.ESCAPE)
                escaped.append(byte ^ 0x20)  # XOR with 0x20
            else:
                escaped.append(byte)
        return escaped
    
    def _create_api_frame(self, frame_type, frame_data):
        """Create a complete API frame with proper structure
        
        Args:
            frame_type: API frame type identifier
            frame_data: Data for the frame
            
        Returns:
            Complete API frame as bytearray
        """
        # Create frame data starting with the frame type
        # Convert frame_type to int if it's not already
        frame_type = int(frame_type)
        
        # Ensure frame_data is a bytearray
        if not isinstance(frame_data, bytearray):
            frame_data = bytearray(frame_data)
        
        data = bytearray([frame_type]) + frame_data
        
        # Calculate length (API data length)
        length = len(data)
        
        # Calculate checksum
        checksum = self._calculate_checksum(data)
        
        # Create the complete frame
        frame = bytearray([self.START_DELIMITER])  # Start delimiter
        frame.extend([length >> 8, length & 0xFF])  # Length (MSB, LSB)
        frame.extend(data)  # Frame data (includes frame type)
        frame.append(checksum)  # Checksum
        
        return frame
    
    def simple_api_send_with_address(self, data_string, use_64bit=False):
        """XBee API frame send with proper checksum calculation
        
        Args:
            data_string: String data to send
            use_64bit: If True, send to 64-bit address; if False, broadcast to 16-bit address
        """
        # Convert string to bytes
        data_bytes = data_string.encode('utf-8')
        
        # Create frame data (everything between length and checksum)
        frame_data = bytearray()
        
        if not use_64bit:
            # Original 16-bit addressing method (frame type 0x01 = TX Request 16-bit address)
            frame_type = 0x01
            frame_data.append(frame_type)
            
            # Frame ID
            frame_data.append(0x01)
            
            # Destination address (broadcast)
            frame_data.append(0xFF)
            frame_data.append(0xFF)
            
            # Data payload
            frame_data.extend(data_bytes)

        else:
            # 64-bit addressing method (frame type 0x10 = TX Request 64-bit address)
            frame_type = 0x10
            frame_data.append(frame_type)
            
            # Frame ID
            frame_data.append(0x01)
            
            # 64-bit destination address (specific XBee)
            frame_data.append(0x00)
            frame_data.append(0x13)
            frame_data.append(0xA2)
            frame_data.append(0x00)
            frame_data.append(0x42)
            frame_data.append(0x01)
            frame_data.append(0x09)
            frame_data.append(0x17)
            
            # 16-bit address (use 0xFFFE for 64-bit addressing mode)
            frame_data.append(0xFF)
            frame_data.append(0xFE)
            
            # Broadcast radius
            frame_data.append(0x00)
            
            # Options
            frame_data.append(0x00)
            
            # Data payload
            frame_data.extend(data_bytes)
        
        # Calculate frame length
        length = len(frame_data)
        
        # Create complete frame
        frame = bytearray()
        frame.append(0x7E)                  # Start delimiter
        frame.append((length >> 8) & 0xFF)  # Length MSB
        frame.append(length & 0xFF)         # Length LSB
        frame.extend(frame_data)            # Frame data
        
        # Calculate checksum - sum all bytes in frame_data and subtract from 0xFF
        checksum = 0
        for b in frame_data:
            checksum += b
        checksum = 0xFF - (checksum & 0xFF)
        frame.append(checksum)
        
        # Print frame for debugging - this helps verify the frame structure
        print(f"Start delimiter: 0x{frame[0]:02X}")
        print(f"Length: {length} bytes (0x{frame[1]:02X} 0x{frame[2]:02X})")
        print(f"Frame type: 0x{frame[3]:02X}")
        print(f"Frame ID: 0x{frame[4]:02X}")
        
        if not use_64bit:
            print(f"16-bit Destination: 0x{frame[5]:02X}{frame[6]:02X}")
            data_start = 7
        else:
            print(f"64-bit Destination: 0x{frame[5]:02X}{frame[6]:02X}{frame[7]:02X}{frame[8]:02X}{frame[9]:02X}{frame[10]:02X}{frame[11]:02X}{frame[12]:02X}")
            print(f"16-bit Address: 0x{frame[13]:02X}{frame[14]:02X}")
            print(f"Broadcast Radius: 0x{frame[15]:02X}")
            print(f"Options: 0x{frame[16]:02X}")
            data_start = 17
        
        print(f"Data: {data_string}")
        print(f"Checksum: 0x{checksum:02X}\n")
        # print(f"Complete frame: {[hex(b) for b in frame]}")
        
        # Send the frame
        try:
            self.uart.write(bytes(frame))
            # result = self.uart.write(bytes(frame))
            #print(f"Sent {result} bytes")
            return True
        except Exception as e:
            print(f"Error sending frame: {e}")
            return False

    def simple_api_send(self, data_string):
        """Most basic XBee API frame send possible with proper checksum calculation
        
        Args:
            data_string: String data to send
        """
        # Convert string to bytes
        data_bytes = data_string.encode('utf-8')
        
        # Create frame data (everything between length and checksum)
        frame_data = bytearray()
        
        # Frame type (0x10 = TX Request 64-bit address)
        # Frame type (0x01 = TX Request 16-bit address)
        frame_type = 0x01
        frame_data.append(frame_type)
        
        # Frame ID
        frame_data.append(0x01)
        
        # Destination address (broadcast)
        frame_data.append(0xFF)
        frame_data.append(0xFF)

        # Destination address (64-bit address)
        # Sink Address = ([0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x09, 0x17])
        # Source A = ([0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x09, 0x19])
        # Source B = ([0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x08, 0xEB])
        # frame_data.extend([0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x09, 0x17])
        # 00 13 A2 00 42 01 09 17

        # frame_data.append(0x00)
        # frame_data.append(0x13)
        # frame_data.append(0xA2)
        # frame_data.append(0x00)
        # frame_data.append(0x42)
        # frame_data.append(0x01)
        # frame_data.append(0x09)
        # frame_data.append(0x17)

        # Data payload
        frame_data.extend(data_bytes)
        
        # Calculate frame length
        length = len(frame_data)
        
        # Create complete frame
        frame = bytearray()
        frame.append(0x7E)                  # Start delimiter
        frame.append((length >> 8) & 0xFF)  # Length MSB
        frame.append(length & 0xFF)         # Length LSB
        frame.extend(frame_data)            # Frame data
        
        # Calculate checksum - sum all bytes in frame_data and subtract from 0xFF
        checksum = 0
        for b in frame_data:
            checksum += b
        checksum = 0xFF - (checksum & 0xFF)
        frame.append(checksum)
        
        # Print frame for debugging - this helps verify the frame structure
        print(f"Start delimiter: 0x{frame[0]:02X}")
        print(f"Length: {length} bytes (0x{frame[1]:02X} 0x{frame[2]:02X})")
        print(f"Frame type: 0x{frame[3]:02X}")
        print(f"Frame ID: 0x{frame[4]:02X}")
        print(f"Destination: 0x{frame[5]:02X}{frame[6]:02X}")
        print(f"Data: {data_string}")
        print(f"Checksum: 0x{checksum:02X}")
        print(f"Complete frame: {[hex(b) for b in frame]}")

        
        # Send the frame
        try:
            result = self.uart.write(bytes(frame))
            print(f"Sent {result} bytes")
            return True
        except Exception as e:
            print(f"Error sending frame: {e}")
            return False
    
    def simple_api_send_64bit(self, data_string):
        """Most basic XBee API frame send possible with proper checksum calculation
        
        Args:
            data_string: String data to send
        """
        # Convert string to bytes
        data_bytes = data_string.encode('utf-8')
        
        # Create frame data (everything between length and checksum)
        frame_data = bytearray()
        
        # Frame type 
        frame_type = 0x10
        frame_data.append(frame_type)
        
        # Frame ID
        frame_data.append(0x01)
        
        # Destination address (64-bit address)
        # Sink Address = ([0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x09, 0x17])
        # Source A = ([0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x09, 0x19])
        # Source B = ([0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x08, 0xEB])
        # frame_data.extend([0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x09, 0x17])
        # 00 13 A2 00 42 01 09 17

        frame_data.append(0x00)
        frame_data.append(0x13)
        frame_data.append(0xA2)
        frame_data.append(0x00)
        frame_data.append(0x42)
        frame_data.append(0x01)
        frame_data.append(0x09)
        frame_data.append(0x17)

        # Destination address (broadcast)
        frame_data.append(0xFF)
        frame_data.append(0xFE)

        # Broadcast radius
        frame_data.append(0x00)

        # Options
        frame_data.append(0x00)

        # Data payload (RF data)
        frame_data.extend(data_bytes)
        
        # Calculate frame length
        length = len(frame_data)
        
        # Create complete frame
        frame = bytearray()
        frame.append(0x7E)                  # Start delimiter
        frame.append((length >> 8) & 0xFF)  # Length MSB
        frame.append(length & 0xFF)         # Length LSB
        frame.extend(frame_data)            # Frame data
        
        # Calculate checksum - sum all bytes in frame_data and subtract from 0xFF
        checksum = 0
        for b in frame_data:
            checksum += b
        checksum = 0xFF - (checksum & 0xFF)
        frame.append(checksum)
        
        # Print frame for debugging - this helps verify the frame structure
        print(f"Start delimiter: 0x{frame[0]:02X}")
        print(f"Length: {length} bytes (0x{frame[1]:02X} 0x{frame[2]:02X})")
        print(f"Frame type: 0x{frame[3]:02X}")
        print(f"Frame ID: 0x{frame[4]:02X}")
        print(f"64-bit Destination: 0x{frame[5]:02X}{frame[6]:02X}{frame[7]:02X}{frame[8]:02X}{frame[9]:02X}{frame[10]:02X}{frame[11]:02X}{frame[12]:02X}")
        print(f"16-bit Address: 0x{frame[13]:02X}{frame[14]:02X}")
        print(f"Broadcast Radius: 0x{frame[15]:02X}")
        print(f"Options: 0x{frame[16]:02X}")
        print(f"Data: {data_string}")
        print(f"Checksum: 0x{checksum:02X}")
        print(f"Complete frame: {[hex(b) for b in frame]}")

        # Send the frame
        try:
            result = self.uart.write(bytes(frame))
            print(f"Sent {result} bytes")
            return True
        except Exception as e:
            print(f"Error sending frame: {e}")
            return False

    def send_at_command(self, command, parameter=None):
        """Send an AT command to the XBee
        
        Args:
            command: Two-character AT command (e.g., 'ID', 'NI')
            parameter: Optional parameter for the command
        """
        # Frame ID - set to 1 for command response
        frame_id = 0x01
        
        # Create frame data for AT Command (0x08)
        frame_data = bytearray([frame_id])  # Frame ID
        
        # Convert command string to bytes
        if isinstance(command, str):
            frame_data.extend(command.encode('ascii'))
        else:
            frame_data.extend(bytearray(command))
        
        # Add parameter if provided
        if parameter is not None:
            if isinstance(parameter, str):
                frame_data.extend(parameter.encode('ascii'))
            elif isinstance(parameter, int):
                frame_data.append(parameter)
            elif isinstance(parameter, list):
                # Convert list to bytearray
                frame_data.extend(bytearray(parameter))
            else:
                frame_data.extend(bytearray(parameter))
        
        # Create the complete API frame
        api_frame = self._create_api_frame(self.AT_COMMAND, frame_data)
        
        # Debug print
        print("AT command frame:", [hex(b) for b in api_frame])
        
        # Write the frame to UART - make sure it's bytes
        self.uart.write(bytes(api_frame))
        
        # Short delay for transmission
        time.sleep(0.1)
        
        # Return frame ID for tracking response
        return frame_id

    def send_data_works(self, data, dest_addr=None):
        """Send data using a Transmit Request frame
        
        Args:
            data: Data to send (string or bytes)
            dest_addr: Optional destination address (defaults to self.dest_addr)
        """
        import time
        
        if dest_addr is None:
            dest_addr = self.dest_addr
        
        # Convert string to bytes if needed
        if isinstance(data, str):
            data = data.encode('utf-8')
        
        # Frame ID - set to 1 for transmit status response
        frame_id = 0x01
        
        # Create frame data for TX Request (0x10)
        frame_data = bytearray([frame_id])  # Frame ID
        
        # Destination 64-bit address for your specific XBee
        frame_data.extend([0x00, 0x13, 0xA2, 0x00, 0x42, 0x01, 0x09, 0x17])
        
        # Destination 16-bit address
        frame_data.extend([(dest_addr >> 8) & 0xFF, dest_addr & 0xFF])
        
        # Broadcast radius (0 = max hops)
        frame_data.append(0x00)
        
        # Options (0 = default)
        frame_data.append(0x00)
        
        # RF data
        frame_data.extend(data)
        
        # Create the API frame
        # Start delimiter
        api_frame = bytearray([self.START_DELIMITER])
        
        # Length (MSB, LSB) - length of the frame data
        length = len(frame_data) + 1  # +1 for frame type
        api_frame.extend([length >> 8, length & 0xFF])
        
        # Frame type (TX Request)
        api_frame.append(self.TX_REQUEST)
        
        # Frame data
        api_frame.extend(frame_data)
        
        # Calculate checksum
        checksum = 0
        for b in frame_data:
            checksum += b
        checksum += self.TX_REQUEST
        checksum = 0xFF - (checksum & 0xFF)
        
        # Add checksum
        api_frame.append(checksum)
        
        # Debug output
        print("API Frame:", [hex(b) for b in api_frame])
        
        # Write to UART - all tests passed, so any format should work
        result = self.uart.write(api_frame)
        print(f"UART write result: {result} bytes")
        
        # Short delay for transmission
        time.sleep(0.1)
        
        # Return frame ID for tracking response
        return frame_id

    def close(self):
        """Close UART connection"""
        self.uart.deinit()
        print("UART closed")

def test_uart_communication(uart_id=0, baudrate=9600, tx_pin=0, rx_pin=1, test_message="UART TEST"):
    """
    Test UART communication by sending a test message and checking for responses
    
    Args:
        uart_id: UART interface ID
        baudrate: Baud rate for communication
        tx_pin: TX pin number
        rx_pin: RX pin number
        test_message: Message to send for testing
    """
    import time
    from machine import UART
    
    print(f"Initializing UART test (ID: {uart_id}, Baud: {baudrate}, TX: {tx_pin}, RX: {rx_pin})...")
    
    # Initialize UART
    uart = UART(uart_id, 
               baudrate=baudrate, 
               bits=8,
               parity=None,
               stop=1,
               tx=tx_pin, 
               rx=rx_pin,
               timeout=1000)
    
    try:
        # Clear any pending data
        pending = uart.any()
        if pending:
            print(f"Clearing {pending} bytes from buffer")
            uart.read()
        
        # Test loop
        counter = 1
        max_tests = 5
        
        print(f"\nStarting UART test - will send {max_tests} messages with 2 second intervals")
        print("Press Ctrl+C to stop the test")
        
        while counter <= max_tests:
            # Create test message with counter
            message = f"{test_message} #{counter}"
            message_bytes = message.encode('utf-8')
            
            # Send message
            print(f"\nTest {counter}/{max_tests}: Sending: '{message}'")
            bytes_written = uart.write(message_bytes)
            print(f"Bytes written: {bytes_written}")
            
            # Listen for response with timeout
            print("Waiting for response (3 seconds)...")
            
            # Set timeout time
            timeout_time = time.time() + 3
            response = b""
            
            # Read with timeout
            while time.time() < timeout_time:
                if uart.any():
                    new_data = uart.read()
                    if new_data:
                        response += new_data
                        print(f"Received {len(new_data)} bytes")
                
                # Short delay to prevent tight loop
                time.sleep(0.1)
                
                # If we've received a full response, we can stop waiting
                if response and not uart.any():
                    break
            
            # Display results
            if response:
                try:
                    decoded = response.decode('utf-8')
                    print(f"Response received: '{decoded}'")
                except UnicodeError:
                    print(f"Raw response (could not decode): {response}")
            else:
                print("No response received within timeout period")
            
            # Increment counter and wait before next test
            counter += 1
            print("Waiting 2 seconds before next test...")
            time.sleep(2)
        
        print("\nUART test completed")
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    except Exception as e:
        print(f"Error during UART test: {e}")
    
    finally:
        # Close UART
        uart.deinit()
        print("UART connection closed")

def main():
    """Main function"""

    #test_uart_communication()

    # Initialize sensors
    mq135_sensor = MQ135(0)
    dht_sensor = EnhancedDHT22(Pin(2, Pin.IN))

    # Initialize XBee communication in API mode
    xbee = APISend(uart_id=0, baudrate=9600, tx_pin=0, rx_pin=1)
    
    # Pass sensors to Node
    sensor_node = Node(mq135=mq135_sensor, dht22=dht_sensor)
    
    # update time in seconds
    update_time = 1
    sample_number = 0

    # Set transmit to True or False
    transmit = True
    # transmit = False

    if transmit == False:
            # Start continuous monitoring without data transmission
            print("Starting continuous monitoring without data transmission...")
            try:
                sensor_node.continuous_monitoring(use_dht=True)
            except KeyboardInterrupt:
                print("Monitoring stopped")
            except Exception as e:
                print("Error during monitoring:", e)
                raise e
    else: 
       # Start continuous monitoring with data transmission
        print("Starting continuous monitoring with data transmission in API mode...")
        try:
            print("Starting main loop...")
            
            while True:
                # Update sensor readings
                sensor_node.update_environment_data()

                # Update RZERO
                sensor_node.mq135.update_rzero(sensor_node.temperature, sensor_node.humidity)
                
                # Get sensor values
                values = sensor_node.read_mq135_values()
                temperatureC = sensor_node.dht22.temperature_celsius()
                humidity = sensor_node.humidity
                corrected_ppm = values['corrected_ppm']
                
                # Print sensor data
                print(f"\nSample: {sample_number}")
                sensor_node.print_sensor_data()
                
                # Format data as string
                data_string = f"DATA:TEMP:{temperatureC:.2f},HUM:{humidity:.2f},PPM:{corrected_ppm:.2f}"
                print(f"Sending data: {data_string}")
                
                # Try the simplified approach
                try:
                    xbee.simple_api_send_64bit(data_string)
                    # xbee.simple_api_send_with_address(data_string, use_64bit=False)
                    
                    # result = xbee.simple_api_send(data_string)
                    # result = xbee.simple_api_send_with_address(data_string, use_64bit=False)
                    # print(f"Simple API send result: {result}")
                except Exception as e:
                    print(f"Error in simple_api_send: {e}")
                
                sample_number += 1
                # Wait before next reading
                time.sleep(update_time)
            
        # Catch keyboard interrupt to stop monitoring
        except KeyboardInterrupt:
            print("Monitoring stopped by user")

        # Catch any other exceptions
        except Exception as e:
            print(f"Main function error: {e}")

        # Close XBee connection if it was initialized
        finally:
            if xbee:
                xbee.close()

if __name__ == "__main__":
    main()