"""
Micropython implementation of a CO monitor using cheap MQ135 and DHT22 sensors
"""

from machine import Pin, ADC, UART
from dht import DHT22

import time
import math

class MQ135():
    """ 
    Class for dealing with MQ135 Gas Sensors with improved accuracy 
    This class provides methods for reading sensor values, calibrating, and calculating PPM
    It also includes methods for temperature and humidity correction, digital output handling,
    
    #TODO: lots of redundant methods from debugging RZERO, clean up later
    """
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
    """
    Enhanced DHT22 sensor implementation with additional features
    This class provides improved error handling, retry logic, and temperature/humidity reading methods
    """
    
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
    """ 
    Local class for dealing with sensors 
    
    This class combines the MQ135 and enhanced DHT22 sensor functionalities
    
    #TODO: maybe remove this class and just use the two sensor classes directly
    """

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

    def print_sensor_data_old(self):
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

    def print_sensor_data(self):
        """Print current temperature, humidity and MQ135 readings with consistent formatting"""
        values = self.read_mq135_values()
        
        # Check for error values
        if values['resistance'] <= 0:
            print("Error: Invalid resistance reading")
            return
        if values['ppm'] <= 0:
            print("Error: Invalid PPM reading")
            return
        
        # Calculate the width needed for consistent display
        total_width = 54  # Set fixed width
        
        # Create borders and format the output
        border = "+" + "-" * (total_width - 2) + "+"
        
        # Print sensor data with consistent formatting
        print(border)
        
        # DHT22 section
        section_title = "DHT22 Sensor Data"
        padding = total_width - len(section_title) - 4  # -4 for "| " and " |"
        left_padding = padding // 2
        right_padding = padding - left_padding
        print("| " + " " * left_padding + section_title + " " * right_padding + " |")
        
        # DHT22 data
        dht_data = f"Temperature: {self.temperature:.2f}C    Humidity: {self.humidity:.2f}%"
        dht_padding = total_width - len(dht_data) - 4
        print("| " + dht_data + " " * dht_padding + " |")
        
        print(border)
        
        # MQ135 section
        section_title = "MQ135 Sensor Data"
        padding = total_width - len(section_title) - 4
        left_padding = padding // 2
        right_padding = padding - left_padding
        print("| " + " " * left_padding + section_title + " " * right_padding + " |")
        
        # MQ135 data lines - format each line separately to avoid unpacking errors
        rzero_line = f"RZero: {values['rzero']:.2f}    Corrected RZero: {values['corrected_rzero']:.2f}"
        rzero_padding = total_width - len(rzero_line) - 4
        print("| " + rzero_line + " " * rzero_padding + " |")
        
        resistance_line = f"Resistance: {values['resistance']:.2f}    PPM: {values['ppm']:.2f}"
        resistance_padding = total_width - len(resistance_line) - 4
        print("| " + resistance_line + " " * resistance_padding + " |")
        
        corrected_ppm_line = f"Corrected PPM: {values['corrected_ppm']:.2f} ppm"
        corrected_ppm_padding = total_width - len(corrected_ppm_line) - 4
        print("| " + corrected_ppm_line + " " * corrected_ppm_padding + " |")
        
        print(border)

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
        
    # TODO: combine these three api send methods into one
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

        # Use the formatter instead of multiple print statements
        # TODO: change to look for diagnostic output flag
        if True:
            formatter = XBeeFrameFormatter(frame)
            print(formatter.format_pretty())

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

    # TODO: remove
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

class XBeeFrameFormatter:
    """
    Format and display XBee API frames
    TODO: Vibe Coded, clean up later
    """
    
    def __init__(self, frame_data=None):
        """Initialize with optional frame data"""
        self.frame_data = frame_data
        self.diag = False  # Debug flag for detailed output
    
    def parse_from_hex_list(self, hex_list: list[str]) -> 'XBeeFrameFormatter':
        """Parse frame from a list of hex strings"""
        # Convert hex strings to bytes
        self.frame_data = bytearray()
        for hex_str in hex_list:
            # Remove '0x' prefix if present and convert to integer
            if hex_str.startswith('0x'):
                hex_str = hex_str[2:]
            self.frame_data.append(int(hex_str, 16))
        return self
    
    def parse_api_frame(self):
        """Parse the frame into components"""
        if not self.frame_data or len(self.frame_data) < 5:
            return "Invalid frame: too short"
            
        # Basic frame parsing
        start_delimiter = self.frame_data[0]
        length_msb = self.frame_data[1]
        length_lsb = self.frame_data[2]
        frame_length = (length_msb << 8) | length_lsb
        
        # Verify expected frame length
        expected_total_length = frame_length + 4  # Start delimiter + 2 length bytes + data + checksum
        if len(self.frame_data) != expected_total_length:
            return f"Warning: Expected total length {expected_total_length}, got {len(self.frame_data)}"
        
        # Extract frame components
        frame_type = self.frame_data[3]
        frame_id = self.frame_data[4]
        
        # For Transmit Request (0x10) frames
        if frame_type == 0x10:
            # 64-bit address (8 bytes)
            addr64 = self.frame_data[5:13]
            # 16-bit address (2 bytes)
            addr16 = self.frame_data[13:15]
            # Broadcast radius and options
            broadcast_radius = self.frame_data[15]
            options = self.frame_data[16]
            
            # RF Data
            rf_data = self.frame_data[17:-1]
            
            # Checksum
            checksum = self.frame_data[-1]
            
            # Try to decode RF data as UTF-8 if possible
            try:
                rf_data_str = rf_data.decode('utf-8')
            except UnicodeDecodeError:
                rf_data_str = "Non-printable data"
            
            # Create formatted output
            return {
                "start_delimiter": f"0x{start_delimiter:02X}",
                "length": f"{frame_length} bytes (0x{length_msb:02X} 0x{length_lsb:02X})",
                "frame_type": f"0x{frame_type:02X} (Transmit Request)",
                "frame_id": f"0x{frame_id:02X}",
                "64bit_dest": f"0x{''.join('%02X' % b for b in addr64)}",
                "16bit_dest": f"0x{''.join('%02X' % b for b in addr16)}",
                "broadcast_radius": f"0x{broadcast_radius:02X}",
                "options": f"0x{options:02X}",
                "data": rf_data_str,
                "data_hex": " ".join(f"0x{b:02X}" for b in rf_data),
                "checksum": f"0x{checksum:02X}"
            }
        else:
            return f"Unsupported frame type: 0x{frame_type:02X}"
    
    def format_pretty(self) -> str:
        """Format the frame data in a pretty, readable format with dynamic sizing"""
        result = self.parse_api_frame()
        if isinstance(result, str):
            return result
        
        # Determine the maximum width needed
        content_items = [
            "Start Delimiter: " + result['start_delimiter'],
            "Length:          " + result['length'],
            "Frame Type:      " + result['frame_type'],
            "Frame ID:        " + result['frame_id'],
            "64-bit Dest:     " + result['64bit_dest'],
            "16-bit Dest:     " + result['16bit_dest'],
            "Broadcast Radius:" + result['broadcast_radius'],
            "Options:         " + result['options'],
            "Checksum:        " + result['checksum'],
            result['data']
        ]
        
        # Get the longest content item plus margin
        max_content_width = max(len(item) for item in content_items) + 10
        
        # Ensure width is enough for hex data display
        hex_line_width = max_content_width
        
        # Set total width for frame
        total_width = max_content_width + 4  # +4 for "| " and " |"
        
        # Create header and section bars
        header_bar = "+" + "-" * (total_width - 2) + "+"
        section_title_width = total_width 
        header_title = "| " + "XBee API Frame".center(section_title_width - 4) + " |"
        rf_data_bar = "| " + "RF Data".center(section_title_width - 4) + " |"
        hex_data_bar = "| " + "Hex Data".center(section_title_width - 4) + " |"
        footer_bar = "| " + "Footer".center(section_title_width - 4) + " |"
        
        # Create a neatly formatted output with consistent spacing
        output = []
        output.append(header_bar)
        output.append(header_title)
        output.append(header_bar)
        
        # Add frame details with consistent padding
        for label, value in [
            ("Start Delimiter: ", result['start_delimiter']),
            ("Length:          ", result['length']),
            ("Frame Type:      ", result['frame_type']),
            ("Frame ID:        ", result['frame_id']),
            ("64-bit Dest:     ", result['64bit_dest']),
            ("16-bit Dest:     ", result['16bit_dest']),
            ("Broadcast Radius:", result['broadcast_radius']),
            ("Options:         ", result['options'])
        ]:
            padding = total_width - len(label) - len(value) - 4  # -4 for "| " and " |"
            output.append("| " + label + value + " " * padding + " |")
        
        # Add RF data section
        output.append(header_bar)
        output.append(rf_data_bar)
        output.append(header_bar)
        
        # Format RF data with consistent padding
        rf_data = result['data']
        rf_data_padding = total_width - len(rf_data) - 4  # -4 for "| " and " |"
        output.append("| " + rf_data + " " * rf_data_padding + " |")
        
        # Add Hex data section
        output.append(header_bar)
        output.append(hex_data_bar)
        output.append(header_bar)
        
        # Format hex data with line breaks for readability
        hex_data = result['data_hex']
        max_hex_per_line = hex_line_width - 4  # -4 for "| " and " |"
        
        # Split the hex data by space to handle words properly
        hex_words = hex_data.split(" ")
        current_line = ""
        
        for word in hex_words:
            # Check if adding this word would exceed the limit
            if len(current_line + " " + word if current_line else word) <= max_hex_per_line:
                current_line += (" " + word if current_line else word)
            else:
                # Line is full, add it and start a new one
                padding = total_width - len(current_line) - 4
                output.append("| " + current_line + " " * padding + " |")
                current_line = word
        
        # Add the last line if there's anything remaining
        if current_line:
            padding = total_width - len(current_line) - 4
            output.append("| " + current_line + " " * padding + " |")
        
        # Add footer
        output.append(header_bar)
        output.append(footer_bar)
        output.append(header_bar)
        
        # Add checksum
        checksum_label = "Checksum: "
        checksum_value = result['checksum']
        checksum_padding = total_width - len(checksum_label) - len(checksum_value) - 4
        output.append("| " + checksum_label + checksum_value + " " * checksum_padding + " |")
        
        # Final border
        output.append(header_bar)
        
        return "\n".join(output)

def main():
    """Main function"""

    # Set Diag flag for debugging
    diag = True

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
                print(f"\nSending data: {data_string}\n")
                
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