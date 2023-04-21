import adafruit_rfm9x
import board
import busio
import digitalio

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import Executor, MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, SetParametersResult

import rfm9x_interfaces.msg

class RFM9xController(Node):
    '''Provides a ROS2 interface to an Adafruit RFM9x radio module.

    RFM9xController provides three services (Receive, Send, and SendWithAck)
    and a number of parameters for configuring and interacting with an Adafruit
    RFM9x LoRa radio from a ROS2 environment.
    '''
    def __init__(
            self,
            executor: Executor,
            spi: busio.SPI,
            cs: digitalio.DigitalInOut,
            reset: digitalio.DigitalInOut,
            frequency: int,
            *,
            preamble_length: int = 8,
            high_power: bool = True,
            baudrate: int = 5000000,
            agc: bool = False,
            crc: bool = True
        ):
        '''Creates a RFM9xController node with the configuration specified

        From adafruit_rfm9x.RFM9x.init:
        "You must specify the following parameters:
        - spi: The SPI bus connected to the radio.
        - cs: The CS pin DigitalInOut connected to the radio.
        - reset: The reset/RST pin DigialInOut connected to the radio.
        - frequency: The frequency (in mhz) of the radio module (433/915mhz typically).
        You can optionally specify:
        - preamble_length: The length in bytes of the packet preamble (default 8).
        - high_power: Boolean to indicate a high power board (RFM95, etc.).  Default
        is True for high power.
        - baudrate: Baud rate of the SPI connection, default is 10mhz but you might
        choose to lower to 1mhz if using long wires or a breadboard.
        - agc: Boolean to Enable/Disable Automatic Gain Control - Default=False (AGC off)
        - crc: Boolean to Enable/Disable Cyclic Redundancy Check - Default=True (CRC Enabled)"
        '''
        super().__init__(node_name='rmf9x_controller', parameter_overrides=[])

        # These depend on wiring and should not be modified after
        # the RFM9xController is initialized.
        self._spi = spi
        self._cs = cs
        self._reset = reset

        self.log = self.get_logger()
        self.executor: Executor = executor
        self._timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter('frequency', frequency, ParameterDescriptor(
            name='frequency', type=2, read_only=True,
            description=('The frequency (in mhz) of the radio module '
                         '(433/915mhz typically).')))
        self.declare_parameter('preamble_length', preamble_length, ParameterDescriptor(
            name='preamble_length', type=2, read_only=True,
            description='The length in bytes of the packet preamble.'))
        self.declare_parameter('high_power', high_power, ParameterDescriptor(
            name='high_power', type=1, read_only=True,
            description='Boolean to indicate a high power board (RFM95, etc.).'))
        self.declare_parameter('baudrate', baudrate, ParameterDescriptor(
            name='baudrate', type=2, read_only=True,
            description=('Baud rate of the SPI connection. Default is 5mhz, '
                    'but you might choose to lower to 1mhz if using long wires '
                    'or a breadboard.')))
        self.declare_parameter('agc', agc, ParameterDescriptor(
            name='agc', type=1, read_only=True,
            description='Boolean to enable/disable automatic gain control.'))
        self.declare_parameter('crc', crc, ParameterDescriptor(
            name='crc', type=1, read_only=True,
            description='Boolean to enable/disable cyclic redundancy check.'))
        self.declare_parameter('signal_bandwidth', 125000, ParameterDescriptor(
            name='signal_bandwidth', type=2,
            description=('The signal bandwidth used by the radio. Try setting '
                         'to a higher value to increase throughput or to a lower '
                         'value to increase the likelihood of successfully received '
                         'payloads. Valid values: 7800, 10400, 15600, 20800, 31250, '
                         '41700, 62500, 125000, 250000')))
        self.declare_parameter('coding_rate', 5, ParameterDescriptor(
            name='coding_rate', type=2,
            integer_range=[IntegerRange(from_value=5, to_value=8)],
            description=('The coding rate used by the radio to control forward '
                         'error correction. Try setting to a higher value to increase '
                         'tolerance of short bursts of interference or to a lower '
                         'value to increase bit rate. Valid values: 5, 6, 7, 8')))
        self.declare_parameter('spreading_factor', 7, ParameterDescriptor(
            name='spreading_factor', type=2,
            integer_range=[IntegerRange(from_value=6, to_value=12)],
            description=('The spreading factor used by the radio. Try setting '
                         'to a higher value to increase the receiver\'s ability to '
                         'distinguish signal from noise or to a lower value to '
                         'increase the data transmission rate. Valid values: 6, 7, '
                         '8, 9, 10, 11, 12')))
        self.declare_parameter('tx_power', 23, ParameterDescriptor(
            name='tx_power', type=2,
            integer_range=[IntegerRange(from_value=-1, to_value=23)],
            description=('The transmit power in dBm. Can be set to a value from '
                         '5 to 23 for high power devices (RFM95/96/97/98, '
                         'high_power=True) or -1 to 14 for low power devices. The '
                         'actual maximum setting for high_power=True is 20dBm but '
                         'for values > 20 the PA_BOOST will be enabled resulting in '
                         'an additional gain of 3dBm. The actual setting is reduced '
                         'by 3dBm. The reported value will reflect the reduced setting.')))
        self.declare_parameter('receive_timeout', 0.5, ParameterDescriptor(
            name='receive_timeout', type=3,
            description=('The amount of time to poll for a received packet. '
                         'If no packet is received, the returned packet will be None.')))
        self.declare_parameter('xmit_timeout', 2.0, ParameterDescriptor(
            name='xmit_timeout', type=3,
            description=('The amount of time to wait for the HW to transmit the '
                         'packet. This is mainly used to prevent a hang due to a HW '
                         'issue.')))
        self.declare_parameter('ack_delay', 0.0, ParameterDescriptor(
            name='ack_delay', type=3,
            description=('The delay time before attempting to send an ACK. If '
                         'ACKs are being missed try setting this to 0.1 or 0.2.')))
        self.declare_parameter('node', 0x01, ParameterDescriptor(
            name='node', type=2,
            integer_range=[IntegerRange(from_value=0, to_value=255)],
            description=('The default address of this Node. If not 255 (0xff) '
                         'then only packets addressed to this node will be accepted. '
                         'First byte of the RadioHead header.')))
        self.declare_parameter('destination', 0x00, ParameterDescriptor(
            name='destination', type=2,
            integer_range=[IntegerRange(from_value=0, to_value=255)],
            description=('The default destination address for packet '
                         'transmissions. If 255 (0xff) then any receiving node should '
                         'accept the packet. Second byte of the RadioHead header.')))

        self.add_on_set_parameters_callback(self._set_parameters_callback)

        # Publish received commands for command parser
        self._cmd_pub = self.create_publisher(rfm9x_interfaces.msg.Payload, 'cmd_data', 10)
        # .. at intervals of 1 second
        self._listen_timer = self.create_timer(1.0, self._listen_timer_callback, self._timer_callback_group)

        # Subscribe to telemetry data
        self._telem_sub = self.create_subscription(rfm9x_interfaces.msg.Payload,
                'telem_data', self._telem_sub_callback, 1)

        # Bind node to radio device
        try:
            self.rfm9x = adafruit_rfm9x.RFM9x(
                    self._spi,
                    self._cs,
                    self._reset,
                    self.get_parameter('frequency').get_parameter_value().integer_value,
                    preamble_length = self.get_parameter('preamble_length').get_parameter_value().integer_value,
                    high_power = self.get_parameter('high_power').get_parameter_value().bool_value,
                    baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value,
                    agc = self.get_parameter('agc').get_parameter_value().bool_value,
                    crc = self.get_parameter('crc').get_parameter_value().bool_value)
        except RuntimeError as exc:
            self.log.fatal(f'Exception raised while initalizing RFM9xController: {str(exc)}')
            raise RuntimeError from exc

    def _set_parameters_callback(self, params) -> SetParametersResult:
        # TODO: Validate changed parameters
        raise NotImplementedError('Parameter setting unimplemented. Edit values in source')

    def _telem_sub_callback(self, msg):
        '''Transmit telemetry data'''
        self.log.info('Sending telemetry')
        telem_bytes = bytes(msg.payload)
        self.rfm9x.send(telem_bytes, keep_listening=True)

    def _listen_timer_callback(self):
        '''Listen for controller messages'''
        if (ctrl_msg := self.rfm9x.receive(with_ack=True)) is not None:
            self.log.info('Received controller message')
            data_array = [byte for byte in ctrl_msg]
            self._cmd_pub.publish(rfm9x_interfaces.msg.Payload(payload=data_array))

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    CS = digitalio.DigitalInOut(board.CE1)
    RESET = digitalio.DigitalInOut(board.D25)
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    node = RFM9xController(executor, spi, CS, RESET, 915)
    executor.add_node(node)

    node.get_logger().info('Running')
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

