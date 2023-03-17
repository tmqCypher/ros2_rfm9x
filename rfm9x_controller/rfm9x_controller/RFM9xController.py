import time
import adafruit_rfm9x
import board
import busio
import digitalio

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor, SetParametersResult

from rfm9x_interfaces.srv import Receive, Send, SendWithAck

class RFM9xController(Node):
    '''Provides a ROS2 interface to an Adafruit RFM9x radio module.

    RFM9xController provides three services (Receive, Send, and SendWithAck)
    and a number of parameters for configuring and interacting with an Adafruit
    RFM9x LoRa radio from a ROS2 environment.
    '''
    def __init__(
            self,
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

        self.rfm9x = None
        self.timer = None

        # These depend on wiring and should not be modified after
        # the RFM9xController is initialized.
        self._spi = spi
        self._cs = cs
        self._reset = reset

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
        self.declare_parameter('ack_wait', 0.5, ParameterDescriptor(
            name='ack_wait', type=3,
            description=('The delay time before attempting a retry after not '
                         'receiving an ACK.')))
        self.declare_parameter('receive_timeout', 0.5, ParameterDescriptor(
            name='receive_timeout', type=3,
            description=('The amount of time to poll for a received packet. '
                         'If no packet is received, the returned packet will be None.')))
        self.declare_parameter('xmit_timeout', 2.0, ParameterDescriptor(
            name='xmit_timeout', type=3,
            description=('The amount of time to wait ofr the HW to transmit the '
                         'packet. This is mainly used to prevent a hang due to a HW '
                         'issue.')))
        self.declare_parameter('ack_retries', 5, ParameterDescriptor(
            name='ack_retries', type=2,
            description='The number of ACK retries before reporting a failure.'))
        self.declare_parameter('ack_delay', 0.0, ParameterDescriptor(
            name='ack_delay', type=3,
            description=('The delay time before attempting to send an ACK. If '
                         'ACKs are being missed try setting this to 0.1 or 0.2.')))
        self.declare_parameter('node', 255, ParameterDescriptor(
            name='node', type=2,
            integer_range=[IntegerRange(from_value=0, to_value=255)],
            description=('The default address of this Node. If not 255 (0xff) '
                         'then only packets addressed to this node will be accepted. '
                         'First byte of the RadioHead header.')))
        self.declare_parameter('destination', 255, ParameterDescriptor(
            name='destination', type=2,
            integer_range=[IntegerRange(from_value=0, to_value=255)],
            description=('The default destination address for packet '
                         'transmissions. If 255 (0xff) then any receiving node should '
                         'accept the packet. Second byte of the RadioHead header.')))

        self.add_on_set_parameters_callback(self._set_parameters_callback)

        self.receive = self.create_service(Receive, 'receive',
                self._receive_callback)
        self.send = self.create_service(Send, 'send',
                self._send_callback)
        self.send_with_ack = self.create_service(SendWithAck, 'send_with_ack', 
                self._send_with_ack_callback)

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

        self.log = self.get_logger()
        self.log.info('Initialized')


    def _set_parameters_callback(self, params) -> SetParametersResult:
        # TODO: Validate changed parameters
        return SetParametersResult(successful=True)

    def _receive_callback(self, request, response):
        if self.rfm9x is None:
            return self._not_bound_response(response)

        packet = self.rfm9x.receive(
                keep_listening=request.keep_listening,
                with_header=request.with_header,
                with_ack=request.with_ack,
                timeout=(request.receive_timeout if request.override_receive_timeout else None))

        if packet is None:
            response.success = False
            return response
        else:
            response.success = True
            header_log_str = 'Not requested'
            if request.with_header:
                response.header.destination = packet[0]
                response.header.node = packet[1]
                response.header.identifier = packet[2]
                response.header.flags = packet[3]
                response.payload = packet[4:]
                header_log_str = (f'destination: {response.header.destination}, '
                                  f'node: {response.header.node}, identifier: '
                                  f'{response.header.identifier}, '
                                  f'flags: {response.header.flags}')
            else:
                response.payload = packet[:]

            self.log.info((f'Received packet\nHeader: {header_log_str}\n'
                           f'Payload: {str(response.payload, "utf-8")}'))

            return response

    def _send_callback(self, request, response):
        if self.rfm9x is None:
            return self._not_bound_response(response)

        destination=(request.header.destination if request.override_destination else None)
        node=(request.header.node if request.override_node else None)
        identifier=(request.header.identifier if request.override_identifier else None)
        flags=(request.header.flags if request.override_flags else None)

        start_time = time.time()
        response.result = self.rfm9x.send(
                request.payload,
                keep_listening=request.keep_listening,
                destination=destination,
                node=node,
                identifier=identifier,
                flags=flags)
        end_time = time.time()

        if response.result:
            self.log.info((f'Transmission sent ({end_time - start_time})\n'
                           f'Header: destination: {destination}, node: {node}, '
                           f'identifier: {identifier}, flags: {flags}\n'
                           f'Payload: {str(request.payload, "utf-8")}'))
        else:
            self.log.warning('Send request timed out')

        return response

    def _send_with_ack_callback(self, request, response):
        if self.rfm9x is None:
            return self._not_bound_response(response)

        # TODO: Logging
        response.ack_received = self.rfm9x.send_with_ack(request.payload)
        return response

    def _not_bound_response(self, response):
        msg = 'Node is not bound to a device'
        self.log.error(msg)
        response.success = False
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)

    CS = digitalio.DigitalInOut(board.CE1)
    RESET = digitalio.DigitalInOut(board.D25)
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    node = RFM9xController(spi, CS, RESET, 915)

    node.get_logger().info('Running')
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
