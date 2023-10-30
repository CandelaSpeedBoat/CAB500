

import time
import numpy as np
import can
from enum import IntEnum

myList = np.arange(0x40B, 0x7F0, 1)


def print_hi(name):
    print(f'Hi, {name}')

def CAB500():
    SINGLE_FRAME_1_BYTE = 0x01 # Frame consists of 1 msg with 1 byte of data
    SINGLE_FRAME_2_BYTE = 0x02 # Frame consists of 1 msg with 2 byte of data
    SINGLE_FRAME_3_BYTE = 0x03
    SINGLE_FRAME_4_BYTE = 0x04
    SINGLE_FRAME_5_BYTE = 0x05
    SINGLE_FRAME_6_BYTE = 0x06
    SINGLE_FRAME_7_BYTE = 0x07
    SINGLE_FRAME_8_BYTE = 0x08

    # def __init__(self):

def ecuResetService(msg_id):
    msg = can.Message(
        arbitration_id=msg_id,
        data=[0x02, 0x11, 0x01],
        is_extended_id=False
    )
    try:
        bus.ch.send(msg)
        print(f"Message sent on {bus.ch.channel_info}")
    except can.CanError:
        print("Message NOT sent")



    class filterFreq(IntEnum):    # Avalible filter frequencies
        TEN_HZ          = 0x01
        TWENTY_HZ       = 0x02
        THIRTY_HZ       = 0x03
        AVERAGE_TEN_MS  = 0xFF


class CanInterface:


    def __init__(self, node_id=0x11, channel='can0', bustype='socketcan', bitrate='500000'):
        self.node_id = node_id
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.accept_virtual = 1

        # TODO move to function to connect to network
        # TODO add function to disconnect network.disconnect()
        # Try to find a CAN-Bus interface
        for interface, channel in [('kvaser', 0), ('kvaser', 1)]:
            try:
                self._can_bus = can.ThreadSafeBus(interface=interface, channel=channel, bitrate=self.bitrate)
                break
            except (OSError, can.CanError, NameError, ImportError):
                pass
        else:
            raise Exception('No CAN-Bus interface was found')
        self.ch = self._can_bus
        #  TODO add filter to notifier
        #  _can_listener = can.Listener()
        #  _can_listener.on_message_received = self._msg_callback
        #  self._can_notifier = can.Notifier(self._can_bus, [_can_listener])
        #  self._can_notifier = can.Notifier()

    def add_can_msg_callback(self):
        pass

    def remove_can_msg_callback(self):
        pass

    def close_bus(self):
        # self._can_notifier.stop(timeout=1)
        self._can_bus.shutdown()

def receive_can_data(can):
    SingleCanFrame = can.Message
    print(SingleCanFrame)


if __name__ == '__main__':
    print_hi('Start')
    bus = CanInterface()
    notifier = can.Notifier(bus.ch, [receive_can_data(can)])
    ecuResetService(0x601)
    bus.close_bus()
    print_hi('Stop')
