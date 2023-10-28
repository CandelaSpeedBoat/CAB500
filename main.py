

import time
import numpy as np
import can

myList = np.arange(0x40B, 0x7F0, 1)


def print_hi(name):
    print(f'Hi, {name}')


class CanInterface:
    def __init__(self, node_id=0x11, channel='can0', bustype='socketcan', bitrate='500000'):
        self.node_id = node_id
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate

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
        self.network = self._can_bus
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


if __name__ == '__main__':
    print_hi('Start')
    bus = CanInterface()
    bus.close_bus()
    print_hi('Stop')
