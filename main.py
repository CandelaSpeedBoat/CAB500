import time
import numpy as np
import can
from enum import IntEnum
import struct

# idList = np.arange(0x40B, 0x7F0, 1)
idList = np.arange(0x600, 0x6FF, 1)
receivedCorrectMsg = False
receivedReadDatabyIDDone = False
udsClientID = 0x0000        #
udsServerID = 0x0000        #
cab500IpID = 0x0000         #
cab500IpIDnew  = 0x06AC      #
udsServerIDnew = 0x06AD      #
udsClientIDnew = 0x06AE      #

debugging = False           # If true, extra debug message is printed


freq_dict = {1: 'IIR10_HZ', 2: 'IIR20_HZ', 3: 'IIR30_HZ', 4: 'IIR40_HZ', 5: 'IIR50_HZ', 6: 'IIR60_HZ',
             7: 'IIR70_HZ', 8: 'IIR80_HZ', 9: 'IIR90_HZ', 10: 'IIR100_HZ', 11: 'IIR110_HZ', 12: 'IIR120_HZ',
             13: 'IIR130_HZ', 14: 'IIR140_HZ', 15: 'IIR150_HZ', 16: 'IIR160_HZ', 255: 'AVERAGE_TEN_MS'}
canSpeed_dict = {0x007D: "baudrate_125k", 0x00FA:"baudrate_250k", 0x01F4: "baudrate_500k"}


def print_hi(name):
    print(f'Msg:, {name}')


class CAB500(IntEnum):
    udsClientID = 0x0000
    udsServerID = 0x0000
    cab500IpID = 0x0000
    receivedCorrectMsg: bool = False
    receivedReadDatabyIDDone: bool = False
    SINGLE_FRAME_1_BYTE = 0x01  # Frame consists of 1 msg with 1 byte of data
    SINGLE_FRAME_2_BYTE = 0x02  # Frame consists of 1 msg with 2 byte of data
    SINGLE_FRAME_3_BYTE = 0x03
    SINGLE_FRAME_4_BYTE = 0x04
    SINGLE_FRAME_5_BYTE = 0x05
    SINGLE_FRAME_6_BYTE = 0x06
    SINGLE_FRAME_7_BYTE = 0x07
    SINGLE_FRAME_8_BYTE = 0x08
    SINGLE_FRAME_9_BYTE = 0x09
    firstMessage = 0x10
    secondMessage = 0x21
    nineUsableData = 0x09

    # ECU reset function --- does not work ---
    ecuResetMsg = 0x11
    ecuResetMsg_send_hard = 0x01
    ecuResetMsg_send_soft = 0x03

    # Read data by identifier, single frame composed of 3 byte, \x03\x22\x then choose function
    readDataById = 0x22
    # Read CAN ID, \x03\x22\xF0\x10, then response, then transmit flow control \x03\x00\x00
    subf_CAN_ID = 0xF010
    subf_FilterFreq = 0xF011
    subf_CANspeed = 0xF012
    subf_FrameFreq = 0xF013

    # Sub function
    flowControl = 0x30  # 00 00
    posResponseRead = 0x62
    negResponse = 0x7F

    # WriteDataByIdentifier
    writeDataById = 0x2E
    posResponseWrite = 0x6E
    # Frequency of filter
    IIR10_HZ = 0x01
    IIR20_HZ = 0x02
    IIR30_HZ = 0x03
    IIR40_HZ = 0x04
    IIR50_HZ = 0x05
    IIR60_HZ = 0x06
    IIR70_HZ = 0x07
    IIR80_HZ = 0x08
    IIR90_HZ = 0x09
    IIR100_HZ = 0x0A
    IIR110_HZ = 0x0B
    IIR120_HZ = 0x0C
    IIR130_HZ = 0x0D
    IIR140_HZ = 0x0E
    IIR150_HZ = 0x0F
    IIR160_HZ = 0x10
    AVERAGE_TEN_MS = 0xFF

    #freq_dict = {"IIR10_HZ": 0x01, "IIR20_HZ": 0x02, "IIR30_HZ": 0x03, "IIR40_HZ": 0x04, "IIR50_HZ": 0x05,
     #            "IIR60_HZ": 0x06, "IIR70_HZ": 0x07, "IIR80_HZ": 0x08, "IIR90_HZ": 0x09, "IIR100_HZ": 0x0A,
      #           "IIR110_HZ": 0x0B, "IIR120_HZ": 0x0C, "IIR130_HZ": 0x0D, "IIR140_HZ": 0x0E,
       #          "IIR150_HZ": 0x0F, "IIR160_HZ": 0x10, "AVERAGE_TEN_MS": 0xFF}
    #freq_dict = {1: 'IIR10_HZ', 2: 'IIR20_HZ', 3: 'IIR30_HZ', 4: 'IIR40_HZ', 5: 'IIR50_HZ', 6: 'IIR60_HZ',
    #             7: 'IIR70_HZ', 8: 'IIR80_HZ', 9: 'IIR90_HZ', 10: 'IIR100_HZ', 11: 'IIR110_HZ', 12: 'IIR120_HZ',
    #             13: 'IIR130_HZ', 14: 'IIR140_HZ', 15: 'IIR150_HZ', 16: 'IIR160_HZ', 255: 'AVERAGE_TEN_MS'}

    # CAN speed ie. baud rate
    baudrate_125k = 0x007D
    baudrate_250k = 0x00FA
    baudrate_500k = 0x01F4


def sendStdCANmessage(msg_id, data_msg, debug):
    msg = can.Message(
        arbitration_id=msg_id,
        data=data_msg,
        is_extended_id=False
    )
    try:
        bus.ch.send(msg)
        if debug:
            print(f"Message sent on {bus.ch.channel_info}")
            print(f"Data sent: {msg}")
    except can.CanError:
        print("Message NOT sent")


def ecuResetService(msg_id, debug):
    if msg_id == 0:
        if debug:
            print("Message not sent, msg_id = 0")
    else:
        # does not work properly on this version of CAB500, will not send response,
        # will generate a error frame
        data_msg = [CAB500.SINGLE_FRAME_2_BYTE, CAB500.ecuResetMsg, CAB500.ecuResetMsg_send_soft]
        sendStdCANmessage(msg_id, data_msg, debug)


def readDatabyIdentifierID(msg_id, debug):
    data_msg = [CAB500.SINGLE_FRAME_3_BYTE, CAB500.readDataById, CAB500.subf_CAN_ID >> 8, CAB500.subf_CAN_ID & 255]
    sendStdCANmessage(msg_id, data_msg, debug)


def readDatabyIdentifierFilerfreq(msg_id, debug):
    data_msg = [CAB500.SINGLE_FRAME_3_BYTE, CAB500.readDataById, CAB500.subf_CAN_ID >> 8, CAB500.subf_FilterFreq & 255]
    sendStdCANmessage(msg_id, data_msg, debug)
    notifier = can.Notifier(bus.ch, [receive_can_data_signle])

def readDatabyIdentifierFramefreq(msg_id, debug):
    data_msg = [CAB500.SINGLE_FRAME_3_BYTE, CAB500.readDataById, CAB500.subf_CAN_ID >> 8, CAB500.subf_FrameFreq & 255]
    sendStdCANmessage(msg_id, data_msg, debug)
    notifier = can.Notifier(bus.ch, [receive_can_data_signle])

def readDatabyIdentifierCANspeed(msg_id, debug):
    data_msg = [CAB500.SINGLE_FRAME_3_BYTE, CAB500.readDataById, CAB500.subf_CAN_ID >> 8, CAB500.subf_CANspeed & 255]
    sendStdCANmessage(msg_id, data_msg, debug)
    notifier = can.Notifier(bus.ch, [receive_can_data_signle])


def flowControl(msg_id, debug):
    data_msg =[CAB500.flowControl, 0x00, 0x00]
    sendStdCANmessage(msg_id, data_msg, debug)


def writeDatabyIdentifier(msg_id, debug):
    if msg_id == 0:
        if debug:
            print("Message not sent, msg_id = 0")
    else:
        data_msg = [CAB500.firstMessage, CAB500.SINGLE_FRAME_9_BYTE, CAB500.writeDataById, CAB500.subf_CAN_ID >> 8,
                    CAB500.subf_CAN_ID & 255, cab500IpIDnew >> 8, cab500IpIDnew & 255, udsClientIDnew >> 8]
        sendStdCANmessage(msg_id, data_msg, debug)
def writeDatabyIdentifier2nd(msg_id, debug):
    data_msg = [CAB500.secondMessage, udsClientIDnew & 255, udsServerIDnew >> 8, udsServerIDnew & 255]
    sendStdCANmessage(msg_id, data_msg, debug)

class CanInterface:

    def __init__(self, node_id, channel='can0', bustype='socketcan', bitrate='500000'):
        self.node_id = node_id
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.accept_virtual = 1

        # TODO move to function to connect to network
        # TODO add function to disconnect network.disconnect()
        # Try to find a CAN-Bus interface
        for interface, channel in [('kvaser', 0), ('kvaser', 1),('socketcan', 'can0'), ('pcan', 'PCAN_USBBUS1'), ('ixxat', 0)]:
            try:
                self._can_bus = can.ThreadSafeBus(interface=interface, channel=channel, bitrate=self.bitrate)
                break
            except (OSError, can.CanError, NameError, ImportError):
                pass
        else:
            raise Exception('No CAN-Bus interface was found')
        self.ch = self._can_bus
        #  TODO add filter to notifier, filter on udsServerID


    def add_can_msg_callback(self):
        pass

    def remove_can_msg_callback(self):
        pass

    def close_bus(self):
        # self._can_notifier.stop(timeout=1)
        self._can_bus.shutdown()


def receive_can_data(msg):
    global receivedCorrectMsg, cab500IpID, udsClientID, udsServerID, receivedReadDatabyIDDone

    # print(f"Recieved data: {msg.data}, id: {hex(msg.arbitration_id)}")
    if msg.is_rx == True and msg.dlc == 8:
        if debugging:
            print("Message recieved and dlc=8")
            print(msg.data)
        if msg.data[0] == CAB500.firstMessage:  # First frame of message
            if msg.data[1] == CAB500.nineUsableData:
                if msg.data[2] == CAB500.posResponseRead:
                    # print(f"Positive response")
                    if int.from_bytes(msg.data[3:5]) == CAB500.subf_CAN_ID:
                        receivedCorrectMsg = True
                        cab500IpID = int.from_bytes(msg.data[5:7])
                        # print(cab500IpID)
                        udsClientID = msg.data[7]
                        # print(udsClientID)
                else:
                    print("Negative response, wrong format" + msg.data)

        elif msg.data[0] == CAB500.secondMessage:
            # print("Second message received")
            udsClientID = int.from_bytes([udsClientID, msg.data[1]])
            udsServerID = int.from_bytes(msg.data[2:4])
            receivedReadDatabyIDDone = True
        elif msg.data[0] == CAB500.flowControl and \
                int.from_bytes([msg.data[1], msg.data[2], msg.data[3],
                                msg.data[4], msg.data[5], msg.data[6],
                                msg.data[7]]) == 0:
            print("flow control message received")
            writeDatabyIdentifier2nd(udsClientID, debugging)
        elif msg.data[0] == CAB500.SINGLE_FRAME_3_BYTE and msg.arbitration_id == udsServerID:
            if msg.data[1] == CAB500.posResponseWrite:
                print("Change accepted, subfunction: ")
                print(hex(msg.data[2]), hex(msg.data[3]))
            elif msg.data[1] == CAB500.negResponse:
                print("Negative response, subfunction: ")
                print(hex(msg.data[2]))
            else:
                print("Error, WriteDataByIdentifier")
                print(hex(msg.data[1]))
                print(hex(msg.data[2]))

def receive_can_data_signle(msg):
    global receivedCorrectMsg, cab500IpID, udsClientID, udsServerID, receivedReadDatabyIDDone

    # print(f"Recieved data: {msg.data}, id: {hex(msg.arbitration_id)}")
    if msg.is_rx == True and msg.dlc == 8:
        if msg.data[0] == CAB500.SINGLE_FRAME_4_BYTE: # and msg.arbitration_id == udsServerID:
            if msg.data[1] == CAB500.posResponseRead:
                # print("Read data by ID, subfunction: ")
                # print(hex((msg.data[2] << 8) + msg.data[3]))
                if (msg.data[2] << 8) + msg.data[3] == CAB500.subf_FilterFreq:
                    print("subFilterFreq: ", freq_dict[msg.data[4]])
        if msg.data[0] == CAB500.SINGLE_FRAME_5_BYTE:  # and msg.arbitration_id == udsServerID:
            if msg.data[1] == CAB500.posResponseRead:
                # print("Read data by ID, subfunction: ")
                # print(hex((msg.data[2] << 8) + msg.data[3]))
                if ((msg.data[2] << 8) + msg.data[3]) == CAB500.subf_CANspeed:
                    print("subCANspeed: ", canSpeed_dict[(msg.data[4] << 8) + msg.data[5]])
                elif ((msg.data[2] << 8) + msg.data[3]) == CAB500.subf_FrameFreq:
                    print("subCANspeed: ", msg.data[5], "ms")

            else:
                print("Error, ReadDataByIdentifier")
                print(hex(msg.data[1]))
                print(hex(msg.data[2]))



if __name__ == '__main__':
    print_hi('Start program')
    print_hi(f'Min: {hex(min(idList))}, Max: {hex(max(idList))}')
    bus = CanInterface(0x11)
    notifier = can.Notifier(bus.ch, [receive_can_data])

    for id in idList:
        readDatabyIdentifierID(id, debugging)
        time.sleep(0.01)
        if receivedCorrectMsg:
            flowControl(id, debugging)
            time.sleep(0.01)
            print_hi('Done, udsClientID: ' + hex(udsClientID) + ' , udsServerID: ' +
                     hex(udsServerID) + ' and cab500IpID: ' + hex(cab500IpID))
            time.sleep(0.01)
            readDatabyIdentifierFilerfreq(udsClientID, debugging)
            time.sleep(0.01)
            readDatabyIdentifierCANspeed(udsClientID, debugging)
            time.sleep(0.01)
            readDatabyIdentifierFramefreq(udsClientID, debugging)

    if receivedReadDatabyIDDone:
        print_hi('Done, udsClientID: ' + hex(udsClientID) + ' , udsServerID: ' +
                 hex(udsServerID) + ' and cab500IpID: ' + hex(cab500IpID))

    #writeDatabyIdentifier(udsClientID, debugging)

    time.sleep(0.01)
    #ecuResetService(udsClientID, debugging)
    bus.close_bus()
    print_hi('Stop program')
