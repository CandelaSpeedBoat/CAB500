import time
import argparse
import numpy as np
import can
from can.notifier import MessageRecipient
from typing import List
import sensorCAB as sense

msgID = 0x0000
receivedReadDatabyIDDone = False
idList1 = np.arange(0x40B, 0x7F1, 1)
idList2 = np.arange(0x221, 0x400, 1)
idList = np.append(idList1, idList2)
filterFreqList = np.linspace(1,16,16)
filterFreqList = np.append(filterFreqList, 255)
framePeriodList = np.linspace(10, 100, 91)

debugging = False  # If true, extra debug message is printed

freq_dict = {1: 'IIR10_HZ', 2: 'IIR20_HZ', 3: 'IIR30_HZ', 4: 'IIR40_HZ', 5: 'IIR50_HZ', 6: 'IIR60_HZ',
             7: 'IIR70_HZ', 8: 'IIR80_HZ', 9: 'IIR90_HZ', 10: 'IIR100_HZ', 11: 'IIR110_HZ', 12: 'IIR120_HZ',
             13: 'IIR130_HZ', 14: 'IIR140_HZ', 15: 'IIR150_HZ', 16: 'IIR160_HZ', 255: 'AVERAGE_TEN_MS'}
canSpeed_dict = {0x007D: "baudrate_125k", 0x00FA: "baudrate_250k", 0x01F4: "baudrate_500k"}


def print_hi(name):
    print(f'Msg:, {name}')


def auto_int(x):
    return int(x, 0)


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
        data_msg = [sense.CAB500.SINGLE_FRAME_2_BYTE, sense.CAB500.ecuResetMsg, sense.CAB500.ecuResetMsg_send_soft]
        sendStdCANmessage(msg_id, data_msg, debug)


def readDatabyIdentifierID(msg_id, debug):
    data_msg = [sense.CAB500.SINGLE_FRAME_3_BYTE, sense.CAB500.readDataById, sense.CAB500.subf_CAN_ID >> 8, sense.CAB500.subf_CAN_ID & 255]
    sendStdCANmessage(msg_id, data_msg, debug)


def readDatabyIdentifierFilerfreq(msg_id, debug):
    # print("readData filter msg id: ", hex(msg_id))
    data_msg = [sense.CAB500.SINGLE_FRAME_3_BYTE, sense.CAB500.readDataById, sense.CAB500.subf_CAN_ID >> 8, sense.CAB500.subf_FilterFreq & 255]
    sendStdCANmessage(msg_id, data_msg, debug)


def readDatabyIdentifierFramefreq(msg_id, debug):
    # print("readData frame freq msg id: ", hex(msg_id))
    data_msg = [sense.CAB500.SINGLE_FRAME_3_BYTE, sense.CAB500.readDataById, sense.CAB500.subf_CAN_ID >> 8, sense.CAB500.subf_FrameFreq & 255]
    sendStdCANmessage(msg_id, data_msg, debug)


def readDatabyIdentifierCANspeed(msg_id, debug):
    # print("readData CAN speed msg id: ", hex(msg_id))
    data_msg = [sense.CAB500.SINGLE_FRAME_3_BYTE, sense.CAB500.readDataById, sense.CAB500.subf_CAN_ID >> 8, sense.CAB500.subf_CANspeed & 255]
    sendStdCANmessage(msg_id, data_msg, debug)


def printInfoCAB500():
    global cab500IpID, udsClientID, udsServerID
    print_hi('Done, udsClientID: ' + hex(udsClientID) + ' , udsServerID: ' +
             hex(udsServerID) + ' and cab500IpID: ' + hex(cab500IpID))
    time.sleep(0.001)
    readDatabyIdentifierFilerfreq(udsClientID, debugging)
    time.sleep(0.001)
    readDatabyIdentifierCANspeed(udsClientID, debugging)
    time.sleep(0.001)
    readDatabyIdentifierFramefreq(udsClientID, debugging)
    time.sleep(0.001)


def flowControl(msg_id, debug):
    data_msg = [sense.CAB500.flowControl, 0x00, 0x00]
    sendStdCANmessage(msg_id, data_msg, debug)


def writeDatabyIdentifier(msg_id, debug):
    if msg_id == 0:
        if debug:
            print("Message not sent, msg_id = 0")
    else:
        data_msg = [sense.CAB500.firstMessage, sense.CAB500.SINGLE_FRAME_9_BYTE, sense.CAB500.writeDataById, sense.CAB500.subf_CAN_ID >> 8,
                    sense.CAB500.subf_CAN_ID & 255, cab500IpIDnew >> 8, cab500IpIDnew & 255, udsClientIDnew >> 8]
        sendStdCANmessage(msg_id, data_msg, debug)


def writeDatabyIdentifier2nd(msg_id, debug):
    data_msg = [sense.CAB500.secondMessage, udsClientIDnew & 255, udsServerIDnew >> 8, udsServerIDnew & 255]
    sendStdCANmessage(msg_id, data_msg, debug)


def writeDatasubFunctionFilterFreq(msg_id, filter_frequency, debug):
    if msg_id == 0:
        if debug:
            print("Message not sent, msg_id = 0")
    else:
        data_msg = [sense.CAB500.SINGLE_FRAME_4_BYTE, sense.CAB500.writeDataById, sense.CAB500.subf_FilterFreq >> 8,
                    sense.CAB500.subf_FilterFreq & 255, filter_frequency]
        sendStdCANmessage(msg_id, data_msg, debug)


def writeDatasubFunctionCANspeed(msg_id, can_speed, debug):
    if msg_id == 0:
        if debug:
            print("Message not sent, msg_id = 0")
    else:
        data_msg = [sense.CAB500.SINGLE_FRAME_4_BYTE, sense.CAB500.writeDataById, sense.CAB500.subf_CANspeed >> 8,
                    sense.CAB500.subf_CANspeed & 255, can_speed]
        sendStdCANmessage(msg_id, data_msg, debug)


def writeDatasubFunctionFramePeriod(msg_id, frame_frequency, debug):
    if msg_id == 0:
        if debug:
            print("Message not sent, msg_id = 0")
    else:
        data_msg = [sense.CAB500.SINGLE_FRAME_5_BYTE, sense.CAB500.writeDataById, sense.CAB500.subf_FrameFreq >> 8,
                    sense.CAB500.subf_FrameFreq & 255, frame_frequency >> 8, frame_frequency & 255]
        sendStdCANmessage(msg_id, data_msg, debug)

class CanInterface:

    def __init__(self, node_id, channel='can0', bustype='socketcan', bitrate='500000'):
        self.node_id = node_id
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.accept_virtual = True


        # TODO move to function to connect to network
        # TODO add function to disconnect network.disconnect()
        # Try to find a CAN-Bus interface
        for interface, channel in [('kvaser', 0), ('kvaser', 1), ('socketcan', 'can0'), ('pcan', 'PCAN_USBBUS1'),
                                   ('ixxat', 0)]:
            try:
                self._can_bus = can.ThreadSafeBus(interface=interface, channel=channel, bitrate=self.bitrate)
                break
            except (OSError, can.CanError, NameError, ImportError):
                pass
        else:
            raise Exception('No CAN-Bus interface was found')
        self.ch = self._can_bus

    def add_can_msg_callback(self):
        pass

    def remove_can_msg_callback(self):
        pass

    def close_bus(self):
        # self._can_notifier.stop(timeout=1)
        self._can_bus.shutdown()


def receive_can_data(msg):
    global receivedCorrectMsg, cab500IpID, udsClientID, udsServerID, receivedReadDatabyIDDone, msgID

    # print(f"Recieved data All: {msg.data}, id: {hex(msg.arbitration_id)}")
    if msg.is_rx == True and msg.dlc == 8:
        if debugging:
            print("Message recieved and dlc=8")
            print(msg.data)
        if msg.data[0] == sense.CAB500.firstMessage:  # First frame of message
            # print("Message recieved and dlc=8 and first msg")
            # print(msg.data)
            if msg.data[1] == sense.CAB500.nineUsableData:
                if msg.data[2] == sense.CAB500.posResponseRead:
                    # print(f"Positive response")
                    if int.from_bytes(msg.data[3:5]) == sense.CAB500.subf_CAN_ID:
                        receivedCorrectMsg = True
                        # print("correct msg state: ", receivedCorrectMsg)
                        flowControl(msgID, debugging)
                        cab500IpID = int.from_bytes(msg.data[5:7])
                        # print("flowcontrol is sent: ", id)
                        udsClientID = msg.data[7]
                else:
                    print("Negative response, wrong format" + msg.data)

        elif msg.data[0] == sense.CAB500.secondMessage:
            # print("Second message received")
            udsClientID = int.from_bytes([udsClientID, msg.data[1]])
            udsServerID = int.from_bytes(msg.data[2:4])
            printInfoCAB500()
            receivedReadDatabyIDDone = True
        elif msg.data[0] == sense.CAB500.flowControl and \
                int.from_bytes([msg.data[1], msg.data[2], msg.data[3],
                                msg.data[4], msg.data[5], msg.data[6],
                                msg.data[7]]) == 0:
            # print("flow control message received")
            writeDatabyIdentifier2nd(udsClientID, debugging)
        elif msg.data[0] == sense.CAB500.SINGLE_FRAME_3_BYTE and msg.arbitration_id == udsServerID:
            if msg.data[1] == sense.CAB500.posResponseWrite:
                print("Change accepted, subfunction: ")
                print(hex(msg.data[2]), hex(msg.data[3]))
            elif msg.data[1] == sense.CAB500.negResponse:
                print("Negative response, subfunction: ")
                print(hex(msg.data[2]))
            else:
                print("Error, WriteDataByIdentifier")
                print(hex(msg.data[1]))
                print(hex(msg.data[2]))


def receive_can_data_single(msg):
    global receivedCorrectMsg, cab500IpID, udsClientID, udsServerID, receivedReadDatabyIDDone
    # print(f"Recieved data single 1: {msg.data}, id: {hex(msg.arbitration_id)}")
    if msg.is_rx == True and msg.dlc == 8:
        if msg.data[0] == sense.CAB500.SINGLE_FRAME_4_BYTE:  # and msg.arbitration_id == udsServerID:
            # print(f"Recieved data single 2: {msg.data}, id: {hex(msg.arbitration_id)}")
            if msg.data[1] == sense.CAB500.posResponseRead:
                # print("Read data by ID, subfunction: ")
                # print(hex((msg.data[2] << 8) + msg.data[3]))
                if (msg.data[2] << 8) + msg.data[3] == sense.CAB500.subf_FilterFreq:
                    print("      UDS Server ID: ", hex(msg.arbitration_id), ", subFilterFreq: ", freq_dict[msg.data[4]])
        if msg.data[0] == sense.CAB500.SINGLE_FRAME_5_BYTE:  # and msg.arbitration_id == udsServerID:
            if msg.data[1] == sense.CAB500.posResponseRead:
                # print("Read data by ID, subfunction: ")
                # print(hex((msg.data[2] << 8) + msg.data[3]))
                if ((msg.data[2] << 8) + msg.data[3]) == sense.CAB500.subf_CANspeed:
                    print("      UDS Server ID: ", hex(msg.arbitration_id), ", subCANspeed: ",
                          canSpeed_dict[(msg.data[4] << 8) + msg.data[5]])
                elif ((msg.data[2] << 8) + msg.data[3]) == sense.CAB500.subf_FrameFreq:
                    print("      UDS Server ID: ", hex(msg.arbitration_id), ", subCAN frame period: ", msg.data[5], "ms")

            else:
                print("Error, ReadDataByIdentifier")
                print(hex(msg.data[1]))
                print(hex(msg.data[2]))


if __name__ == '__main__':
    globals()
    parser = argparse.ArgumentParser(
        description="Script to change config of the LEM CAB500 sensor"
    )
    parser.add_argument("--udsClientID", required=False, type=auto_int, default=0x3C2, metavar="Current UDS client ID")
    parser.add_argument("--subF", required=False, type=str, default="nil", metavar="subFunctions: canID, filterFreq,"
                                                                                   " CANspeed, framePeriod and reboot.")
    parser.add_argument("--clientID", required=False, type=int, default=0x0000, metavar="New udsClient ID")
    parser.add_argument("--serverID", required=False, type=int, default=0x0000, metavar="New udsServer ID")
    parser.add_argument("--IpID", required=False, type=int, default=0x0000, metavar="New Ip message ID")
    args = parser.parse_args()

    msgID = args.udsClientID
    udsClientIDnew = args.clientID
    udsServerIDnew = args.serverID
    cab500IpIDnew = args.IpID
    subFunc = args.subF


    print_hi('Start program')
    bus = CanInterface(0x11)
    logger = can.SizedRotatingLogger(
        base_filename="data/logger_CAB500.csv",
        max_bytes=1 * 1024 ** 2,  # =2MB
        append=True
    )
    listeners: List[MessageRecipient] = [
        receive_can_data,  # Callback function
        receive_can_data_single,
        logger,  # Regular Listener object
    ]

    notifier = can.Notifier(bus.ch, listeners)
    time.sleep(0.01)
    readDatabyIdentifierID(msgID, debugging)
    #print_hi("message sent")
    time.sleep(0.02)
    #print_hi("Waited")
    if receivedReadDatabyIDDone:
        # print(hex(udsClientID))
        receivedReadDatabyIDDone = False
    else:
        print("No CAB500 found")
        run_test = input("Run test program: [Y/N] ")
        if run_test == "Y":
            msgID = 0x03C2
        else:
            exit(1)


        # writeDatabyIdentifier(udsClientID, debugging)
        # ecuResetService(udsClientID, debugging)

    match subFunc:
        case "canID":
            print_hi(subFunc)
            if all(x in idList for x in [udsClientIDnew, udsServerIDnew, cab500IpIDnew]):
                writeDatabyIdentifier(msgID, debugging)
            else:
                print("Values not in valid set 0x221-0x400 and 0x40B-0x7F1")

        case "filterFreq":
            print_hi(subFunc)
            filterF = input("Select filter frequency: 1: IIR10_HZ, 2: IIR20_HZ, 3: IIR30_HZ, 4: IIR40_HZ, 5: IIR50_HZ, "
                  "6: IIR60_HZ, 7: IIR70_HZ, 8: IIR80_HZ, 9: IIR90_HZ, 10: IIR100_HZ, 11: IIR110_HZ, 12: IIR120_HZ, "
                  "13: IIR130_HZ, 14: IIR140_HZ, 15: IIR150_HZ, 16: IIR160_HZ, 255: AVERAGE_TEN_MS: ")
            print(auto_int(filterF))
            if all(x in filterFreqList for x in [auto_int(filterF)]):
                writeDatasubFunctionFilterFreq(msgID, auto_int(filterF), debugging)
            else:
                print("Values not in valid set")

        case "CANspeed":
            print_hi(subFunc)
            canSpeed = input("Select CAN speed: 125/250/500 [kBaud] ")
            if all(x in [125, 250, 500] for x in [auto_int(canSpeed)]):
                writeDatasubFunctionCANspeed(msgID, auto_int(canSpeed), debugging)
            else:
                print("Values not in valid set")

        case "framePeriod":
            print_hi(subFunc)
            canSpeed = input("Select frame period: 10-100 [ms] ")
            if all(x in framePeriodList for x in [auto_int(canSpeed)]):
                writeDatasubFunctionCANspeed(msgID, auto_int(canSpeed), debugging)
            else:
                print("Values not in valid set")
        case "reboot":
            ecuResetService(msgID, debugging)
            print_hi("Rebooting: " + str(msgID))
            time.sleep(0.5)

        case _:
            print_hi("No subfunction matched: canID, filterFreq, CANspeed, framePeriod and reboot to choose from.")
            print_hi("Not valid: " + subFunc)

    time.sleep(0.02)
    readDatabyIdentifierID(msgID, debugging)
    time.sleep(0.02)
    bus.close_bus()
    print_hi('Stop program')
