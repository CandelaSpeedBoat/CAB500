import time
import numpy as np
import can
import logging
import os
from can.notifier import MessageRecipient
from typing import List
import sensorCAB as sense

idList1 = np.arange(0x40B, 0x7F1, 1)
idList2 = np.arange(0x221, 0x400, 1)
idList = np.append(idList1, idList2)
# idList = np.arange(0x3F0, 0x3F1, 1)
receivedCorrectMsg = False
receivedReadDatabyIDDone = False
udsClientID = 0x0000  #
udsServerID = 0x0000  #
cab500IpID = 0x0000  #
cab500IpIDnew = 0x06AC  #
udsServerIDnew = 0x06AD  #
udsClientIDnew = 0x06AE  #
id = 0

debugging = False  # If true, extra debug message is printed

freq_dict = {1: 'IIR10_HZ', 2: 'IIR20_HZ', 3: 'IIR30_HZ', 4: 'IIR40_HZ', 5: 'IIR50_HZ', 6: 'IIR60_HZ',
             7: 'IIR70_HZ', 8: 'IIR80_HZ', 9: 'IIR90_HZ', 10: 'IIR100_HZ', 11: 'IIR110_HZ', 12: 'IIR120_HZ',
             13: 'IIR130_HZ', 14: 'IIR140_HZ', 15: 'IIR150_HZ', 16: 'IIR160_HZ', 255: 'AVERAGE_TEN_MS'}
canSpeed_dict = {0x007D: "baudrate_125k", 0x00FA: "baudrate_250k", 0x01F4: "baudrate_500k"}


def print_hi(name):
    print(f'Msg: {name}')


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
    logging.info(time.strftime("%H%M%S") + "," + str(hex(udsClientID)) + ",cab500IpID," + str(hex(cab500IpID)))
    logging.info(time.strftime("%H%M%S") + "," + str(hex(udsClientID)) + ",udsServerID," + str(hex(udsServerID)))
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


class CanInterface:

    def __init__(self, node_id, channel='can0', bustype='socketcan', bitrate='500000'):
        self.node_id = node_id
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        # self.accept_virtual = True # Only for kvaser interface as far as I know

        # Try to find a CAN-Bus interface
        for interface, channel in [('socketcan', 'can5'), ('pcan', 'PCAN_USBBUS1'),
                                   ('ixxat', 0)]:
            try:
                self._can_bus = can.ThreadSafeBus(interface=interface, channel=channel, bitrate=self.bitrate)
                break
            except (OSError, can.CanError, NameError):
                pass
        else:
            raise Exception('No CAN-Bus interface was found')
        self.ch = self._can_bus
        print("CAN-nterface: ", interface, " and channel: ", channel, " chosen.")

    def add_can_msg_callback(self):
        pass

    def remove_can_msg_callback(self):
        pass

    def close_bus(self):
        # self._can_notifier.stop(timeout=1)
        self._can_bus.shutdown()


def receive_can_data(msg):
    global receivedCorrectMsg, cab500IpID, udsClientID, udsServerID, receivedReadDatabyIDDone, id

    # print(f"Recieved data All: {msg.data}, id: {hex(msg.arbitration_id)}")
    if msg.is_rx == True and msg.dlc == 8:
        if debugging:
            # print("Message recieved and dlc=8")
            # print(msg.data)
          if msg.data[0] == sense.CAB500.firstMessage:  # First frame of message
            # print("Message recieved and dlc=8 and first msg")
            # print(msg.data)
            if msg.data[1] == sense.CAB500.nineUsableData:
                if msg.data[2] == sense.CAB500.posResponseRead:
                    print(f"Positive response")
                    if int.from_bytes(msg.data[3:5],'big') == sense.CAB500.subf_CAN_ID:
                        receivedCorrectMsg = True
                        print("correct msg state: ", receivedCorrectMsg)
                        flowControl(id, debugging)
                        cab500IpID = int.from_bytes(msg.data[5:7],'big')
                        print("flowcontrol is sent: ", id)
                        udsClientID = msg.data[7]
                else:
                    print("Negative response, wrong format" + msg.data)

        elif msg.data[0] == sense.CAB500.secondMessage:
            # print("Second message received")
            udsClientID = int.from_bytes([udsClientID, msg.data[1]],'big')
            udsServerID = int.from_bytes(msg.data[2:4],'big')
            printInfoCAB500()
            receivedReadDatabyIDDone = True
        elif msg.data[0] == sense.CAB500.flowControl and \
                int.from_bytes([msg.data[1], msg.data[2], msg.data[3],
                                msg.data[4], msg.data[5], msg.data[6],
                                msg.data[7]],'big') == 0:
            print("flow control message received")
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
                    print("UDS Server ID: ", hex(msg.arbitration_id), ", subFilterFreq: ", freq_dict[msg.data[4]])
                    logging.info(time.strftime("%H%M%S") + "," + str(hex(udsClientID)) + ",subFilterFreq," + freq_dict[msg.data[4]])
        if msg.data[0] == sense.CAB500.SINGLE_FRAME_5_BYTE:  # and msg.arbitration_id == udsServerID:
            if msg.data[1] == sense.CAB500.posResponseRead:
                # print("Read data by ID, subfunction: ")
                # print(hex((msg.data[2] << 8) + msg.data[3]))
                if ((msg.data[2] << 8) + msg.data[3]) == sense.CAB500.subf_CANspeed:
                    print("UDS Server ID: ", hex(msg.arbitration_id), ", subCANspeed: ",
                          canSpeed_dict[(msg.data[4] << 8) + msg.data[5]])
                    logging.info(time.strftime("%H%M%S") + "," + str(hex(udsClientID)) + ",subCANspeed," + canSpeed_dict[(msg.data[4] << 8) + msg.data[5]])
                elif ((msg.data[2] << 8) + msg.data[3]) == sense.CAB500.subf_FrameFreq:
                    print("UDS Server ID: ", hex(msg.arbitration_id), ", subFramePeriod: ", msg.data[5], "ms")
                    logging.info(time.strftime("%H%M%S") + "," + str(hex(udsClientID)) + ",subFramePeriod," + str(msg.data[5]) + "ms")

            else:
                print("Error, ReadDataByIdentifier")
                print(hex(msg.data[1]))
                print(hex(msg.data[2]))


if __name__ == '__main__':
    globals()

    print_hi('Start program')
    print_hi(f'Min: {hex(min(idList))}, Max: {hex(max(idList))}')
    if os.path.isdir("data"):
        filename = "data/" + time.strftime("%Y%m%d") + "_discoveredSensors.log"
        print("Logging")
        logging.basicConfig(filename=filename, level=logging.INFO, style='{', datefmt='%Y-%m-%d %H:%M:%S',
                            format='{asctime} {levelname} {filename}:{lineno}: {message}')
        logging.info(["Time", "msgID", "subfunction", "value"])
    else:
        print("No folder to keep log data in, no logs will be written")

    bus = CanInterface(0x11)
    if os.path.isdir("data"):
        b_filename = "data/logger_CAB500.csv"
    else:
        b_filename = "logger_CAB500.csv"
    logger = can.SizedRotatingLogger(
        base_filename=b_filename,
        max_bytes=1 * 1024 ** 2,  # =2MB
        append=True
    )
    listeners: List[MessageRecipient] = [
        receive_can_data,  # Callback function
        receive_can_data_single,
        logger  # Regular Listener object
    ]

    notifier = can.Notifier(bus.ch, listeners)
    wait = 0

    for id in idList:
        readDatabyIdentifierID(id, debugging)
        while (wait < 2) and not receivedCorrectMsg:
            # print("CorrectMsg state:",(not receivedCorrectMsg), "Wait: ", wait)
            ok = receivedCorrectMsg
            time.sleep(0.001)
            wait += 1
        if receivedCorrectMsg:
            receivedCorrectMsg = False
            wait = 0
        while (wait < 2) and not receivedReadDatabyIDDone:
            # print("ID Done state:",(not receivedReadDatabyIDDone), "Wait: ", wait)
            ok = receivedReadDatabyIDDone
            time.sleep(0.001)
            wait += 1
        if receivedReadDatabyIDDone:
            receivedReadDatabyIDDone = False
        wait = 0

    time.sleep(0.01)
    bus.close_bus()
    print_hi('Stop program')
