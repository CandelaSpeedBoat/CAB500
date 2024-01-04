# Definitions for LEM CAB500 Current sensor
#
# Written by Björn Magnusson Westin, magnusson.bjorn@gmail.com, 2023-2024
# Used as base to configure the sensor

from enum import IntEnum


class CAB500(IntEnum):
    udsClientID = 0x0000
    udsServerID = 0x0000
    cab500IpID = 0x0000
    # receivedCorrectMsg: bool = False
    # receivedReadDatabyIDDone: bool = False
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

    # freq_dict = {"IIR10_HZ": 0x01, "IIR20_HZ": 0x02, "IIR30_HZ": 0x03, "IIR40_HZ": 0x04, "IIR50_HZ": 0x05,
    #            "IIR60_HZ": 0x06, "IIR70_HZ": 0x07, "IIR80_HZ": 0x08, "IIR90_HZ": 0x09, "IIR100_HZ": 0x0A,
    #           "IIR110_HZ": 0x0B, "IIR120_HZ": 0x0C, "IIR130_HZ": 0x0D, "IIR140_HZ": 0x0E,
    #          "IIR150_HZ": 0x0F, "IIR160_HZ": 0x10, "AVERAGE_TEN_MS": 0xFF}
    # freq_dict = {1: 'IIR10_HZ', 2: 'IIR20_HZ', 3: 'IIR30_HZ', 4: 'IIR40_HZ', 5: 'IIR50_HZ', 6: 'IIR60_HZ',
    #             7: 'IIR70_HZ', 8: 'IIR80_HZ', 9: 'IIR90_HZ', 10: 'IIR100_HZ', 11: 'IIR110_HZ', 12: 'IIR120_HZ',
    #             13: 'IIR130_HZ', 14: 'IIR140_HZ', 15: 'IIR150_HZ', 16: 'IIR160_HZ', 255: 'AVERAGE_TEN_MS'}

    # CAN speed ie. baud rate
    baudrate_125k = 0x007D
    baudrate_250k = 0x00FA
    baudrate_500k = 0x01F4
