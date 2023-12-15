import sensorCAB
import can

debugging = False
msgID = 0x06A0

sensor = sensorCAB.CAB500(0x11)
notifier = can.Notifier(sensor.ch, [sensor.receive_can_data])

print(hex(sensorCAB.CAB500.subf_FilterFreq))
sensor.readDatabyIdentifierID(msgID)

