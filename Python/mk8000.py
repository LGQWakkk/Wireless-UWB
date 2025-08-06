# 20250802 Wakkk
import serial
from serial.tools import list_ports
import struct
import numpy as np

def find_serial_port():
    available_ports = list_ports.comports()
    if not available_ports:
        print("No available serial ports found")
        return None
    first_port = available_ports[0]
    print(f"Using Serial Port: {first_port.device}")
    try:
        ser = serial.Serial(first_port.device, baudrate=115200, timeout=1)
        print("Serial Port Opened")
        return ser
    except serial.SerialException as e:
        print(f"Error Opening Serial Port: {e}")
        return None

# MK8000 数据包格式
# [帧头0xF0] [有效数据长度0x05] [发送地址2Byte] [距离2Byte] [信号强度 1Byte] [帧尾0xAA]
# 帧总长度为8 Bytes
uwb_data = []
def parse_mk8000_data(data):
    if data[0] != 0xF0 or data[1] != 0x05 or data[7] != 0xAA: # 校验固定字节是否正确
        return False

    raw_address = struct.unpack('<H', data[2:4])[0] # (uint16_t)buffer[3]<<8)|((uint16_t)buffer[2])
    raw_distance = struct.unpack('<H', data[4:6])[0] # (uint16_t)buffer[5]<<8)|((uint16_t)buffer[4])
    raw_rssi = data[6]
    distance = raw_distance / 100.0 # 距离单位m
    rssi = raw_rssi - 256 # 信号强度单位dBm(-256~-1)
    uwb_data.append([raw_address, distance, rssi])
    print(f"Address: {raw_address}, Distance: {distance:.2f}m, RSSI: {rssi}dBm")
    return True

MK8000_FRAME_LEN = 8
serial_port = find_serial_port()
if serial_port:
    data_packet_size = MK8000_FRAME_LEN
    while True:
        data = serial_port.read(data_packet_size)
        if len(data) == data_packet_size:
            if parse_mk8000_data(data):
                pass
            else:
                print("Invalid Data Received")
                serial_port.reset_input_buffer()
        else:
            print("Invalid Data Received")
            serial_port.reset_input_buffer()
        
        # # Check Data
        # data_len = len(gps_data)
        # if data_len > 1000:
        #     print("Data List Length:", data_len)
        #     data_list_np = np.array(gps_data)
        #     np.save("gps.npy", data_list_np)
        #     break 
    print("Data Saved")
    serial_port.close()
