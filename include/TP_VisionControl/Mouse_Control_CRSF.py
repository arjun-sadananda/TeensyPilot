#!/usr/bin/env python3

#
# https://github.com/crsf-wg/crsf/wiki
# This simple python parser implements a loose CRSF implementation. 
# Strict mode would require the packet start with CRSF_SYNC (0xC8) 
# but this ignores that byte and processes any packet that's 4-64 bytes long 
# and the CRC checks out. This is similar to the parsers in EdgeTX (2.9), iNav (7.0), and Betaflight (4.5).

# Note the default baud rate is arbitrarily chosen for testing. 
# Use 420k/416666 for receivers, or 400k+ etc for transmitters. 
# Half-duplex inverted (S.PORT / external module protocol) is not supported.

# Add the --tx option to also send CHANNELS_PACKED at 50Hz. The channel values will all be 1500us.

import serial
import time
import argparse
from enum import IntEnum

import pyautogui

# from tkinter.tix import MAX

CRSF_SYNC = 0xC8

class PacketsTypes(IntEnum):
    GPS = 0x02
    VARIO = 0x07
    BATTERY_SENSOR = 0x08
    BARO_ALT = 0x09
    HEARTBEAT = 0x0B
    VIDEO_TRANSMITTER = 0x0F
    LINK_STATISTICS = 0x14
    RC_CHANNELS_PACKED = 0x16
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_INFO = 0x29
    CONFIG_READ = 0x2C
    CONFIG_WRITE = 0x2D
    RADIO_ID = 0x3A

def crc8_dvb_s2(crc, a) -> int:
  crc = crc ^ a
  for ii in range(8):
    if crc & 0x80:
      crc = (crc << 1) ^ 0xD5
    else:
      crc = crc << 1
  return crc & 0xFF

def crc8_data(data) -> int:
    crc = 0
    for a in data:
        crc = crc8_dvb_s2(crc, a)
    return crc

def crsf_validate_frame(frame) -> bool:
    return crc8_data(frame[2:-1]) == frame[-1]

def signed_byte(b):
    return b - 256 if b >= 128 else b

def packCrsfToBytes(channels) -> bytes:
    # channels is in CRSF format! (0-1984)
    # Values are packed little-endianish such that bits BA987654321 -> 87654321, 00000BA9
    # 11 bits per channel x 16 channels = 22 bytes
    if len(channels) != 16:
        raise ValueError('CRSF must have 16 channels')
    result = bytearray()
    destShift = 0
    newVal = 0
    for ch in channels:
        # Put the low bits in any remaining dest capacity
        newVal |= (ch << destShift) & 0xff
        result.append(newVal)

        # Shift the high bits down and place them into the next dest byte
        srcBitsLeft = 11 - 8 + destShift
        newVal = ch >> (11 - srcBitsLeft)
        # When there's at least a full byte remaining, consume that as well
        if srcBitsLeft >= 8:
            result.append(newVal & 0xff)
            newVal >>= 8
            srcBitsLeft -= 8

        # Next dest should be shifted up by the bits consumed
        destShift = srcBitsLeft

    return result

def channelsCrsfToChannelsPacket(channels) -> bytes:
    result = bytearray([CRSF_SYNC, 24, PacketsTypes.RC_CHANNELS_PACKED]) # 24 is packet length
    result += packCrsfToBytes(channels)
    result.append(crc8_data(result[2:]))
    return result

def handleCrsfPacket(ptype, data):
    if ptype == PacketsTypes.RADIO_ID and data[5] == 0x10:
        #print(f"OTX sync")
        pass
    elif ptype == PacketsTypes.LINK_STATISTICS:
        rssi1 = signed_byte(data[3])
        rssi2 = signed_byte(data[4])
        lq = data[5]
        snr = signed_byte(data[6])
        antenna = data[7]
        mode = data[8]
        power = data[9]
        # telemetry strength
        downlink_rssi = signed_byte(data[10])
        downlink_lq = data[11]
        downlink_snr = signed_byte(data[12])
        print(f"RSSI={rssi1}/{rssi2}dBm LQ={lq:03} mode={mode}") # ant={antenna} snr={snr} power={power} drssi={downlink_rssi} dlq={downlink_lq} dsnr={downlink_snr}")
    elif ptype == PacketsTypes.ATTITUDE:
        pitch = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10000.0
        roll = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10000.0
        yaw = int.from_bytes(data[7:9], byteorder='big', signed=True) / 10000.0
        print(f"Attitude: Pitch={pitch:0.2f} Roll={roll:0.2f} Yaw={yaw:0.2f} (rad)")
    elif ptype == PacketsTypes.FLIGHT_MODE:
        packet = ''.join(map(chr, data[3:-2]))
        print(f"Flight Mode: {packet}")
    elif ptype == PacketsTypes.BATTERY_SENSOR:
        vbat = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
        curr = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10.0
        mah = data[7] << 16 | data[8] << 7 | data[9]
        pct = data[10]
        print(f"Battery: {vbat:0.2f}V {curr:0.1f}A {mah}mAh {pct}%")
    elif ptype == PacketsTypes.BARO_ALT:
        print(f"BaroAlt: ")
    elif ptype == PacketsTypes.DEVICE_INFO:
        packet = ' '.join(map(hex, data))
        print(f"Device Info: {packet}")
    elif data[2] == PacketsTypes.GPS:
        lat = int.from_bytes(data[3:7], byteorder='big', signed=True) / 1e7
        lon = int.from_bytes(data[7:11], byteorder='big', signed=True) / 1e7
        gspd = int.from_bytes(data[11:13], byteorder='big', signed=True) / 36.0
        hdg =  int.from_bytes(data[13:15], byteorder='big', signed=True) / 100.0
        alt = int.from_bytes(data[15:17], byteorder='big', signed=True) - 1000
        sats = data[17]
        print(f"GPS: Pos={lat} {lon} GSpd={gspd:0.1f}m/s Hdg={hdg:0.1f} Alt={alt}m Sats={sats}")
    elif ptype == PacketsTypes.VARIO:
        vspd = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
        print(f"VSpd: {vspd:0.1f}m/s")
    elif ptype == PacketsTypes.RC_CHANNELS_PACKED:
        #print(f"Channels: (data)")
        pass
    else:
        packet = ' '.join(map(hex, data))
        print(f"Unknown 0x{ptype:02x}: {packet}")


e_x_prev = 0.0
e_y_prev = 0.0
prev_time = time.time()

# 1811+172 = 1639/2 = 992
# 1811-992 = 819
RC_STICK_MIN = 172
RC_STICK_MAX = 1811
RC_STICK_RANGE = RC_STICK_MAX - RC_STICK_MIN
RC_STICK_MID = (RC_STICK_MIN + RC_STICK_MAX) // 2

ANGLE_LIMIT = 55.0  # degrees
CAMERA_ANGLE = 25.0  # degrees

# 848, 678
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080
# Ideally Use LOS instead of pixel errors directly
e_y_integral = 0.0

def controller(e_x, e_y):
    global e_x_prev, e_y_prev, prev_time, e_y_integral

    dt = time.time() - prev_time  # Time step, adjust as needed
    prev_time = time.time()

    d_e_x = (e_x - e_x_prev)/dt
    d_e_y = (e_y - e_y_prev)/dt

    e_x_prev = e_x
    e_y_prev = e_y

    if abs(e_y) < .25:
        e_y_integral += e_y * dt  # Simple integral term, could be accumulated over time
    else:
        e_y_integral = 0.0  # Reset integral if error is large
    Kp_roll     = RC_STICK_RANGE*1
    Kp_yaw_rate = RC_STICK_RANGE*1
    Kp_pitch    = RC_STICK_RANGE*1
    Kd_roll     = 0.0
    Kd_yaw_rate = 0.0
    Kd_pitch    = 0.0
    Ki_throttle = RC_STICK_RANGE*1


    HOVER_THROTTLE = RC_STICK_MIN + (RC_STICK_RANGE * 0.45)  # Adjust as needed
    PITCH_BIAS = RC_STICK_RANGE/2 * (CAMERA_ANGLE / ANGLE_LIMIT)

    # Total control outputs
    u_roll      = RC_STICK_MID + Kp_roll * e_x      + Kd_roll * d_e_x
    u_yaw_rate  = RC_STICK_MID + Kp_yaw_rate * e_x  + Kd_yaw_rate * d_e_x
    u_pitch     = RC_STICK_MID + Kp_pitch * e_y     + Kd_pitch * d_e_y      + PITCH_BIAS
    u_throttle  = HOVER_THROTTLE - Kp_pitch * e_y   + Kd_pitch * d_e_y    - Ki_throttle * e_y_integral
    # Clip to valid range
    u_roll      = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_roll)))
    u_yaw_rate  = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_yaw_rate)))
    u_pitch     = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_pitch)))
    u_throttle  = max(RC_STICK_MIN, min(RC_STICK_MAX, int(u_throttle)))

    return u_roll, u_pitch, u_throttle, u_yaw_rate


parser = argparse.ArgumentParser()
parser.add_argument('-P', '--port', default='COM19', required=False)
parser.add_argument('-b', '--baud', default=921600, required=False)
parser.add_argument('-t', '--tx', required=False, default=True, action='store_true',
                    help='Enable sending CHANNELS_PACKED every 20ms (all channels 1500us)')
args = parser.parse_args()


# time_passed = time.time()
# start_time = time.time()

# time_period = 2.0 # seconds for full sweep

with serial.Serial(args.port, args.baud, timeout=2) as ser:
    input = bytearray()
    while True:
        
        # sweeping values for testing
        # roll = int(MIN + (MAX - MIN) * ((time.time() - start_time) % time_period) / time_period)
        # pitch = int(MIN + (MAX - MIN) * ((time.time() - start_time) % time_period) / time_period)
        # throttle = int(MIN + (MAX - MIN) * ((time.time() - start_time) % time_period) / time_period)
        # yaw = int(MIN + (MAX - MIN) * ((time.time() - start_time) % time_period) / time_period)

        # pitch = 992
        # throttle = 1200
        
        x, y = pyautogui.position()
        
        e_x = x - FRAME_WIDTH / 2
        e_x /= (FRAME_WIDTH / 2)
        e_y = (y - FRAME_HEIGHT / 2)
        e_y /= (FRAME_HEIGHT / 2)

        roll, pitch, throttle, yaw_rate = controller(e_x, e_y)

        # print(f"Mouse cursor is at X: {x}, Y: {y}")
        # print(f"Error values are: e_x: {e_x}, e_y: {e_y}")
        if ser.in_waiting > 0:
            input.extend(ser.read(ser.in_waiting))
            
        else:
            if args.tx:
                ser.write(channelsCrsfToChannelsPacket([roll, pitch, throttle, yaw_rate] + [992 for ch in range(12)]))
                print("Sent CHANNELS_PACKED")
            time.sleep(0.020)
        # print(f"Buffer len: {len(input)}")
        while len(input) > 2:
            # This simple parser works with malformed CRSF streams
            # it does not check the first byte for SYNC_BYTE, but
            # instead just looks for anything where the packet length
            # is 4-64 bytes, and the CRC validates
            # print(f"Buffer len: {len(input)}")
            expected_len = input[1] + 2
            if expected_len > 64 or expected_len < 4:
                input = bytearray()
            elif len(input) >= expected_len:
                single = input[:expected_len] # copy out this whole packet
                input = input[expected_len:] # and remove it from the buffer

                if not crsf_validate_frame(single): # single[-1] != crc:
                    packet = ' '.join(map(hex, single))
                    print(f"crc error: {packet}")
                else:
                    handleCrsfPacket(single[2], single)
            else:
                break