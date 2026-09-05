#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
小车执行板（STM32 阿克曼） UART 帧 测试/发帧辅助脚本
帧协议：AA 55 LEN DEV CMD [PAYLOAD] CRC16(2B,低在前)
CRC：CRC-16/MODBUS（poly 反射 0xA001，初值 0xFFFF），输入 = LEN..PAYLOAD

用法（命令行）：
  python uart_test.py selfcheck
  python uart_test.py --port COM5 car-move-fwd 200
  python uart_test.py --port COM5 car-stop-all
  python uart_test.py --port COM5 car-query
  python uart_test.py --port COM5 arm-query
  python uart_test.py --port COM5 car-move-dist 30 200
  python uart_test.py --port COM5 car-turn-angle 30 150 打印收到的状态帧

串口选项：--baud 115200
"""
import sys
import time
# serial 仅在用到串口时再导入（selfcheck 不需 pyserial）

DEV_CAR = 0x01
DEV_ARM = 0x02
CMD_STATUS = 0x0A

CMD = {
    'car': {
        0x01: '持续移动', 0x02: '移动指定距离', 0x03: '持续转动',
        0x04: '转动指定角度', 0x05: '停止', 0x06: '状态查询',
    },
    'arm': {
        0x01: '持续升降', 0x02: '升降指定距离', 0x03: '持续前后移爪',
        0x04: '前后移爪指定距离', 0x05: '夹取/松开', 0x06: '状态查询',
    },
}


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc


def make_frame(dev, cmd, payload=b''):
    body = bytes([dev, cmd]) + bytes(payload)
    length = len(body)
    crc = crc16_modbus(bytes([length]) + body)
    frame = bytes([0xAA, 0x55, length]) + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
    return frame


def parse_frame(raw):
    if len(raw) < 5:
        return None
    if raw[0] != 0xAA or raw[1] != 0x55:
        return None
    length = raw[2]
    need = 3 + length + 2
    if len(raw) < need:
        return None
    body = raw[3:3 + length]
    crc_bytes = raw[3 + length:need]
    crc_calc = crc16_modbus(bytes([length]) + body)
    crc_exp = crc_bytes[0] | (crc_bytes[1] << 8)
    if crc_calc != crc_exp:
        return {'valid': False, 'reason': 'CRC mismatch'}
    return {
        'valid': True,
        'dev': body[0],
        'cmd': body[1],
        'payload': list(body[2:]),
    }


# ---------- 常用指令 ----------
def cmd_move_fwd(speed=200):        return make_frame(DEV_CAR, 0x01, [0x01, speed & 0xFF])
def cmd_move_back(speed=200):       return make_frame(DEV_CAR, 0x01, [0x02, speed & 0xFF])
def cmd_move_dist(cm, speed=200):   return make_frame(DEV_CAR, 0x02, [0x01, cm & 0xFF, (cm >> 8) & 0xFF, speed & 0xFF])
def cmd_turn_left(speed=200):       return make_frame(DEV_CAR, 0x03, [0x01, speed & 0xFF])
def cmd_turn_angle(deg, speed=150): return make_frame(DEV_CAR, 0x04, [0x01, deg & 0xFF, (deg >> 8) & 0xFF, speed & 0xFF])
def cmd_stop_all():                 return make_frame(DEV_CAR, 0x05, [0x00])
def cmd_stop_wheels():              return make_frame(DEV_CAR, 0x05, [0x01])
def cmd_car_query():                return make_frame(DEV_CAR, 0x06)
def cmd_arm_query():                return make_frame(DEV_ARM, 0x06)
def cmd_arm_lift(dir_, speed=100):  return make_frame(DEV_ARM, 0x01, [dir_, speed & 0xFF])
def cmd_arm_reach(dir_, speed=100): return make_frame(DEV_ARM, 0x03, [dir_, speed & 0xFF])
def cmd_arm_grip(act):              return make_frame(DEV_ARM, 0x05, [act & 0xFF])


def selfcheck():
    # 标准 CRC-16/MODBUS 校验向量
    assert crc16_modbus(b'123456789') == 0x4B37, 'CRC self-check failed'
    f = make_frame(DEV_CAR, 0x01, [0x01, 0xC8])
    p = parse_frame(f)
    assert p['valid'] and p['dev'] == DEV_CAR and p['cmd'] == 0x01, 'frame roundtrip failed'
    bad = bytearray(f); bad[-1] ^= 0xFF
    assert parse_frame(bytes(bad))['valid'] is False, 'corrupt frame should be rejected'
    print('selfcheck OK: CRC=0x%04X, frame roundtrip OK, corrupt-frame rejected.' % crc16_modbus(b'123456789'))


def stream_read(ser, timeout_s=1.0):
    """尝试解析串口流中的一条状态帧。"""
    buf = bytearray()
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        # 尝试任意起点匹配
        for i in range(len(buf)):
            sub = parse_frame(bytes(buf[i:]))
            if sub:
                return sub
        if len(buf) > 512:
            buf = buf[-128:]
    return None


def main():
    args = sys.argv[1:]
    if not args or args[0] == 'selfcheck':
        selfcheck()
        return

    port = None
    if args[0] == '--port':
        port = args[1]
        args = args[2:]
    baud = 115200
    if '--baud' in args:
        i = args.index('--baud')
        baud = int(args[i + 1])
        args = args[i + 2:]

    if not args:
        print('缺少命令，请指定要发送的指令名'); return

    name = args[0]
    build = {
        'car-move-fwd': lambda: cmd_move_fwd(),
        'car-move-back': lambda: cmd_move_back(),
        'car-move-dist': lambda: cmd_move_dist(int(args[1]) if len(args) > 1 else 30),
        'car-stop-all': lambda: cmd_stop_all(),
        'car-stop-wheels': lambda: cmd_stop_wheels(),
        'car-turn-left': lambda: cmd_turn_left(),
        'car-turn-angle': lambda: cmd_turn_angle(int(args[1]) if len(args) > 1 else 30),
        'car-query': lambda: cmd_car_query(),
        'arm-query': lambda: cmd_arm_query(),
        'arm-lift': lambda: cmd_arm_lift(int(args[1]) if len(args) > 1 else 1),
        'arm-reach': lambda: cmd_arm_reach(int(args[1]) if len(args) > 1 else 1),
        'arm-grip': lambda: cmd_arm_grip(int(args[1]) if len(args) > 1 else 1),
    }
    if name not in build:
        print('未知指令:', name); return

    if not port:
        print('需要 --port 指定串口（或使用 selfcheck）'); return

    frame = build[name]()
    print('TX:', frame.hex(' ').upper())
    import serial  # pip install pyserial
    ser = serial.Serial(port, baud, timeout=1)
    ser.write(frame)
    # 发送后尝试接收一条状态帧（若为查询/到位可能回发）
    resp = stream_read(ser, timeout_s=0.8)
    if resp:
        print('RX:', resp)
    ser.close()


if __name__ == '__main__':
    main()