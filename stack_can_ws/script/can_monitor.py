#!/usr/bin/env python3
import argparse
import can
import time

PICK_DESC = {0: '不抓取', 1: '抓取', 2: '抓取失败', 3: '无效'}
UNLOAD_DESC = {0: '不卸货', 1: '卸货', 2: '卸货无效', 3: '无效'}

def parse_args():
    ap = argparse.ArgumentParser(description="Monitor 29-bit CAN status frame and decode fields")
    ap.add_argument('--if', dest='iface', default='can0', help='SocketCAN interface (e.g., can0, vcan0)')
    ap.add_argument('--id', dest='canid', default='0x18FED188', help='Extended CAN ID (decimal or 0x...)')
    ap.add_argument('--no-raw', action='store_true', help='Do not print raw bytes')
    ap.add_argument('--abs-ts', action='store_true', help='Print absolute timestamp instead of relative')
    return ap.parse_args()

def parse_frame(data: bytes):
    # Pad to 8 bytes if needed
    b = list(data[:8]) + [0] * max(0, 8 - len(data))

    # 字段解码（小端/Intel）
    # 1) 距离: byte0..1, uint16 LE, 0.2 m/LSB
    dist_raw = b[0] | (b[1] << 8)
    dist_m = dist_raw * 0.2

    # 2) 方向角(转向指令；右正/左负): byte2..3, uint16 LE, raw = steer_deg + 180
    ang_raw = b[2] | (b[3] << 8)
    steer_deg = ang_raw - 180
    # 规范为 [-180, +179]
    if steer_deg < -180:
        steer_deg += 360
    elif steer_deg >= 180:
        steer_deg -= 360

    # 3) 档速度: byte4, uint8, 0.5 km/h/LSB, offset -10
    sp_raw = b[4]
    speed_kmh = sp_raw * 0.5 - 50.0

    # 4) 动作/请求位: byte5
    pick = b[5] & 0x03                  # bit0..1
    unload = (b[5] >> 2) & 0x03         # bit2..3
    remote = (b[5] >> 4) & 0x01         # bit4
    dump = (b[5] >> 5) & 0x01           # bit5

    # 5) 状态/动作指令: byte6
    estop = b[6] & 0x01                 # bit0
    drive = (b[6] >> 1) & 0x01          # bit1
    action = (b[6] >> 2) & 0x0F         # bit2..5（当前未使用，预留）

    return {
        'dist_m': dist_m,
        'steer_deg': steer_deg,
        'speed_kmh': speed_kmh,
        'pick': pick,
        'unload': unload,
        'remote': remote,
        'dump': dump,
        'estop': estop,
        'drive': drive,
        'action': action,
        'raw': b
    }

def fmt_raw(b):
    return ' '.join(f'{x:02X}' for x in b)

def main():
    args = parse_args()
    try:
        canid = int(args.canid, 0)
    except Exception:
        print(f'Bad --id: {args.canid}')
        return

    bus = can.Bus(channel=args.iface, bustype='socketcan', fd=False)
    bus.set_filters([{"can_id": canid, "can_mask": 0x1FFFFFFF, "extended": True}])

    t0 = time.time()
    print(f'Listening {args.iface}, extid=0x{canid:X}')
    print('Fields: dist[m], steer[deg](右正/左负), speed[km/h], '
          'estop, drive, remote, dump, pick, unload, action, raw')

    try:
        for msg in bus:
            if msg.arbitration_id != canid:
                continue
            d = parse_frame(msg.data)
            ts = msg.timestamp if args.abs_ts else (msg.timestamp - t0)
            # 组装关键信息
            head = f'[{ts:9.3f}] 0x{msg.arbitration_id:X}'
            key = (f'dist={d["dist_m"]:>5.1f}m  steer={d["steer_deg"]:>+4.0f}°  '
                   f'speed={d["speed_kmh"]:>+5.1f}km/h  '
                   f'estop={d["estop"]} drive={d["drive"]} remote={d["remote"]} dump={d["dump"]}  '
                   f'pick={d["pick"]}({PICK_DESC.get(d["pick"], "-")}) '
                   f'unload={d["unload"]}({UNLOAD_DESC.get(d["unload"], "-")})  '
                   f'action={d["action"]}')
            raw = f'  raw: {fmt_raw(d["raw"])}' if not args.no_raw else ''
            print(f'{head} | {key}{raw}')
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
