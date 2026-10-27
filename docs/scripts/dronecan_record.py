#!/usr/bin/env python3
'''
DroneCAN 录制脚本
- 连接 SLCAN 串口（如 /dev/ttyACM0），打印在线节点
- 将所有收到的 Transfer 解码后写入 JSONL 文件（一行一条）
- Ctrl-C 安全退出
用法:
    python3 dronecan_record.py [--port /dev/ttyACM0] [--bitrate 1000000]
        [--node-id 100] [--out 前缀] [--filter-nodeid 10 11]
输出: <前缀>_<时间戳>.jsonl 与同名 .nodes.txt（在线节点列表）
'''
import sys
import time
import json
import argparse
import signal

import dronecan
from dronecan import uavcan


def msg_type_name(transfer):
    # 稳定取消息类型名（依赖 DSDL 生成的类）
    try:
        return uavcan.get_descriptor(transfer.payload.dtype.__class__
                                     if hasattr(transfer.payload, 'dtype')
                                     else type(transfer.payload)).full_name
    except Exception:
        try:
            return str(transfer.message_type)
        except Exception:
            return 'unknown'


def main():
    parser = argparse.ArgumentParser(description='DroneCAN recorder (SLCAN serial)')
    parser.add_argument('--port', default='/dev/ttyACM0', help='SLCAN 串口设备')
    parser.add_argument('--bitrate', default=1000000, type=int)
    parser.add_argument('--node-id', default=100, type=int, help='本节点 ID（旁听用，随意）')
    parser.add_argument('--out', default='can_record', help='输出文件前缀')
    parser.add_argument('--filter-nodeid', nargs='+', type=int,
                        help='只记录这些源节点 ID（默认全部）')
    args = parser.parse_args()

    fname = '%s_%s.jsonl' % (args.out, time.strftime('%Y%m%d_%H%M%S'))
    nodes_fname = fname.replace('.jsonl', '.nodes.txt')

    print('Connecting to %s @ %d ...' % (args.port, args.bitrate))
    try:
        node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)
    except Exception as ex:
        print('无法连接 %s: %s' % (args.port, ex))
        print('提示: 若 dronecan_gui_tool / slcand 正占用串口，请先断开。')
        sys.exit(1)

    nodes = {}   # node_id -> 首次心跳时间/最后一次看到的 UTC
    fout = open(fname, 'w')
    count = [0]

    def on_transfer(t):
        # add_transfer_hook 签名: hook(transfer)
        src = t.source_node_id
        if args.filter_nodeid and src not in args.filter_nodeid:
            return
        if src is not None:
            nodes[src] = time.time()
        payload = bytes(t.payload)
        rec = {
            'ts': getattr(t, 'ts_real', None),           # UTC 秒
            'ts_monotonic': getattr(t, 'ts_monotonic', None),
            'src_node': src,
            'prio': t.priority,
            'transfer_id': t.transfer_id,
            'type': msg_type_name(t),
            'type_id': t.message_type if isinstance(t.message_type, int) else None,
            'data_len': len(payload),
            'data_hex': payload.hex(),
        }
        # 尝试解码为 DSDL 对象并序列化可读字段
        try:
            obj = t.payload
            rec['decoded'] = {f: str(getattr(obj, f)) for f in
                              obj.dtype.fields if hasattr(obj, f)}
        except Exception:
            pass
        fout.write(json.dumps(rec) + '\n')
        count[0] += 1
        if count[0] % 100 == 0:
            fout.flush()

    node.add_transfer_hook(on_transfer)

    running = [True]

    def stop(sig, frame):
        running[0] = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    print('录制中 -> %s （Ctrl-C 结束）' % fname)
    last_report = time.time()
    try:
        while running[0]:
            node.spin(0.1)
            now = time.time()
            if now - last_report >= 5:
                print('已记录 %d 条 | 在线节点: %s'
                      % (count[0],
                         ','.join(str(n) for n in sorted(nodes)) or '无'))
                last_report = now
    finally:
        with open(nodes_fname, 'w') as nf:
            for n in sorted(nodes):
                nf.write('%d\n' % n)
        fout.close()
        print('\n结束: 共 %d 条 -> %s' % (count[0], fname))
        print('在线节点列表 -> %s: %s'
              % (nodes_fname, ','.join(str(n) for n in sorted(nodes)) or '无'))


if __name__ == '__main__':
    main()
