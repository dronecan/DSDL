#!/usr/bin/env python3
"""DroneCAN MotorPosition v2: DSDL and multi-frame transport round-trip.

Run from the DSDL repository root with `dronecan` installed:
    python3 test_motor_position.py
    python3 test_motor_position_continuity.py
"""
import os
import dronecan

ROOTS = [n for n in ('uavcan', 'com', 'ardupilot', 'dronecan') if os.path.isdir(n)]
assert 'uavcan' in ROOTS, 'Run from the DSDL repository root'
dronecan.load_dsdl(*ROOTS, exclude_dist=True)

Msg = dronecan.TYPENAMES['uavcan.equipment.esc.MotorPosition']
assert Msg.default_dtid == 1037
assert Msg.get_min_bitlen() == 79
assert Msg.get_max_bitlen() == 95


def roundtrip(message, lengths):
    transfer = dronecan.transport.Transfer(
        payload=message, source_node_id=42, transfer_id=3)
    frames = transfer.to_frames()
    actual = [len(f.bytes) for f in frames]
    assert actual == lengths, f'frame lengths {actual}, wanted {lengths}'
    assert len(frames) == 2
    assert all(len(f.bytes) <= 8 for f in frames)
    assert frames[0].start_of_transfer and not frames[0].end_of_transfer
    assert not frames[-1].start_of_transfer and frames[-1].end_of_transfer
    assert all(((f.message_id >> 8) & 0xFFFF) == 1037 for f in frames)
    incoming = dronecan.transport.Transfer()
    incoming.from_frames(frames)
    assert incoming.data_type_signature == Msg.get_data_type_signature()
    return incoming.payload

checks = 0
for count in (0, 1, -1, 123456, -123456, 2**31 - 1, -(2**31)):
    msg = Msg()
    msg.esc_index = 3
    msg.valid = True
    msg.origin_id = 0xA1B2C3D4
    msg.sample_seq = 254
    msg.position.ticks.ticks = count
    msg.position.ticks.ticks_per_revolution = 42
    rx = roundtrip(msg, [8, 8])
    assert rx.esc_index == 3 and rx.valid is True
    assert rx.origin_id == 0xA1B2C3D4 and rx.sample_seq == 254
    assert dronecan.get_active_union_field(rx.position) == 'ticks'
    assert rx.position.ticks.ticks == count
    assert rx.position.ticks.ticks_per_revolution == 42
    checks += 1

for rev in (0.0, 1.125, -12.5, 125000.0):
    msg = Msg()
    msg.esc_index = 31
    msg.valid = True
    msg.origin_id = 0xFFFFFFFF
    msg.sample_seq = 255
    msg.position.revolutions = rev
    rx = roundtrip(msg, [8, 6])
    assert rx.esc_index == 31 and rx.valid is True
    assert rx.origin_id == 0xFFFFFFFF and rx.sample_seq == 255
    assert dronecan.get_active_union_field(rx.position) == 'revolutions'
    assert rx.position.revolutions == rev
    checks += 1

msg = Msg()
msg.esc_index = 0
msg.valid = False
msg.origin_id = 500
msg.sample_seq = 0
msg.position.ticks.ticks = -42
msg.position.ticks.ticks_per_revolution = 84
rx = roundtrip(msg, [8, 8])
assert rx.valid is False and rx.origin_id == 500 and rx.sample_seq == 0
checks += 1

print(f'OK: {checks} DSDL multi-frame serialization checks')
print('DTID:', Msg.default_dtid)
print(f'NEW signature: 0x{Msg.get_data_type_signature():016X}')
print('Tick: 12-byte payload / 2 CAN frames (8 + 8 bytes incl. tails)')
print('Float: 10-byte payload / 2 CAN frames (8 + 6 bytes incl. tails)')
