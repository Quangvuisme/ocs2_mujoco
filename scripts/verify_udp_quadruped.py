#!/usr/bin/env python3
"""
UDP Test Script for MuJoCo Bridge Communication

This script acts as a SIMPLE CONTROLLER to test UDP communication
with the MuJoCo bridge (quadruped_bridge running in MuJoCo).

Usage:
    # Make sure MuJoCo with quadruped_bridge is running first!
    python3 verify_udp.py --mode passive
    python3 verify_udp.py --mode pd

It will:
1. Receive state packets FROM MuJoCo bridge (port 9101)
2. Compute torques (passive=0, pd=simple PD control)
3. Send command packets TO MuJoCo bridge (port 9102)
4. Display joint positions and verify communication

This helps verify:
- UDP communication between controller and MuJoCo bridge
- Joint order mapping (MuJoCo order)
- PASSIVE mode: sends zero torques
- PD_CONTROL mode: sends PD torques to reach default stance
"""

import socket
import struct
import time
import numpy as np
import argparse
import threading

# UDP Ports (same as quadruped_bridge)
STATE_PORT = 9101   # Bridge SENDS state TO this port (we receive here)
CMD_PORT = 9102     # Bridge RECEIVES commands ON this port (we send here)

# Packet formats
# State packet: uint64_t timestamp + double[36]
STATE_FORMAT = '<Q36d'  # little-endian: uint64 + 36 doubles
STATE_SIZE = struct.calcsize(STATE_FORMAT)  # 8 + 36*8 = 296 bytes

# Command packet: uint64_t timestamp + double[12]
CMD_FORMAT = '<Q12d'  # little-endian: uint64 + 12 doubles
CMD_SIZE = struct.calcsize(CMD_FORMAT)  # 8 + 12*8 = 104 bytes

# Joint names for display
MUJOCO_JOINT_NAMES = [
    'LF_HAA', 'LF_HFE', 'LF_KFE',  # 0-2
    'RF_HAA', 'RF_HFE', 'RF_KFE',  # 3-5
    'LH_HAA', 'LH_HFE', 'LH_KFE',  # 6-8
    'RH_HAA', 'RH_HFE', 'RH_KFE',  # 9-11
]

# Default standing pose (MuJoCo order: LF, RF, LH, RH) anymal_c
# DEFAULT_JOINTS = np.array([
#     -0.25,  0.60, -0.85,   # LF
#      0.25,  0.60, -0.85,   # RF
#     -0.25, -0.60,  0.85,   # LH
#      0.25, -0.60,  0.85    # RH
# ])

# Default standing pose (MuJoCo order: LF, RF, LH, RH) go2
DEFAULT_JOINTS = np.array([
    0.01,  0.70, -1.37,   # LF
     -0.01,  0.70, -1.37,   # RF
    0.01, 0.70,  -1.37,   # LH
     -0.01, 0.70,  -1.37    # RH
])


class SimpleController:
    """Simple UDP controller to test communication with MuJoCo bridge"""
    def __init__(self, mode='passive'):
        self.mode = mode  # 'passive' or 'pd'
        self.running = False
        
        # Received state
        self.last_state_time = 0.0
        self.base_pos = np.zeros(3)
        self.base_euler = np.zeros(3)
        self.joint_pos = np.zeros(12)
        self.joint_vel = np.zeros(12)
        self.state_count = 0
        
        # PD gains
        self.KP = 150.0
        self.KD = 5.0
        self.MAX_TORQUE = 80.0
        
        # Sockets
        self.state_sock = None
        self.cmd_sock = None
        self.cmd_count = 0
        
    def parse_state_packet(self, data):
        """Parse state packet from MuJoCo bridge"""
        if len(data) != STATE_SIZE:
            return None
        
        unpacked = struct.unpack(STATE_FORMAT, data)
        timestamp_ns = unpacked[0]
        state = np.array(unpacked[1:])
        
        # Extract components
        result = {
            'timestamp': timestamp_ns / 1e9,
            'euler': state[0:3],      # roll, pitch, yaw
            'pos': state[3:6],        # x, y, z
            'joint_pos': state[6:18], # MuJoCo order
            'ang_vel': state[18:21],
            'lin_vel': state[21:24],
            'joint_vel': state[24:36]
        }
        return result
    
    def create_command_packet(self, torques, timestamp):
        """Create command packet to send to MuJoCo bridge"""
        timestamp_ns = int(timestamp * 1e9)
        return struct.pack(CMD_FORMAT, timestamp_ns, *torques)
    
    def compute_torques(self, state):
        """Compute torques based on mode"""
        if self.mode == 'passive':
            return np.zeros(12)
        
        elif self.mode == 'pd':
            # PD control to default standing pose
            torques = np.zeros(12)
            for i in range(12):
                error = DEFAULT_JOINTS[i] - state['joint_pos'][i]
                derror = -state['joint_vel'][i]
                torques[i] = self.KP * error + self.KD * derror
                torques[i] = np.clip(torques[i], -self.MAX_TORQUE, self.MAX_TORQUE)
            return torques
        
        return np.zeros(12)
    
    def start(self):
        """Initialize sockets"""
        # Socket to receive state (UDP server)
        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.state_sock.bind(('0.0.0.0', STATE_PORT))
        self.state_sock.setblocking(False)
        
        # Socket to send commands (UDP client)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.running = True
        print(f"[Controller] Started in {self.mode.upper()} mode")
        print(f"  State RX on port {STATE_PORT}")
        print(f"  Command TX to 127.0.0.1:{CMD_PORT}")
        
    def stop(self):
        """Cleanup"""
        self.running = False
        if self.state_sock:
            self.state_sock.close()
        if self.cmd_sock:
            self.cmd_sock.close()
        print(f"[Controller] Stopped (RX:{self.state_count}, TX:{self.cmd_count})")
    
    def step(self):
        """One control step: receive state -> compute torque -> send command"""
        # Try to receive state
        try:
            data, addr = self.state_sock.recvfrom(512)
            state = self.parse_state_packet(data)
            
            if state:
                self.state_count += 1
                self.last_state_time = state['timestamp']
                self.base_pos = state['pos']
                self.base_euler = state['euler']
                self.joint_pos = state['joint_pos']
                self.joint_vel = state['joint_vel']
                
                # Compute torques
                torques = self.compute_torques(state)
                
                # Send command
                cmd_packet = self.create_command_packet(torques, self.last_state_time)
                self.cmd_sock.sendto(cmd_packet, ('127.0.0.1', CMD_PORT))
                self.cmd_count += 1
                
                return True, state, torques
        except BlockingIOError:
            pass
        
        return False, None, None


def run_test(mode, duration=10.0):
    """Run controller test with MuJoCo bridge"""
    print("\n" + "="*70)
    if mode == 'passive':
        print("TEST: PASSIVE MODE")
        print("Sends zero torques to MuJoCo. Robot should fall/rest.")
    else:
        print("TEST: PD CONTROL MODE")
        print("Sends PD torques to reach default standing pose.")
    print("="*70)
    
    controller = SimpleController(mode=mode)
    
    try:
        controller.start()
        print("\nWaiting for state from MuJoCo bridge...")
        
        start_time = time.time()
        last_print_time = 0
        
        while time.time() - start_time < duration:
            success, state, torques = controller.step()
            
            current_time = time.time() - start_time
            
            # Print every 0.5s
            if success and current_time - last_print_time >= 0.5:
                last_print_time = current_time
                
                print(f"\n[t={state['timestamp']:.2f}s] RX:{controller.state_count}, TX:{controller.cmd_count}")
                print(f"  Base: pos=[{state['pos'][0]:.3f}, {state['pos'][1]:.3f}, {state['pos'][2]:.3f}] "
                      f"euler=[{np.degrees(state['euler'][0]):.1f}°, {np.degrees(state['euler'][1]):.1f}°, {np.degrees(state['euler'][2]):.1f}°]")
                
                # Show joint positions with errors from default
                print(f"  Joints (MuJoCo order):")
                for i in range(4):
                    leg = ['LF', 'RF', 'LH', 'RH'][i]
                    j = i * 3
                    joint_slice = state['joint_pos'][j:j+3]
                    default_slice = DEFAULT_JOINTS[j:j+3]
                    errors = joint_slice - default_slice
                    print(f"    {leg}: [{joint_slice[0]:+.3f}, {joint_slice[1]:+.3f}, {joint_slice[2]:+.3f}] "
                          f"err=[{errors[0]:+.3f}, {errors[1]:+.3f}, {errors[2]:+.3f}]")
                
                # Show torques
                print(f"  Torques:")
                for i in range(4):
                    leg = ['LF', 'RF', 'LH', 'RH'][i]
                    j = i * 3
                    torque_slice = torques[j:j+3]
                    print(f"    {leg}: [{torque_slice[0]:+.2f}, {torque_slice[1]:+.2f}, {torque_slice[2]:+.2f}] Nm")
                
                # Check status
                if mode == 'passive':
                    max_torque = np.max(np.abs(torques))
                    status = "✓" if max_torque < 0.1 else "✗"
                    print(f"  {status} Max torque: {max_torque:.2f} Nm (should be ~0)")
                else:
                    max_error = np.max(np.abs(state['joint_pos'] - DEFAULT_JOINTS))
                    status = "✓" if max_error < 0.05 else "○"
                    print(f"  {status} Max joint error: {max_error:.3f} rad (target < 0.05)")
            
            time.sleep(0.001)  # 1ms loop
            
        print(f"\n[TEST] Completed after {duration:.1f}s")
        print(f"  Total states received: {controller.state_count}")
        print(f"  Total commands sent: {controller.cmd_count}")
        
    except KeyboardInterrupt:
        print("\n[Interrupted]")
    finally:
        controller.stop()


def run_self_test():
    """Self-test mode - verify packet format without MuJoCo"""
    print("\n" + "="*70)
    print("SELF-TEST MODE (no MuJoCo needed)")
    print("="*70)
    
    # Test 1: Packet sizes
    print("\n[1] Packet Size Verification:")
    print(f"  State packet:   {STATE_SIZE} bytes (expected: 296)")
    print(f"  Command packet: {CMD_SIZE} bytes (expected: 104)")
    
    state_ok = STATE_SIZE == 296
    cmd_ok = CMD_SIZE == 104
    print(f"  {'✓' if state_ok else '✗'} State packet size correct")
    print(f"  {'✓' if cmd_ok else '✗'} Command packet size correct")
    
    # Test 2: Create and parse state packet
    print("\n[2] State Packet Encoding/Decoding:")
    timestamp_ns = int(1.234 * 1e9)
    test_euler = np.array([0.01, 0.02, 0.03])  # roll, pitch, yaw
    test_pos = np.array([0.1, 0.2, 0.54])
    test_joints = DEFAULT_JOINTS.copy()
    
    # Create state array
    state = np.zeros(36)
    state[0:3] = test_euler
    state[3:6] = test_pos
    state[6:18] = test_joints
    
    packet = struct.pack(STATE_FORMAT, timestamp_ns, *state)
    # Parse it back
    unpacked = struct.unpack(STATE_FORMAT, packet)
    timestamp_ns = unpacked[0]
    parsed_state = np.array(unpacked[1:])
    
    print(f"  Created packet: {len(packet)} bytes")
    print(f"  Timestamp: {timestamp_ns} ns ({timestamp_ns/1e9:.3f} s)")
    print(f"  Euler (roll,pitch,yaw): [{parsed_state[0]:.3f}, {parsed_state[1]:.3f}, {parsed_state[2]:.3f}]")
    print(f"  Position (x,y,z): [{parsed_state[3]:.3f}, {parsed_state[4]:.3f}, {parsed_state[5]:.3f}]")
    print(f"  Joint positions (MuJoCo order):")
    for i in range(4):
        leg = ['LF', 'RF', 'LH', 'RH'][i]
        j = i * 3
        print(f"    {leg}: [{parsed_state[6+j]:.3f}, {parsed_state[6+j+1]:.3f}, {parsed_state[6+j+2]:.3f}]")
    
    # Verify encoding
    euler_ok = np.allclose(parsed_state[0:3], test_euler)
    pos_ok = np.allclose(parsed_state[3:6], test_pos)
    joints_ok = np.allclose(parsed_state[6:18], test_joints)
    
    print(f"\n  {'✓' if euler_ok else '✗'} Euler angles encoded correctly")
    print(f"  {'✓' if pos_ok else '✗'} Position encoded correctly")
    print(f"  {'✓' if joints_ok else '✗'} Joint positions encoded correctly")
    
    # Test 3: Command packet
    print("\n[3] Command Packet Encoding/Decoding:")
    test_torques = np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0])
    timestamp_ns = int(2.0 * 1e9)
    cmd_packet = struct.pack(CMD_FORMAT, timestamp_ns, *test_torques)
    print(f"  Created command packet: {len(cmd_packet)} bytes")
    
    # Parse it
    result = struct.unpack(CMD_FORMAT, cmd_packet)
    parsed_ts = result[0]
    parsed_torques = np.array(result[1:])
    
    print(f"  Timestamp: {parsed_ts} ns ({parsed_ts/1e9:.3f} s)")
    print(f"  Torques: [{', '.join([f'{t:.1f}' for t in parsed_torques])}]")
    
    torques_ok = np.allclose(parsed_torques, test_torques)
    print(f"  {'✓' if torques_ok else '✗'} Torques encoded correctly")
    
    # Test 4: Joint order mapping display
    print("\n[4] Joint Order Reference:")
    print("  MuJoCo/URDF order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)")
    print("  OCS2 order:        LF(0-2), LH(3-5), RF(6-8), RH(9-11)")
    print("\n  MuJoCo → OCS2 mapping:")
    print("    MuJoCo[0-2] (LF) → OCS2[0-2] (LF)")
    print("    MuJoCo[3-5] (RF) → OCS2[6-8] (RF)")
    print("    MuJoCo[6-8] (LH) → OCS2[3-5] (LH)")
    print("    MuJoCo[9-11](RH) → OCS2[9-11](RH)")
    
    print("\n  Default standing pose (MuJoCo order):")
    for i, (name, val) in enumerate(zip(MUJOCO_JOINT_NAMES, DEFAULT_JOINTS)):
        print(f"    [{i:2d}] {name}: {val:+.2f}")
    
    # Summary
    print("\n" + "="*70)
    all_ok = state_ok and cmd_ok and euler_ok and pos_ok and joints_ok and torques_ok
    if all_ok:
        print("✓ ALL TESTS PASSED")
    else:
        print("✗ SOME TESTS FAILED")
    print("="*70)


def main():
    parser = argparse.ArgumentParser(description='Test UDP communication with MuJoCo bridge')
    parser.add_argument('--mode', choices=['passive', 'pd', 'selftest'], default='selftest',
                       help='Test mode: passive (zero torques), pd (PD control), or selftest (no MuJoCo)')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Test duration in seconds (for passive/pd modes)')
    args = parser.parse_args()
    
    print("="*70)
    print("  UDP Communication Test for MuJoCo Bridge")
    print("="*70)
    
    if args.mode == 'selftest':
        run_self_test()
        return
    
    print(f"\n*** IMPORTANT: Make sure MuJoCo with quadruped_bridge is running! ***")
    print(f"\nMode: {args.mode.upper()}")
    print(f"Duration: {args.duration}s")
    print("\nStarting in 2 seconds...")
    time.sleep(2)
    
    run_test(args.mode, args.duration)
    
    print("\n" + "="*70)
    print("  Test Complete")
    print("="*70)


if __name__ == '__main__':
    main()
