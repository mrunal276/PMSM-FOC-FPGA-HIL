import zipfile
import os
import numpy as np
import matplotlib.pyplot as plt

script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
ila_path = os.path.join(project_dir, 'foc_motor_ctrl.srcs', 'sources_1', 'ip', 'ila_0', 'iladata.ila')
figures_dir = os.path.join(project_dir, 'reports', 'figures')
os.makedirs(figures_dir, exist_ok=True)

with zipfile.ZipFile(ila_path, 'r') as z:
    with z.open('waveform.csv') as f:
        content = f.read().decode('utf-8', errors='ignore').splitlines()

headers = [c.strip() for c in content[0].split(',')]
rows = [line.split(',') for line in content[2:] if line.strip()]

data = {h: [] for h in headers}
for r in rows:
    for h, v in zip(headers, r):
        data[h].append(v.strip())

samples = [int(x) for x in data['Sample in Window']]
trigger = [int(x) for x in data['TRIGGER']]
fsm_state = [int(x, 16) for x in data['fsm_state[2:0]']]
theta = [int(x, 16) for x in data['theta_fdbk[15:0]']]
duty_a = [int(x, 16) / 32768.0 for x in data['duty_a[15:0]']]
duty_b = [int(x, 16) / 32768.0 for x in data['duty_b[15:0]']]
duty_c = [int(x, 16) / 32768.0 for x in data['duty_c[15:0]']]
carrier_sync = [int(x, 16) for x in data['carrier_sync']]
fault_active = [int(x, 16) for x in data['fault_active']]
open_loop_active = [int(x, 16) for x in data['open_loop_active']]
foc_enable = [int(x, 16) for x in data['foc_enable']]
gate_ah = [int(x, 16) for x in data['gate_ah_OBUF']]
gate_al = [int(x, 16) for x in data['gate_al_OBUF']]
gate_bh = [int(x, 16) for x in data['gate_bh_OBUF']]
gate_bl = [int(x, 16) for x in data['gate_bl_OBUF']]

# Convert theta (16-bit unsigned angle) to radians [0, 2*pi]
theta_rad = [t * (2.0 * np.pi / 65536.0) for t in theta]

# Plot 1: Full ILA Capture Overview
plt.figure(figsize=(12, 10))

plt.subplot(4, 1, 1)
plt.plot(samples, theta_rad, color='#1f77b4', lw=1.5, label=r'$\theta_e$ (Electrical Angle [rad])')
plt.axvline(512, color='red', linestyle='--', alpha=0.7, label='Trigger Event (Sample 512)')
plt.ylabel('Angle (rad)')
plt.title('Hardware ILA Capture on Xilinx Zybo Z7-10 (xc7z010-1) - Full 1024 Sample Buffer')
plt.grid(True, alpha=0.3)
plt.legend(loc='upper right')

plt.subplot(4, 1, 2)
plt.plot(samples, duty_a, label='Phase A Duty', color='#d62728', lw=1.2)
plt.plot(samples, duty_b, label='Phase B Duty', color='#2ca02c', lw=1.2)
plt.plot(samples, duty_c, label='Phase C Duty', color='#ff7f0e', lw=1.2)
plt.axvline(512, color='red', linestyle='--', alpha=0.7)
plt.ylabel('Normalized Duty')
plt.grid(True, alpha=0.3)
plt.legend(loc='upper right')

plt.subplot(4, 1, 3)
plt.step(samples, fsm_state, where='post', color='#9467bd', lw=1.8, label='FSM State (2: RAMP, 3: CLOSED_LOOP, 4: FAULT, 0: IDLE)')
plt.axvline(512, color='red', linestyle='--', alpha=0.7)
plt.ylabel('FSM State')
plt.yticks([0, 2, 3, 4], ['0: IDLE', '2: RAMP', '3: RUN', '4: FAULT'])
plt.grid(True, alpha=0.3)
plt.legend(loc='upper right')

plt.subplot(4, 1, 4)
plt.step(samples, [x + 3.3 for x in gate_ah], where='post', label='Gate AH', color='#1f77b4')
plt.step(samples, [x + 2.2 for x in gate_al], where='post', label='Gate AL', color='#aec7e8')
plt.step(samples, [x + 1.1 for x in fault_active], where='post', label='Fault Active', color='#d62728')
plt.step(samples, open_loop_active, where='post', label='Open-Loop Active', color='#ffbb78')
plt.axvline(512, color='red', linestyle='--', alpha=0.7)
plt.xlabel('Sample Index (Carrier Synchronization Clock ticks @ 25 kHz)')
plt.ylabel('Digital Logic')
plt.yticks([0.5, 1.6, 2.7, 3.8], ['Open-Loop', 'Fault Active', 'Gate AL', 'Gate AH'])
plt.grid(True, alpha=0.3)
plt.legend(loc='upper right')

plt.tight_layout()
out1 = os.path.join(figures_dir, 'ila_hardware_capture.png')
plt.savefig(out1, dpi=300)
plt.close()
print(f'Saved {out1}')

# Plot 2: Zoomed-in Transition (Samples 480 to 560)
zoom_slice = slice(480, 560)
z_samples = samples[zoom_slice]

plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(z_samples, [duty_a[i] for i in range(480, 560)], label='Duty A', color='#d62728', marker='.')
plt.plot(z_samples, [duty_b[i] for i in range(480, 560)], label='Duty B', color='#2ca02c', marker='.')
plt.plot(z_samples, [duty_c[i] for i in range(480, 560)], label='Duty C', color='#ff7f0e', marker='.')
plt.axvline(511, color='blue', linestyle=':', label='Closed-Loop Handover (State 3)')
plt.axvline(512, color='red', linestyle='--', label='Fault Trip Trigger (State 4)')
plt.axvline(534, color='black', linestyle='-.', label='Shutdown Latch (All Gates 0)')
plt.ylabel('Duty Cycle [0, 1.0]')
plt.title('Zoomed Hardware Transition: RAMP (2) -> CLOSED-LOOP (3) -> FAULT (4) -> SAFE SHUTDOWN')
plt.grid(True, alpha=0.3)
plt.legend(loc='upper right')

plt.subplot(3, 1, 2)
plt.step(z_samples, [fsm_state[i] for i in range(480, 560)], where='post', color='#9467bd', lw=2)
plt.ylabel('FSM State')
plt.yticks([0, 2, 3, 4], ['IDLE (0)', 'RAMP (2)', 'RUN (3)', 'FAULT (4)'])
plt.grid(True, alpha=0.3)

plt.subplot(3, 1, 3)
plt.step(z_samples, [gate_ah[i] for i in range(480, 560)], where='post', label='Gate AH', color='green', lw=1.5)
plt.step(z_samples, [fault_active[i] for i in range(480, 560)], where='post', label='Fault Flag', color='red', lw=1.5)
plt.xlabel('Sample Index (Carrier Periods @ 25 kHz, 40 us per tick)')
plt.ylabel('Status')
plt.grid(True, alpha=0.3)
plt.legend(loc='upper right')

plt.tight_layout()
out2 = os.path.join(figures_dir, 'ila_transition_zoom.png')
plt.savefig(out2, dpi=300)
plt.close()
print(f'Saved {out2}')
