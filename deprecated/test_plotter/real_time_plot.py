from collections import deque

import matplotlib.pyplot as plt
import serial

PORT = "/dev/ttyACM0"
BAUDRATE = 115200

WINDOW_US = 5_000_000


# ---------------------------------------------------------
# Serial
# ---------------------------------------------------------

ser = serial.Serial(PORT, BAUDRATE, timeout=0.01)


# ---------------------------------------------------------
# Data buffers
# ---------------------------------------------------------

timestamps = deque()
rhd_a = deque()
rhd_b = deque()


# ---------------------------------------------------------
# Plot
# ---------------------------------------------------------

plt.ion()

fig, (ax_a, ax_b) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

(line_a,) = ax_a.plot([], [], color="blue", linewidth=1)
(line_b,) = ax_b.plot([], [], color="orange", linewidth=1)

ax_a.set_ylabel("RHD_A")
ax_b.set_ylabel("RHD_B")
ax_b.set_xlabel("Time since power-on (seconds)")

ax_a.grid(True)
ax_b.grid(True)

plt.tight_layout()


# ---------------------------------------------------------
# Main loop
# ---------------------------------------------------------

try:
    while plt.fignum_exists(fig.number):
        # Read all available serial data
        while ser.in_waiting:
            raw = ser.readline()

            try:
                timestamp_us, command, register, a, b = map(
                    int, raw.decode("ascii").strip().split(",")
                )
            except (ValueError, UnicodeDecodeError):
                continue

            # Store the DEVICE timestamp directly.
            timestamps.append(timestamp_us)
            rhd_a.append(a)
            rhd_b.append(b)

        # Nothing received yet
        if not timestamps:
            plt.pause(0.01)
            continue

        # -------------------------------------------------
        # Current device time
        # -------------------------------------------------

        newest_us = timestamps[-1]

        # 5 seconds before current device time
        oldest_us = newest_us - WINDOW_US

        # -------------------------------------------------
        # Remove samples older than the window
        # -------------------------------------------------

        while timestamps and timestamps[0] < oldest_us:
            timestamps.popleft()
            rhd_a.popleft()
            rhd_b.popleft()

        # -------------------------------------------------
        # Convert DEVICE timestamp:
        #
        #       microseconds -> seconds
        #
        # Do NOT use time.time()
        # -------------------------------------------------

        x = [timestamp / 1_000_000.0 for timestamp in timestamps]

        # -------------------------------------------------
        # Update plots
        # -------------------------------------------------

        line_a.set_data(x, rhd_a)
        line_b.set_data(x, rhd_b)

        # -------------------------------------------------
        # EXACT 5 SECOND WINDOW
        # -------------------------------------------------

        xmin = oldest_us / 1_000_000.0
        xmax = newest_us / 1_000_000.0

        ax_a.set_xlim(xmin, xmax)

        # -------------------------------------------------
        # Y-axis scaling
        # -------------------------------------------------

        if rhd_a:
            amin = min(rhd_a)
            amax = max(rhd_a)

            if amin == amax:
                amin -= 1
                amax += 1

            margin = (amax - amin) * 0.05

            ax_a.set_ylim(amin - margin, amax + margin)

        if rhd_b:
            bmin = min(rhd_b)
            bmax = max(rhd_b)

            if bmin == bmax:
                bmin -= 1
                bmax += 1

            margin = (bmax - bmin) * 0.05

            ax_b.set_ylim(bmin - margin, bmax + margin)

        # -------------------------------------------------
        # Redraw
        # -------------------------------------------------

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        plt.pause(0.001)


except KeyboardInterrupt:
    print("Stopping...")


finally:
    ser.close()
    plt.close(fig)
