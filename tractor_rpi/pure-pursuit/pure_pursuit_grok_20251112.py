import math
import socket
import json
import time
import argparse
import signal
import sys

class PurePursuit:
    def __init__(self, wheelbase=1.27, max_steer=0.623, pos_tol=0.1, target_ip='127.0.0.1', target_port=6004, rate_hz=20.0):
        """
        Initialize the Pure Pursuit controller with UDP sender integration.
        
        :param wheelbase: Vehicle wheelbase in meters (default: 1.27m).
        :param max_steer: Maximum steering angle in radians (default: 0.623 rad ~35.7° from turning data).
                          Outside wheel circle diameter when driving hard rght = 4.27 m / 2 = 2.135 m. (radius)
                          Subtract rear half-track width (29 in / 2 = 0.3685 m): R_center ≈ 2.135 - 0.3685 = 1.7665 m
                          From the Ackermann model, δ_max = atan(L / R_center) ≈ atan(1.27 / 1.7665) ≈ 0.623 rad (~35.7° or ~39% of 90° full lock).
        :param pos_tol: Position tolerance for goal reaching in meters.
        :param target_ip: IP for UDP sending (default: localhost).
        :param target_port: UDP port (default: 6004).
        :param rate_hz: Send rate in Hz (default: 20.0).
        """
        self.L = wheelbase
        self.delta_max = max_steer
        self.pos_tol = pos_tol
        self.gps_offset_x = 0.3048  # 12 inches forward in meters.
        self.gps_offset_y = -0.1524  # 6 inches to the right in meters (y positive left, so negative for right offset? Wait, user said "to the left", assuming facing forward, left is +y).
        # Note: User said "6" to the left (when looking from front), so if forward is +x, left is +y, offset_y = +0.1524.
        self.gps_offset_y = 0.1524
        self.path = []  # List of (x, y, yaw_rad, ld, v) in local frame.
        self.idx = 0
        self.goal_reached = True
        self.ref_lat = 0.0
        self.ref_lon = 0.0

        # UDP Sender integration
        self.target_ip = target_ip
        self.target_port = target_port
        self.rate_hz = rate_hz
        self.period = 1.0 / rate_hz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.messages_sent = 0
        self.start_time = None
        self.last_stats_time = None
        self.last_stats_count = 0
        self.running = True

        # Last known pose (for sending at rate even if no new GPS)
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_h = 0.0

    def load_path(self, filename):
        """
        Load mission path from TXT file.
        Format per line: lat lon yaw_rad lookahead_m speed_mps
        """
        self.path = []
        try:
            with open(filename, 'r') as f:
                lines = f.readlines()
        except FileNotFoundError:
            print(f"Error: Mission file '{filename}' not found.")
            return

        first = True
        for line_num, line in enumerate(lines, 1):
            parts = line.strip().split()
            if len(parts) != 5:
                print(f"Warning: Skipping invalid line {line_num}: {line.strip()}")
                continue
            try:
                lat, lon, yaw, ld, v = map(float, parts)
                if first:
                    self.ref_lat = lat
                    self.ref_lon = lon
                    first = False
                x, y = self.latlon_to_xy(lat, lon)
                self.path.append((x, y, yaw, ld, v))
            except ValueError:
                print(f"Warning: Skipping invalid numeric line {line_num}: {line.strip()}")
                continue

        self.idx = 0
        self.goal_reached = False if self.path else True
        print(f"Loaded {len(self.path)} waypoints from {filename}.")

    def latlon_to_xy(self, lat, lon):
        """Convert lat/lon to local x/y meters (equirectangular projection)."""
        lat0_rad = math.radians(self.ref_lat)
        dlat_rad = math.radians(lat - self.ref_lat)
        dlon_rad = math.radians(lon - self.ref_lon)
        x = 111320.0 * dlon_rad * math.cos(lat0_rad)
        y = 110540.0 * dlat_rad
        return x, y

    def gps_to_base(self, gps_lat, gps_lon, heading_rad):
        """Transform GPS position to base_link (rear axle center)."""
        gps_x, gps_y = self.latlon_to_xy(gps_lat, gps_lon)
        c = math.cos(heading_rad)
        s = math.sin(heading_rad)
        # Offset transformation: rotate offset vector by heading, then subtract from GPS
        delta_x = c * self.gps_offset_x - s * self.gps_offset_y
        delta_y = s * self.gps_offset_x + c * self.gps_offset_y
        base_x = gps_x - delta_x
        base_y = gps_y - delta_y
        return base_x, base_y, heading_rad

    def compute_steering(self, curr_x, curr_y, curr_h):
        """Compute steering angle and speed using Pure Pursuit."""
        if self.goal_reached or not self.path:
            return 0.0, 0.0

        found = False
        for i in range(self.idx, len(self.path)):
            px, py, _, p_ld, p_v = self.path[i]
            dist = math.hypot(px - curr_x, py - curr_y)
            if dist > p_ld:
                self.idx = i
                ld = p_ld
                v = p_v
                found = True
                break

        if not found:
            # End of path handling
            self.idx = len(self.path)
            if self.idx == 0:
                return 0.0, 0.0
            px, py, pyaw, ld, v = self.path[-1]
            dx = px - curr_x
            dy = py - curr_y
            rel_x = dx * math.cos(curr_h) + dy * math.sin(curr_h)
            rel_y = -dx * math.sin(curr_h) + dy * math.cos(curr_h)
            if abs(rel_x) <= self.pos_tol:
                self.goal_reached = True
                return 0.0, 0.0
            # Extend lookahead: intersect circle with line
            rel_yaw = pyaw - curr_h
            k_end = math.tan(rel_yaw)
            x_end = rel_x
            y_end = rel_y
            l_end = y_end - k_end * x_end
            a = 1.0 + k_end ** 2
            b = 2.0 * k_end * l_end  # Corrected
            c = l_end ** 2 - ld ** 2
            disc = b ** 2 - 4.0 * a * c
            if disc < 0:
                dist_end = math.hypot(rel_x, rel_y)
                if dist_end > 0:
                    scale = ld / dist_end
                    yt = rel_y * scale
                else:
                    yt = rel_y
            else:
                sqrt_disc = math.sqrt(disc)
                x_ld = (-b + math.copysign(sqrt_disc, rel_x)) / (2.0 * a)  # Choose forward direction
                y_ld = k_end * x_ld + l_end
                yt = y_ld
        else:
            # Lookahead point
            px, py, _, _, _ = self.path[self.idx]
            dx = px - curr_x
            dy = py - curr_y
            rel_x = dx * math.cos(curr_h) + dy * math.sin(curr_h)
            rel_y = -dx * math.sin(curr_h) + dy * math.cos(curr_h)
            yt = rel_y
            ld = self.path[self.idx][3]
            v = self.path[self.idx][4]

        # Compute steering
        ld2 = ld ** 2
        delta = math.atan2(2.0 * yt * self.L, ld2)
        delta = max(min(delta, self.delta_max), -self.delta_max)
        return delta, v

    def angle_to_pwm(self, delta):
        """Map steering angle to PWM (0=full left, 1023=full right, 512=straight)."""
        normalized = delta / self.delta_max  # -1 to 1
        pwm = 512.0 - 512.0 * normalized  # Inverts: positive delta (left?) to lower PWM
        return max(0, min(1023, int(round(pwm))))

    def send_cmd_vel(self, linear_x, angular_z_pwm):
        """Send CMD velocity via UDP as JSON."""
        cmd = {
            'linear_x': linear_x,
            'angular_z': angular_z_pwm,  # PWM value for steering
            'timestamp': time.time()
        }
        message = json.dumps(cmd).encode('utf-8')
        self.sock.sendto(message, (self.target_ip, self.target_port))
        self.messages_sent += 1

    def print_statistics(self):
        """Print stats every 5s."""
        if self.start_time is None:
            return
        current_time = time.time()
        if self.last_stats_time is None or current_time - self.last_stats_time >= 5.0:
            elapsed = current_time - self.last_stats_time if self.last_stats_time else current_time - self.start_time
            interval_messages = self.messages_sent - (self.last_stats_count or 0)
            actual_rate = interval_messages / elapsed if elapsed > 0 else 0

            total_elapsed = current_time - self.start_time
            overall_rate = self.messages_sent / total_elapsed if total_elapsed > 0 else 0

            print(f"\n=== SENDER STATISTICS ===")
            print(f"Messages sent: {self.messages_sent}")
            print(f"Target rate: {self.rate_hz:.1f} Hz")
            print(f"Actual rate (5s): {actual_rate:.1f} Hz")
            print(f"Overall rate: {overall_rate:.1f} Hz")
            print(f"Running time: {total_elapsed:.1f} seconds")
            print(f"=========================\n")

            self.last_stats_time = current_time
            self.last_stats_count = self.messages_sent

    def run_interactive(self):
        """Interactive mode: Read GPS from stdin, compute, send at input rate."""
        print("Pure Pursuit ready (interactive mode). Enter GPS data to compute and send UDP.")
        print("Format: lat lon heading_deg (e.g., 40.7128 -74.0060 90)")
        print("'q' to quit.\n")
        self.start_time = time.time()
        self.last_stats_time = time.time()

        while self.running:
            try:
                inp = input("GPS input (lat lon heading_deg) or 'q': ").strip()
                if inp.lower() == 'q':
                    break
                parts = inp.split()
                if len(parts) != 3:
                    print("Invalid input. Need exactly 3 values.")
                    continue
                lat = float(parts[0])
                lon = float(parts[1])
                h_deg = float(parts[2])
                h_rad = math.radians(h_deg)

                # Update pose
                self.last_x, self.last_y, self.last_h = self.gps_to_base(lat, lon, h_rad)
                delta, v = self.compute_steering(self.last_x, self.last_y, self.last_h)
                pwm = self.angle_to_pwm(delta)

                # Send
                self.send_cmd_vel(v, pwm)

                print(f"Base: x={self.last_x:.3f}m, y={self.last_y:.3f}m, h={math.degrees(self.last_h):.1f}°")
                print(f"Target idx={self.idx}/{len(self.path)}, δ={math.degrees(delta):.2f}°, PWM={pwm}, v={v:.2f}m/s")
                if self.goal_reached:
                    print("Goal reached! Stopping.")
                    break

                self.print_statistics()

            except ValueError:
                print("Invalid numeric input.")
            except Exception as e:
                print(f"Error: {e}")
        self.cleanup()

    def run_timed(self):
        """Timed mode: Send at 20 Hz using last known pose (for simulation/real GPS loop)."""
        if not self.path:
            print("No path loaded. Load a mission file first.")
            return

        print(f"Starting timed Pure Pursuit sender at {self.rate_hz} Hz")
        print(f"Target UDP: {self.target_ip}:{self.target_port}")
        print(f"Using last known pose for computation (update via external GPS input).")
        print("Press Ctrl+C to stop\n")

        self.start_time = time.time()
        self.last_stats_time = self.start_time
        next_send_time = self.start_time

        try:
            while self.running:
                current_time = time.time()

                if current_time >= next_send_time:
                    # Compute using last pose
                    delta, v = self.compute_steering(self.last_x, self.last_y, self.last_h)
                    pwm = self.angle_to_pwm(delta)

                    # Send
                    self.send_cmd_vel(v, pwm)

                    # Print every 100th for visibility
                    if self.messages_sent % 100 == 0:
                        print(f"Sent #{self.messages_sent}: v={v:.2f}, PWM={pwm} (δ={math.degrees(delta):.1f}°), idx={self.idx}")

                    next_send_time += self.period
                    if next_send_time < current_time:
                        next_send_time = current_time + self.period

                self.print_statistics()

                sleep_time = next_send_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup."""
        self.running = False
        if self.start_time:
            total_elapsed = time.time() - self.start_time
            overall_rate = self.messages_sent / total_elapsed if total_elapsed > 0 else 0
            print(f"\n=== FINAL STATISTICS ===")
            print(f"Total messages sent: {self.messages_sent}")
            print(f"Total time: {total_elapsed:.1f} seconds")
            print(f"Average rate: {overall_rate:.1f} Hz")
            print(f"========================")
        self.sock.close()


def main():
    parser = argparse.ArgumentParser(description='Pure Pursuit with UDP CMD Vel Sender')
    parser.add_argument('mission_file', nargs='?', help='Mission file path (required for timed mode)')
    parser.add_argument('--ip', default='127.0.0.1', help='Target IP (default: localhost)')
    parser.add_argument('--port', type=int, default=6004, help='UDP port (default: 6004)')
    parser.add_argument('--rate', type=float, default=20.0, help='Send rate Hz (default: 20.0)')
    parser.add_argument('--mode', choices=['interactive', 'timed'], default='interactive',
                        help='Run mode: interactive (stdin GPS) or timed (20Hz loop)')

    args = parser.parse_args()

    pp = PurePursuit(
        target_ip=args.ip,
        target_port=args.port,
        rate_hz=args.rate
    )

    if args.mode == 'timed' and not args.mission_file:
        print("Error: --mode timed requires mission_file argument.")
        sys.exit(1)

    if args.mission_file:
        pp.load_path(args.mission_file)

    # Signal handler
    def signal_handler(sig, frame):
        pp.cleanup()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    if args.mode == 'interactive':
        pp.run_interactive()
    else:
        pp.run_timed()

    # For testing: set initial pose if needed
    pp.last_x, pp.last_y, pp.last_h = 0.0, 0.0, 0.0  # Or simulate


if __name__ == "__main__":
    main()