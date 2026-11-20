# Production-folder placement reviewed 2026-11-20; runtime logic unchanged.
"""ISM330DHCX acquisition and pure integration helpers."""
import math
import statistics
import struct

GYRO_SCALE = 0.00875

ACCEL_SCALE = 6.1e-05

GNSS_FIELDS = ('lat', 'lon', 'fix_quality', 'speed_mps', 'speed_timestamp', 'heading_deg', 'headValid', 'carrier', 'relpos_length_m', 'relpos_heading_accuracy_deg', 'expectedErrDeg', 'relposned_count', 'relposned_timestamp', 'relpos_valid', 'relpos_gnss_fix_ok', 'relpos_ref_pos_miss', 'relpos_ref_obs_miss', 'timestamp', 'fatal_error')

FIELDS = ('utc', 'monotonic_s', 'sample', 'phase', 'dt_s', 'timing_gap', 'gyro_x_raw', 'gyro_y_raw', 'gyro_z_raw', 'accel_x_raw', 'accel_y_raw', 'accel_z_raw', 'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps', 'accel_x_g', 'accel_y_g', 'accel_z_g', 'gyro_x_corrected_dps', 'gyro_y_corrected_dps', 'gyro_z_corrected_dps', 'relative_yaw_ccw_deg', 'yaw_segment', 'imu_compass_once_anchored_deg', 'imu_minus_gnss_deg', 'gnss_packet_age_s', 'gnss_heading_age_s', 'gnss_heading_usable', 'gnss_socket_available') + tuple(('gnss_' + key for key in GNSS_FIELDS))

def finite(value):
    return isinstance(value, (int, float)) and (not isinstance(value, bool)) and math.isfinite(value)

def compass_from_yaw(anchor_heading, yaw_since_anchor):
    return (anchor_heading - yaw_since_anchor) % 360.0

def decode_sample(block):
    if len(block) != 12:
        raise ValueError('IMU sample must contain exactly 12 bytes')
    raw = struct.unpack('<6h', bytes(block))
    return (raw, tuple((x * GYRO_SCALE for x in raw[:3])), tuple((x * ACCEL_SCALE for x in raw[3:])))

def bias_statistics(samples):
    if len(samples) < 2:
        raise ValueError('not enough stationary samples')
    axes = list(zip(*samples))
    if len(axes) != 3 or any((not finite(v) for axis in axes for v in axis)):
        raise ValueError('invalid gyro calibration samples')
    return ([statistics.mean(a) for a in axes], [statistics.stdev(a) for a in axes])

class Sensor:

    def __init__(self, bus):
        self.bus = bus
        self.address = 107

    def configure(self):
        identity = self.bus.read_byte_data(self.address, 15)
        if identity != 107:
            raise RuntimeError(f'WHO_AM_I expected 0x6b, got {identity:#04x}')
        for register, value in ((18, 68), (16, 64), (17, 64)):
            self.bus.write_byte_data(self.address, register, value)
            if self.bus.read_byte_data(self.address, register) != value:
                raise RuntimeError(f'IMU configuration readback failed at {register:#04x}')

    def read(self):
        if self.bus.read_byte_data(self.address, 30) & 3 != 3:
            return None
        return decode_sample(self.bus.read_i2c_block_data(self.address, 34, 12))

class YawTracker:

    def __init__(self):
        self.yaw = 0.0
        self.segment = 0
        self.previous_rate = None
        self.anchor_heading = None
        self.anchor_yaw = None

    def update(self, rate, delta, heading=None):
        gap = delta > 0.05
        if gap:
            self.segment += 1
            self.yaw = 0.0
            self.anchor_heading = self.anchor_yaw = None
            self.previous_rate = None
        if self.previous_rate is not None:
            self.yaw += 0.5 * (self.previous_rate + rate) * delta
        self.previous_rate = rate
        if self.anchor_heading is None and heading is not None:
            self.anchor_heading, self.anchor_yaw = (heading, self.yaw)
        compass = None if self.anchor_heading is None else compass_from_yaw(self.anchor_heading, self.yaw - self.anchor_yaw)
        return (gap, compass)
