"""Heading pipeline ported from the legacy code (commit 5df3f5b).

  HeadingFilter    utils/headingFilter.py KalmanFilter, body unchanged. camera_process feeds
                   it the ZED 2i magnetometer's magnetic_heading once per new IMU sample,
                   i.e. once per camera frame (~30 Hz), so its lag depends on the frame rate.
  zed_heading      camera_process lines 454-457: filter, then add the magnetic declination.
                   Legacy hard-coded +6 (Istanbul); it is a parameter now (RoboBoat in the
                   USA used -6).
  select_heading   nav_process lines 462-471: HEADING_SOURCE 'ZED', 'FC' or 'FUSED'.

test/test_heading.py checks these against the frozen legacy copies.
"""

LEGACY_DECLINATION_DEG = 6.0
SOURCES = ('ZED', 'FC', 'FUSED')


class HeadingFilter:
    """Scalar Kalman filter on a heading in degrees, wrap-around aware (legacy KalmanFilter).

    process_variance 1e-3 and measurement_variance 1e-1 are the values camera_process uses.
    """

    def __init__(self, process_variance=1e-3, measurement_variance=1e-1):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimated_value = 0.0
        self.estimated_error = 1.0
        self.is_initialized = False

    def update(self, measurement):
        if not self.is_initialized:
            self.estimated_value = measurement
            self.is_initialized = True
            return self.estimated_value

        # Handle wrap-around for heading
        diff = measurement - self.estimated_value
        while diff > 180.0:
            diff -= 360.0
        while diff < -180.0:
            diff += 360.0

        priori_error = self.estimated_error + self.process_variance
        kalman_gain = priori_error / (priori_error + self.measurement_variance)

        self.estimated_value = self.estimated_value + kalman_gain * diff

        # Keep within 0-360
        self.estimated_value = self.estimated_value % 360.0

        self.estimated_error = (1.0 - kalman_gain) * priori_error

        return self.estimated_value


def zed_heading(heading_filter, raw_magnetic_heading, declination_deg=LEGACY_DECLINATION_DEG):
    """One ZED sample: filtered magnetic heading plus declination, in [0, 360)."""
    filtered = heading_filter.update(raw_magnetic_heading)
    return (filtered + declination_deg) % 360


def signed_angle_difference(angle1, angle2):
    """Same as navigation.signed_angle_difference (legacy navigasyon)."""
    return (angle2 - angle1 + 180) % 360 - 180


def select_heading(source, zed_heading_deg, fc_heading_deg):
    """Heading the navigation uses.

    zed_heading_deg: legacy reads shared_state.get('zed_heading', 0.0), so a ZED that never
    reported gives 0.0 (north). The caller decides what to pass before the first sample.
    fc_heading_deg: None when the FC has not reported; then the ZED value is used.
    """
    if source == 'FC' and fc_heading_deg is not None:
        return fc_heading_deg
    if source == 'FUSED' and fc_heading_deg is not None:
        diff = signed_angle_difference(zed_heading_deg, fc_heading_deg)
        return (zed_heading_deg + (diff * 0.5)) % 360
    return zed_heading_deg
