"""Focused offline checks for evidence grouping, signs, boundaries and failures."""
import unittest
import numpy as np
from analyze_encoder_rtk_20261120 import independent_positions, wheel_series, ticks_between, enu, wrap


def encoder(t, ms, ticks=100, session='a', contiguous=1):
    return dict(source=dict(session=session), payload=dict(side='L',monotonic_s=t,
                device_ms=ms,delta_ticks=ticks,contiguous=contiguous))


class EvidenceTests(unittest.TestCase):
    def test_republished_position_and_identical_new_fix(self):
        rows=[]
        for t,decode in ((10.01,10.),(10.02,10.),(10.21,10.2)):
            rows.append(dict(source=dict(session='a'),payload=dict(monotonic_s=t,
                gnss_position_age_s=t-decode,gnss_lat=40.,gnss_lon=-80.,utc='x')))
        result=independent_positions(rows)
        self.assertEqual(len(result),2) # same coordinates still represent two decode events
        self.assertAlmostEqual(result[1]['t'],10.2)

    def test_fractional_boundaries_and_signed_reverse(self):
        w=wheel_series([encoder(1.,100,-100),encoder(1.1,200,-100),encoder(1.2,300,-100)],'L')
        self.assertAlmostEqual(ticks_between(w,1.05,1.15),-100)
        self.assertIsNone(ticks_between(w,.9,1.15))

    def test_no_bridge_over_missing_samples(self):
        w=wheel_series([encoder(1.,100),encoder(1.1,200),encoder(2.,1100)],'L')
        self.assertIsNone(ticks_between(w,1.05,2.))

    def test_device_reset_and_producer_restart(self):
        for third in (encoder(1.2,0),encoder(1.2,300,session='b'),encoder(1.2,300,contiguous=0)):
            w=wheel_series([encoder(1.,100),encoder(1.1,200),third],'L')
            self.assertIsNone(ticks_between(w,1.05,1.2))

    def test_device_uint32_wrap_is_continuous(self):
        w=wheel_series([encoder(1.,2**32-50),encoder(1.1,50)],'L')
        self.assertAlmostEqual(ticks_between(w,1.,1.1),100)

    def test_stationary_signed_jitter_cancels(self):
        w=wheel_series([encoder(1.,100,0),encoder(1.1,200,4),encoder(1.2,300,-4)],'L')
        self.assertEqual(ticks_between(w,1.,1.2),0)

    def test_reference_frame_and_heading_wrap(self):
        xy=enu([40.,40.00001],[ -80.,-79.99999],(40.,-80.))
        self.assertTrue(np.allclose(xy[0],[0,0]))
        self.assertGreater(xy[1,0],0); self.assertGreater(xy[1,1],0)
        self.assertEqual(wrap(359-1),-2)


if __name__=='__main__': unittest.main()
