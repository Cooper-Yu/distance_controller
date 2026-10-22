"""Per-ray binding preserves missing data and box-filter/physical-body distinction."""

from copy import deepcopy
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as N
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from route_scan_binding import bind_scan_pair


def scan(values):
    return N(
        header=N(stamp=N(sec=12, nanosec=34), frame_id='laser'),
        angle_min=0.0,
        angle_max=(len(values) - 1) * 0.1,
        angle_increment=0.1,
        time_increment=0.0001,
        scan_time=0.1,
        range_min=0.15,
        range_max=10.0,
        ranges=list(values),
    )


def mount():
    return N(
        header=N(stamp=N(sec=0, nanosec=0), frame_id='base_link'),
        child_frame_id='laser',
        transform=N(
            translation=N(x=0.02, y=0.0, z=0.17345), rotation=N(x=0.0, y=0.0, z=1.0, w=0.0)
        ),
    )


def manifest():
    return dict(
        schema=1,
        type='laser_filters/LaserScanBoxFilter',
        params=dict(
            box_frame='base_link',
            invert=False,
            min_x=-0.195,
            max_x=0.165,
            min_y=-0.145,
            max_y=0.145,
            min_z=-0.5,
            max_z=0.5,
        ),
    )


def bind(raw, filtered, **kwargs):
    return bind_scan_pair(
        raw,
        filtered,
        kwargs.get('tf', mount()),
        kwargs.get('mask', manifest()),
        kwargs.get('epoch', 'gid:fixture'),
    )


class ScanBindingTest(unittest.TestCase):
    def test_laser_pi_offset_and_box_exterior_point_preserved(self):
        result = bind(scan([0.2]), scan([math.nan]))
        ray = result['rays'][0]
        self.assertEqual(ray['raw_point_base_xyz'], (-0.18000000000000002, 0, 0.17345))
        self.assertTrue(ray['raw_point_inside_filter_box'])
        self.assertTrue(ray['supplied_box_would_remove'])
        self.assertAlmostEqual(ray['raw_planar_model_gap_m'], 0.01)
        self.assertFalse(ray['self_return_confirmed'])
        self.assertEqual(ray['category'], 'raw_finite_filtered_missing')

    def test_no_clipping_overlap_or_outside_box_near_return(self):
        for value, inside, positive_gap in ((0.16, True, False), (0.22, False, True)):
            ray = bind(scan([value]), scan([math.nan]))['rays'][0]
            self.assertEqual(ray['raw']['value'], value)
            self.assertEqual(ray['raw_point_inside_filter_box'], inside)
            self.assertEqual(ray['raw_planar_model_gap_m'] > 0, positive_gap)
            self.assertFalse(ray['self_return_confirmed'])

    def test_infinities_nan_and_out_of_range_stay_explicit(self):
        values = [math.inf, -math.inf, math.nan, 0.1, 11.0]
        report = bind(scan(values), scan(values))
        self.assertEqual(
            [r['raw']['kind'] for r in report['rays']],
            ['positive_inf', 'negative_inf', 'nan', 'below_min', 'above_max'],
        )
        self.assertEqual(report['counts'], {'both_missing': 5})
        self.assertTrue(all(r['raw_point_base_xyz'] is None for r in report['rays']))
        json.dumps(report, allow_nan=False)
        self.assertFalse(report['executable'])
        self.assertFalse(report['certified_free'])
        self.assertNotIn('motion_authorized', report)

    def test_filtered_only_and_changed_finite_measurements_not_silently_merged(self):
        report = bind(scan([math.inf, 1.0, 2.0]), scan([0.5, 1.1, 2.0]))
        self.assertEqual(
            [r['category'] for r in report['rays']],
            ['filtered_finite_raw_missing', 'finite_changed', 'finite_retained'],
        )
        self.assertIsNone(report['rays'][0]['raw_point_base_xyz'])
        self.assertIsNotNone(report['rays'][0]['filtered_point_base_xyz'])
        self.assertNotEqual(
            report['rays'][1]['raw_point_base_xyz'], report['rays'][1]['filtered_point_base_xyz']
        )

    def test_stamp_frame_count_and_every_geometry_field_mismatch_refused(self):
        from route_scan_binding import GEOMETRY

        mutations = [
            lambda s: setattr(s.header.stamp, 'nanosec', 35),
            lambda s: setattr(s.header, 'frame_id', 'other'),
            lambda s: s.ranges.append(1.0),
        ]
        mutations += [
            lambda s, key=key: setattr(s, key, getattr(s, key) + 0.00001) for key in GEOMETRY
        ]
        for mutate in mutations:
            raw, filtered = scan([1.0, 2.0]), scan([1.0, 2.0])
            mutate(filtered)
            with self.assertRaises(ValueError):
                bind(raw, filtered)

    def test_bad_geometry_empty_scan_epoch_and_stamp_refused(self):
        for mutate in (
            lambda s: setattr(s, 'angle_increment', 0),
            lambda s: setattr(s, 'angle_max', 9),
            lambda s: setattr(s, 'range_max', math.inf),
            lambda s: setattr(s.header.stamp, 'nanosec', -1),
            lambda s: setattr(s, 'time_increment', -0.1),
        ):
            raw = scan([1.0, 2.0])
            mutate(raw)
            with self.assertRaises(ValueError):
                bind(raw, deepcopy(raw))
        for raw, epoch in ((scan([]), 'gid'), (scan([1.0]), ''), (scan([1.0]), None)):
            with self.assertRaises(ValueError):
                bind(raw, deepcopy(raw), epoch=epoch)

    def test_tf_identity_time_and_quaternion_required(self):
        mutations = [
            lambda t: setattr(t.header, 'frame_id', 'odom'),
            lambda t: setattr(t, 'child_frame_id', 'other'),
            lambda t: setattr(t.header.stamp, 'sec', 11),
            lambda t: setattr(t.transform.translation, 'x', math.nan),
            lambda t: setattr(t.transform.rotation, 'z', 0),
        ]
        for mutate in mutations:
            tf = mount()
            mutate(tf)
            with self.assertRaises(ValueError):
                bind(scan([1]), scan([1]), tf=tf)
        tf = mount()
        tf.header.stamp = N(sec=12, nanosec=34)
        self.assertEqual(
            bind(scan([1]), scan([1]), tf=tf)['transform']['timing'], 'scan_stamp_matched'
        )

    def test_mask_manifest_is_explicit_and_hash_changes(self):
        original = manifest()
        for change in ({'box_frame': 'laser'}, {'invert': 0}, {'min_x': math.nan}, {'min_y': 1.0}):
            changed = deepcopy(original)
            changed['params'].update(change)
            with self.assertRaises(ValueError):
                bind(scan([1]), scan([1]), mask=changed)
        report = bind(scan([1]), scan([1]), mask=original)
        original['params']['min_x'] = -0.19
        other = bind(scan([1]), scan([1]), mask=original)
        self.assertNotEqual(report['filter_manifest_sha256'], other['filter_manifest_sha256'])
        self.assertEqual(report['filter_manifest']['params']['min_x'], -0.195)
        self.assertFalse(report['active_filter_verified'])
        self.assertFalse(report['live_freshness_verified'])

    def test_strict_box_boundary_and_invert_are_not_self_classifiers(self):
        mask = manifest()
        mask['params']['min_x'] = 0.02 - 0.2
        ray = bind(scan([0.2]), scan([math.nan]), mask=mask)['rays'][0]
        self.assertFalse(ray['raw_point_inside_filter_box'])
        self.assertFalse(ray['supplied_box_would_remove'])
        mask['params']['invert'] = True
        other = bind(scan([0.2]), scan([math.nan]), mask=mask)['rays'][0]
        self.assertTrue(other['supplied_box_would_remove'])
        self.assertFalse(other['self_return_confirmed'])

    def test_input_arrays_remain_unchanged_and_zero_is_not_valid_range(self):
        raw, filtered, tf, mask = (
            scan([0, math.inf, 0.2]),
            scan([0, math.inf, math.nan]),
            mount(),
            manifest(),
        )
        before = repr(raw), repr(filtered), repr(tf), deepcopy(mask)
        result = bind(raw, filtered, tf=tf, mask=mask)
        self.assertEqual(before, (repr(raw), repr(filtered), repr(tf), mask))
        self.assertEqual(result['rays'][0]['raw']['kind'], 'below_min')
        self.assertEqual([r['index'] for r in result['rays']], [0, 1, 2])


if __name__ == '__main__':
    unittest.main()
