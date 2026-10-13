import ast
import math
from pathlib import Path
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from runway_geometry import (
    EARTH_RADIUS_M, OPPOSITE_RUNWAY, RUNWAY_THRESHOLDS,
    detect_runway, distance_and_bearing, runway_deviations, valid_position,
)


def destination(start, bearing, meters):
    lat, lon = map(math.radians, start)
    distance = meters / EARTH_RADIUS_M
    lat2 = math.asin(math.sin(lat) * math.cos(distance)
                     + math.cos(lat) * math.sin(distance) * math.cos(bearing))
    lon2 = lon + math.atan2(math.sin(bearing) * math.sin(distance) * math.cos(lat),
                            math.cos(distance) - math.sin(lat) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


class RunwayGeometryTests(unittest.TestCase):
    def test_all_thresholds_and_centerlines(self):
        for name, start in RUNWAY_THRESHOLDS.items():
            with self.subTest(runway=name):
                end = RUNWAY_THRESHOLDS[OPPOSITE_RUNWAY[name]]
                distance, bearing = distance_and_bearing(start, end)
                self.assertEqual(detect_runway(*start), name)
                for position in (start, end, destination(start, bearing, distance * EARTH_RADIUS_M / 2)):
                    self.assertEqual(runway_deviations(name, *position, math.degrees(bearing)), (0, 0.0))

    def test_signed_offsets_and_reciprocal_direction(self):
        for name, start in RUNWAY_THRESHOLDS.items():
            _, bearing = distance_and_bearing(start, RUNWAY_THRESHOLDS[OPPOSITE_RUNWAY[name]])
            for meters in (-25, 25):
                position = destination(start, bearing + math.pi / 2, meters)
                offset, _ = runway_deviations(name, *position, 0)
                reverse_offset, _ = runway_deviations(OPPOSITE_RUNWAY[name], *position, 0)
                self.assertEqual(offset, meters)
                self.assertEqual(reverse_offset, -meters)

    def test_heading_sign_and_wraparound(self):
        start = RUNWAY_THRESHOLDS['06R']
        _, bearing = distance_and_bearing(start, RUNWAY_THRESHOLDS['24L'])
        heading = math.degrees(bearing)
        for delta in (-179, -10, 0, 10, 179, 181, 359):
            _, actual = runway_deviations('06R', *start, (heading + delta) % 360)
            self.assertEqual(actual, (delta + 180) % 360 - 180)

    def test_invalid_samples(self):
        for sample in ((None, -73, 42), (float('nan'), -73, 42), (45, -73, float('inf')),
                       (91, -73, 42), (45, -181, 42), (0, 0, 0)):
            self.assertFalse(valid_position(*sample))


class RunwayIntegrationTests(unittest.TestCase):
    def setUp(self):
        # Load only these functions: importing main would start hardware and the GUI.
        tree = ast.parse(Path('main_aircraft_agent.py').read_text(encoding='utf-8'))
        names = {'reset_runway_detection', 'update_runway_outputs', 'get_position',
                 '_collect_output_values', '_push_output_values'}
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
        self.agent = SimpleNamespace()
        self.env = dict(agent=self.agent, current_runway=None, runway_ready_at=0,
                        runway_lock=threading.Lock(), time=time, detect_runway=detect_runway,
                        runway_deviations=runway_deviations, valid_position=valid_position)
        exec(compile(ast.Module(body=functions, type_ignores=[]), '<agent functions>', 'exec'), self.env)

    def test_reset_wait_retry_and_latched_runway(self):
        update = self.env['update_runway_outputs']
        update(*RUNWAY_THRESHOLDS['06R'], 42, time.monotonic())
        self.assertEqual(self.agent.current_runway_o, '06R')
        update(*RUNWAY_THRESHOLDS['24L'], 42, time.monotonic())
        self.assertEqual(self.agent.current_runway_o, '06R')
        self.env['reset_runway_detection']()
        ready = self.env['runway_ready_at']
        self.assertEqual(self.agent.current_runway_o, '')
        update(*RUNWAY_THRESHOLDS['24R'], 222, ready - .1)
        self.assertEqual(self.agent.current_runway_o, '')
        update(None, None, None, ready + .1)
        self.assertEqual(self.agent.current_runway_o, '')
        update(*RUNWAY_THRESHOLDS['24R'], 222, ready + .2)
        self.assertEqual(self.agent.current_runway_o, '24R')

    def test_real_ingescape_types_setters_and_resync(self):
        import ingescape as igs
        from echo_aircraft_agent import Echo
        outputs = [('current_runway', igs.STRING_T),
                   ('runway-centerline deviation', igs.INTEGER_T),
                   ('heading_deviation', igs.DOUBLE_T)]
        try:
            for name, kind in outputs:
                igs.output_create(name, kind, None)
                self.assertTrue(igs.output_exists(name))
            agent = Echo()
            self.env['agent'] = agent
            agent.current_runway_o = '24L'
            agent.runway_centerline_deviation_o = -25
            agent.heading_deviation_o = -2.5
            values = self.env['_collect_output_values']()
            self.env['_push_output_values'](values)
            self.assertEqual(igs.output_string('current_runway'), '24L')
            self.assertEqual(igs.output_int('runway-centerline deviation'), -25)
            self.assertEqual(igs.output_double('heading_deviation'), -2.5)
        finally:
            for name, _ in outputs:
                igs.output_remove(name)

    def test_position_uses_true_heading_and_preserves_magnetic_output(self):
        client = MagicMock()
        start = RUNWAY_THRESHOLDS['06R']
        _, bearing = distance_and_bearing(start, RUNWAY_THRESHOLDS['24L'])
        client.__enter__.return_value.getPOSI.return_value = (
            *start, 30, 0, 0, math.degrees(bearing), 1)
        self.env['xpc'] = SimpleNamespace(XPlaneConnect=lambda: client)
        result = self.env['get_position'](alt=100, heading=57)
        self.assertEqual(result[1], 57)
        self.assertEqual(self.agent.heading_deviation_o, 0.0)

    def test_position_failure_does_not_select_a_runway(self):
        client = MagicMock()
        client.__enter__.return_value.getPOSI.side_effect = TimeoutError('no simulator')
        self.env['xpc'] = SimpleNamespace(XPlaneConnect=lambda: client)
        self.env['get_position'](alt=100, heading=57)
        self.assertIsNone(self.env['current_runway'])


if __name__ == '__main__':
    unittest.main()
