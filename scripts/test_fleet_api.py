import math
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch
from fleet_api import DroneLink, ned_to_enu, position_covariance_enu


class ApiTests(unittest.TestCase):
    def setUp(self):
        self.connection = MagicMock()
        with patch('fleet_api.mavutil.mavlink_connection', return_value=self.connection), \
                patch('fleet_api.threading.Thread.start'):
            self.drone = DroneLink(1)

    def message(self, kind, sysid=2, component=1, **fields):
        return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: sysid,
                               get_srcComponent=lambda: component, **fields)

    def position(self, sysid=2, boot=1000):
        self.drone._handle(self.message('LOCAL_POSITION_NED', sysid, time_boot_ms=boot,
                          x=1, y=2, z=-3, vx=.1, vy=.2, vz=-.3), 10)

    def test_coordinate_conversion(self):
        self.assertEqual(ned_to_enu(1, 2, -3), [2, 1, 3])
        self.position()
        self.assertEqual(self.drone._estimate(10)['position_enu_m'], [2, 1, 3])
        self.drone._send_velocity(.2, .1, .3)
        args = self.connection.mav.set_position_target_local_ned_send.call_args.args
        self.assertEqual(args[1:3], (2, 1))
        self.assertEqual(args[8:11], (.1, .2, -.3))

    def test_covariance_rotation_and_missing(self):
        p = [0.] * 45
        p[0], p[9], p[17], p[1], p[2], p[10] = 4, 9, 16, 1, 2, 3
        self.assertEqual(position_covariance_enu(p), [[9, 1, -3], [1, 4, -2], [-3, -2, 16]])
        p[0] = math.nan
        self.assertIsNone(position_covariance_enu(p))
        self.assertFalse(self.drone._estimate(10)['uncertainty_valid'])

    def test_other_drone_is_ignored(self):
        self.position(sysid=1)
        self.assertIsNone(self.drone._pos)
        self.position()
        self.assertEqual(self.drone.received_positions, 1)

    def test_stale_ekf_and_reboot(self):
        self.position()
        self.drone._hb_seen = self.drone._ekf_seen = 10
        self.drone._ekf = 11
        self.assertTrue(self.drone._estimate(10)['valid'])
        self.assertFalse(self.drone._estimate(11)['valid'])
        self.drone.set_alignment([5, 0, 0], 2)
        self.position(boot=500)
        self.assertFalse(self.drone._estimate(10)['aligned'])

    def test_bad_velocity_rejected(self):
        for command in ((math.nan, 0, 0), (1, 0, 0), (0, 0, 1)):
            with self.assertRaises(ValueError):
                self.drone.send_velocity(*command)
        with self.assertRaises(RuntimeError):
            self.drone.send_velocity(0, 0, 0)

    def test_accuracy_summary_is_not_full_covariance(self):
        self.drone._handle(self.message('UTM_GLOBAL_POSITION',h_acc=1500,v_acc=200,vel_acc=30,flags=0),10)
        state=self.drone._estimate(10)
        self.assertEqual(state['accuracy_summary']['h_acc_m'],1.5)
        self.assertTrue(state['accuracy_summary']['valid'])
        self.assertFalse(state['uncertainty_valid'])
        self.assertIsNone(state['position_cov_enu_m2'])
        self.assertFalse(self.drone._estimate(13)['accuracy_summary']['valid'])
        self.drone._handle(self.message('UTM_GLOBAL_POSITION',h_acc=65535,v_acc=0,vel_acc=0,flags=0),14)
        self.assertFalse(self.drone._estimate(14)['accuracy_summary']['valid'])

    def test_expired_command_stops_and_stale_state_lands(self):
        self.position()
        self.drone._hb_seen = self.drone._ekf_seen = 10
        self.drone._ekf = 11
        self.drone._armed, self.drone._mode = True, 'GUIDED'
        self.drone._velocity = (9.5, .2, .1, 0)
        self.drone._control_step(10.1)
        self.assertTrue(self.drone._ttl_expired)
        args = self.connection.mav.set_position_target_local_ned_send.call_args.args
        self.assertEqual(args[8:11], (0, 0, 0))
        self.drone._control_step(11)
        self.connection.mav.set_mode_send.assert_called_with(2, 1, 9)
        self.assertIsNone(self.drone._velocity)


if __name__ == '__main__':
    unittest.main()
