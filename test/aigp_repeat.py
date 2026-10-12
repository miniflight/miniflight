from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from target.aigp.client import Packet
from target.aigp.experiments.repeat import fresh_race
from target.aigp.native import NativeRaceState


class ResetTest(unittest.TestCase):
    def test_accepted_request_without_a_reset_is_retried_before_handoff(self):
        clock, client, reader = [0.0], Mock(), Mock()
        client.connected, client.gates = True, None
        ack = SimpleNamespace(get_type=lambda: 'COMMAND_ACK', command=31000, result=0)
        def tick():
            clock[0] += .2
            return clock[0]
        def poll(timeout):
            client.telemetry = {'LOCAL_POSITION_NED': SimpleNamespace(x=0, y=0, z=0, vx=0, vy=0, vz=0, _host_received_at=clock[0])}
            return (Packet(ack, clock[0]),)
        def states():
            fresh = client.reset.call_count >= 2  # The native stub ignores the first accepted request.
            return (NativeRaceState(started=not fresh, valid=True, completed=not fresh,
                    time_seconds=0 if fresh else 25, active_gate_index=0 if fresh else 6,
                    finish_time_seconds=-1 if fresh else 25, received_at=clock[0]),)
        client.poll.side_effect, reader.poll.side_effect = poll, states
        with patch('target.aigp.experiments.repeat.SimulatorClient', return_value=client), \
             patch('target.aigp.experiments.repeat.RaceStateReader', return_value=reader), \
             patch('target.aigp.experiments.repeat.time.monotonic', side_effect=tick):
            transferred, reset, course = fresh_race('unused', timeout=10, course_packets=('fixed course',))
        self.assertEqual(reset['requests'], 2)
        self.assertTrue(reset['before']['completed'])
        self.assertFalse(reset['after']['started'])
        self.assertEqual(reset['after']['time_seconds'], 0)
        self.assertIs(transferred, reader)
        client.disconnect.assert_called_once()
        reader.close.assert_not_called()
