"""Shared settings must fail before launching; A failure remains visible to B."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import yaml
import inspect
from ai_inference.command_publisher import CommandPublisher
from intelion_interfaces.msg import ControlCommand

# A-side output checks (confidence/age limits in CommandPublisher) live in A's folder and
# are merged through A's own PR (CA-03 item 1). Until then these cases are skipped, not removed.
A_OUTPUT_CHECKS = 'confidence_min' in inspect.signature(CommandPublisher.__init__).parameters

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT.parent/'pac_worlds/worlds/franka_world.sdf'
spec = importlib.util.spec_from_file_location('shared_launch', ROOT/'launch/all.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


class SharedContractTest(unittest.TestCase):
    def config(self):
        return yaml.safe_load((ROOT/'config/scenario_config.yaml').read_text())

    def test_world_and_surface_agree(self):
        launch.validate_config(self.config(), WORLD)

    def test_mismatched_geometry_rejected(self):
        config = self.config(); config['planned_position'][2] = .705
        with self.assertRaises(ValueError): launch.validate_config(config, WORLD)

    def test_unsafe_or_inconsistent_settings_rejected(self):
        for key,value in [('command_ttl_ns',10000000), ('confidence_min',float('nan')),
                          ('accept_radius_m',.06), ('ai_ttl_ns',0), ('v_max_m_s',float('nan')),
                          ('camera_frame','wrong')]:
            with self.subTest(key=key):
                config = self.config(); config[key] = value
                with self.assertRaises(ValueError): launch.validate_config(config, WORLD)

    def make_publisher(self):
        if not A_OUTPUT_CHECKS:
            self.skipTest('A CommandPublisher has no confidence/age limits yet (CA-03 item 1)')
        class FakeNode:
            def create_publisher(self, *args): return self
            def publish(self, msg): self.last = msg
        return CommandPublisher(FakeNode(), confidence_min=.5, max_age_ns=200000000)

    def publish(self, pub, confidence=.5, age=0, status=ControlCommand.ST_OK):
        with patch('ai_inference.command_publisher.time.monotonic_ns', return_value=300000000):
            return pub.publish(0,status,[.5,0.,.71,0.],confidence,300000000-age)

    def test_low_nan_and_excess_confidence_fail(self):
        pub = self.make_publisher()
        for conf in [.49,float('nan'),1.01]:
            msg = self.publish(pub, confidence=conf)
            self.assertEqual(msg.status,ControlCommand.ST_NOT_DETECTED)
            self.assertEqual(msg.confidence,0.)

    def test_ttl_boundary_agrees_with_b_gate(self):
        pub = self.make_publisher()
        self.assertEqual(self.publish(pub, age=200000000).status,ControlCommand.ST_OK)
        for age in [-1,200000001]:
            self.assertEqual(self.publish(pub,age=age).status,ControlCommand.ST_SENSOR_INVALID)

    def test_failure_is_published_and_sequence_kept(self):
        pub = self.make_publisher()
        first = self.publish(pub)
        failed = self.publish(pub,status=ControlCommand.ST_NOT_DETECTED)
        self.assertEqual(failed.seq,first.seq+1)
        self.assertEqual(failed.writer_boot_id,first.writer_boot_id)
        self.assertEqual(failed.layout_version,1)
        self.assertEqual(failed.confidence,0.)


if __name__ == '__main__': unittest.main()
