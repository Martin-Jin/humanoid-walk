import tempfile
import unittest
from pathlib import Path

from humanoid_walk.utils.device import resolve_device
from humanoid_walk.utils.paths import new_run_dir
from humanoid_walk.utils.seeding import make_rng, spawn_seeds


class SeedingTests(unittest.TestCase):
    def test_same_seed_gives_same_numbers(self):
        self.assertEqual(make_rng(7).random(), make_rng(7).random())

    def test_spawned_seeds_are_deterministic_and_distinct(self):
        seeds = spawn_seeds(3, 4)
        self.assertEqual(seeds, spawn_seeds(3, 4))
        self.assertEqual(len(set(seeds)), 4)


class DeviceTests(unittest.TestCase):
    def test_cpu_is_always_available(self):
        self.assertEqual(resolve_device("cpu"), "cpu")

    def test_unknown_preference_is_rejected(self):
        with self.assertRaises(ValueError):
            resolve_device("tpu")


class PathsTests(unittest.TestCase):
    def test_new_run_dir_creates_a_folder(self):
        with tempfile.TemporaryDirectory() as root:
            path = new_run_dir(Path(root))
            self.assertTrue(path.is_dir())
            self.assertTrue(path.name.startswith("run-"))


if __name__ == "__main__":
    unittest.main()
