import json
import unittest
from base_oracle import actual_base_epochs


class BaseOracleTest(unittest.TestCase):
    def row(self, fraction, base):
        return json.dumps(dict(event='pair', rover=dict(time=dict(seconds=1400000000, fraction=fraction)), base=base))

    def test_raw_base_keeps_its_time_even_when_preprocessing_uses_dr(self):
        base = dict(time=dict(seconds=1400000000, fraction=0.0), sats=[])
        self.assertEqual(actual_base_epochs([self.row(.8, base)]),
                         {1400000000800: 1400000000000})

    def test_missing_base_and_precise_week_boundary(self):
        self.assertEqual(actual_base_epochs([self.row(.001, None)]),
                         {1400000000001: -1})

    def test_duplicate_rover_is_not_silently_overwritten(self):
        row = self.row(0., None)
        with self.assertRaises(ValueError):
            actual_base_epochs([row, row])


if __name__ == '__main__':
    unittest.main()
