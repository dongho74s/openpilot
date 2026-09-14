"""Source contracts for the two comma 4 audio register faults observed at boot.

These tests do not emulate STM32 hardware; firmware build and live Panda health
must additionally be checked on a parked device.
"""
from pathlib import Path
import re
import unittest

BOARD = Path(__file__).resolve().parents[1] / "board"


class TestCuatroSoundInit(unittest.TestCase):
  def setUp(self):
    self.source = (BOARD / "stm32h7/sound.h").read_text(errors="replace")
    self.init = self.source.split("void sound_init(void) {", 1)[1]

  def test_only_clocked_sai4_is_configured(self):
    # Cuatro uses SAI4 A/B; SAI1 is not clocked in peripherals_init().
    self.assertNotRegex(self.init, r"&SAI1(?:->|_Block_)")
    self.assertIn("&SAI4->GCR", self.init)

  def test_microphone_shift_precedes_channel_enable(self):
    shift = re.search(r"register_set\(&DFSDM1_Channel3->CHCFGR2,.*?;", self.init, re.S)
    enable = re.search(r"register_set\(&DFSDM1_Channel3->CHCFGR1,.*?DFSDM_CHCFGR1_CHEN.*?;", self.init, re.S)
    self.assertIsNotNone(shift)
    self.assertIsNotNone(enable)
    self.assertLess(shift.start(), enable.start(), "DTRBS is write-protected once CHEN is set")
    self.assertIn("DFSDM_CHCFGR2_DTRBS_Msk", shift.group())
    self.assertIn("DFSDM_CHCFGR2_OFFSET_Msk", shift.group())


if __name__ == "__main__":
  unittest.main()
