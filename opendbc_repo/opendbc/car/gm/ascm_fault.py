"""Malibu ASCM harness on openpilot longitudinal: notice when the ASCM gives up its own picture of the car's motion.

The live ASCM feeds the radar and the front camera on the object bus. In the friend's Malibu (rlogs 0000004a--16 at
56.83 s and 00000042--9 at 49.58 s) it first marked its roll and pitch invalid (0x300) and 10 ms later its path curvature
(0x306) and its speed and yaw rate (0x308). Its own ACC, the camera's outputs and the radar's moving mode all went off with
it, and they stayed off until the car was restarted. openpilot keeps driving on the radar tracks, so nothing else tells
the driver that the stock forward-collision alert and automatic braking are gone.

0x308's LongVelInvalid and YawRateInvalid are GM "...V" validity bits (1 = invalid), placed as in cadillac_ct6_object.dbc
(F_Vehicle_Path_Data_2). In the 83 segments received so far they were set only in that fault, never at ignition on or
in a normal drive, and once set they never cleared within a drive. The cause of the fault is not known.
"""

ASCM_PATH_INVALID_FRAMES = 100  # 0x308 runs at 50 Hz: 2 s of invalid frames in a row


class AscmPathDataWatch:
  def __init__(self):
    self.invalid_frames = 0

  def update(self, long_vel_invalid, yaw_rate_invalid) -> bool:
    """Feed the 0x308 frames received since the last call; True once ASCM_PATH_INVALID_FRAMES in a row were invalid.
    A valid frame clears it. No frame keeps the last answer."""
    for long_vel, yaw_rate in zip(long_vel_invalid, yaw_rate_invalid, strict=True):
      if long_vel or yaw_rate:
        self.invalid_frames = min(self.invalid_frames + 1, ASCM_PATH_INVALID_FRAMES)
      else:
        self.invalid_frames = 0
    return self.invalid_frames >= ASCM_PATH_INVALID_FRAMES
