# Joystick

**Hardware needed**: device running openpilot, laptop, joystick (optional)

With joystick_control, you can connect your laptop to your comma device over the network and debug controls using a joystick or keyboard.
joystick_control uses [inputs](https://pypi.org/project/inputs) which supports many common gamepads and joysticks.

## Usage

The car must be off, and openpilot must be offroad before starting `joystick_control`.

### Using a keyboard

SSH into your comma device and start joystick_control with the following command:

```shell
tools/joystick/joystick_control.py --keyboard
```

The available buttons and axes will print showing their key mappings. In general, the WASD keys control gas and brakes and steering torque in 5% increments.

### Joystick on your comma three

Plug the joystick into your comma three aux USB-C port. Then, SSH into the device and start `joystick_control.py`.

### Joystick on your laptop

In order to use a joystick over the network, we need to run joystick_control locally from your laptop and have it send `testJoystick` packets over the network to the comma device.

1. Connect a joystick to your PC.
2. Connect your laptop to your comma device's hotspot and open a new SSH shell. Since joystick_control is being run on your laptop, we need to write a parameter to let controlsd know to start in joystick debug mode:
   ```shell
   # on your comma device
   echo -n "1" > /data/params/d/JoystickDebugMode
   ```
3. Run bridge with your laptop's IP address. This republishes the `testJoystick` packets sent from your laptop so that openpilot can receive them:
   ```shell
   # on your comma device
   cereal/messaging/bridge {LAPTOP_IP} testJoystick
   ```
4. Start joystick_control on your laptop in ZMQ mode.
   ```shell
   # on your laptop
   export ZMQ=1
   tools/joystick/joystick_control.py
   ```

---
Now start your car and openpilot should go into joystick mode with an alert on startup! The status of the axes will display on the alert, while button statuses print in the shell.

Make sure the conditions are met in the panda to allow controls (e.g. cruise control engaged). You can also make a modification to the panda code to always allow controls.

![](https://github.com/commaai/openpilot/assets/8762862/e640cbca-cb7a-4dcb-abce-b23b036ad8e7)

## DJI RC 2 on the same network (carrotpilot)

Turn on **System → Remote Controller → DJI RC 2 remote control** (`Rc2RemoteControl`) in Carrot web. While it is on,
manager runs `rc2_receiver` (also offroad), and a drive started with it on runs `joystickd` instead of `controlsd`.
The switch is read once at ignition, so changing it mid-drive applies from the next drive; a reboot turns it off.

- The RC 2 app (`rc2-openpilot-joystick`, outside this repo) reads the sticks from the DJI input stream on
  `127.0.0.1:40007`, finds the comma by broadcasting on its network (falling back to the last known address and a /24
  sweep) and talks to `rc2_receiver` over UDP 7720. The wire format is in `rc2_protocol.py`; every packet is signed with
  the key in `/data/rc2_joystick/key`, which the receiver creates the first time it runs. Pair the app once with
  `pair.sh` in the app folder.
- Left stick up/down is gas/brake, right stick left/right is steering. Full stick asks for the most openpilot can
  request (+2.0 / -4.0 m/s^2, full steering torque), with no speed cap; the car controller and the panda still apply
  their own limits.
- The sticks pass through only while the app is armed and its dead-man button (shutter by default) is held. On link loss
  (0.3 s), dead-man release or stale stick data the receiver brakes at 1.5 m/s^2 with zero torque, and both sticks must
  return to centre before they pass through again. If the receiver itself stops, `joystickd` brakes at 1.5 m/s^2 too.
- Engaging still needs someone in the driver's seat pressing the cruise buttons, and brake, seatbelt, door and driver
  monitoring still disengage. Use it only in a closed area.
- `rc2_receiver.py --dry-run --key <file>` runs it on a computer without a car, to check the app end to end.
