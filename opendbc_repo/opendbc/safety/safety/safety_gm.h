#pragma once

#include "safety_declarations.h"

// TODO: do checksum and counter checks. Add correct timestep, 0.1s for now.
#define GM_COMMON_RX_CHECKS \
    {.msg = {{0x184, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}}, \
    {.msg = {{0x34A, 0, 5, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}}, \
    {.msg = {{0x1E1, 0, 7, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}}, \
    {.msg = {{0x1C4, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}}, \
    {.msg = {{0xC9, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}}, \

#define GM_ACC_RX_CHECKS \
    {.msg = {{0xBE, 0, 6, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U},    /* Volt, Silverado, Acadia Denali */ \
             {0xBE, 0, 7, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U},    /* Bolt EUV */ \
             {0xBE, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}}},  /* Escalade */ \

static const LongitudinalLimits *gm_long_limits;

enum {
  GM_BTN_UNPRESS = 1,
  GM_BTN_RESUME = 2,
  GM_BTN_SET = 3,
  GM_BTN_MAIN = 5,
  GM_BTN_CANCEL = 6,
};

typedef enum {
  GM_ASCM,
  GM_CAM
} GmHardware;
static GmHardware gm_hw = GM_ASCM;
static bool gm_cam_long = false;
static bool gm_pcm_cruise = false;
static bool gm_has_acc = true;
static bool gm_pedal_long = false;
static bool gm_cc_long = false;
static bool gm_skip_relay_check = false;
static bool gm_force_ascm = false;
static bool gm_sdgm = false;
static bool gm_manual_hold = false;
// Comma between the ASCM and the powertrain bus (CHEVROLET_MALIBU_ASCM, StarPilot's ASCM_INT): forwarded like a camera
// harness, but the brake is read from ECMAcceleratorPos (0xBE) as on the ASCM path.
static bool gm_ascm_int = false;
// The same car without ECMAcceleratorPos: brake from 0xC9, and 0xF1 instead of 0xBE in the RX checks.
static bool gm_ascm_int_c9_brake = false;

// The panda keeps the real PSCMStatus (0x184) from the Malibu's ASCM and openpilot relays a copy. When that copy stops
// while the panda stays in this mode (openpilot stopped with the engine on, up to the 5 s heartbeat timeout), the ASCM
// loses the steering module, latches U0131 for the ignition cycle and its ACC stops following (Malibu 00000042,
// 2026-11-23). So the real frame passes whenever openpilot's copy is older than 300 ms (three missed 10 Hz copies).
#define GM_ASCM_INT_PSCM_STALE_US 300000U
static bool gm_ascm_int_pscm_tx_seen = false;
static uint32_t gm_ascm_int_pscm_tx_us = 0U;

static bool gm_ascm_int_pscm_relayed(void) {
  // A stale copy is forgotten, so the 32-bit timer wrapping (~72 min) cannot make it look fresh again.
  if (gm_ascm_int_pscm_tx_seen &&
      (get_ts_elapsed(microsecond_timer_get(), gm_ascm_int_pscm_tx_us) > GM_ASCM_INT_PSCM_STALE_US)) {
    gm_ascm_int_pscm_tx_seen = false;
  }
  return gm_ascm_int_pscm_tx_seen;
}

// Bolt with the factory camera ACC and a comma pedal (CHEVROLET_BOLT_ACC_PEDAL), ported from StarPilot's
// BOLT_2022_PEDAL + PANDA_PADDLE_SCHED path (firestar5683/StarPilot 2589caad gm.h).
static bool gm_bolt_acc_pedal = false;

// The EBCM sends the regen paddle (0xBD) and the ECM the PRNDL2 (0x1F5) on the powertrain bus themselves, so a host
// frame sent on its own timing alternates with the stock one and the regen request stutters on/off. For this car the
// host frames are buffered instead, and the panda sends the latest one right after each stock frame, through the tx
// hook again so a stale apply frame cannot leave once controls are off. A feed older than 100 ms is not sent.
#define GM_SPOOF_FEED_STALE_US 100000U
typedef struct {
  bool valid;
  uint8_t data[8];
  uint32_t last_feed_us;
} GmStockSyncedSpoof;
static GmStockSyncedSpoof gm_paddle_spoof;
static GmStockSyncedSpoof gm_prndl2_spoof;
static bool gm_spoof_internal_tx = false;

// Defined by the CAN driver, which the firmware includes after the safety code.
void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook);
void can_set_checksum(CANPacket_t *packet);

static void gm_spoof_reset(GmStockSyncedSpoof *spoof) {
  spoof->valid = false;
  spoof->last_feed_us = 0U;
  for (int i = 0; i < 8; i++) {
    spoof->data[i] = 0U;
  }
}

static void gm_spoof_feed(GmStockSyncedSpoof *spoof, const CANPacket_t *msg, int len) {
  for (int i = 0; i < 8; i++) {
    spoof->data[i] = (i < len) ? GET_BYTE(msg, i) : 0U;
  }
  spoof->valid = true;
  spoof->last_feed_us = microsecond_timer_get();
}

static void gm_spoof_send_after_stock(const GmStockSyncedSpoof *spoof, int addr, int len) {
  if (spoof->valid && (get_ts_elapsed(microsecond_timer_get(), spoof->last_feed_us) <= GM_SPOOF_FEED_STALE_US)) {
    CANPacket_t to_send = {0};
    to_send.addr = (uint32_t)addr;
    to_send.bus = 0U;
    to_send.data_len_code = (uint8_t)len;
    for (int i = 0; i < len; i++) {
      to_send.data[i] = spoof->data[i];
    }
    can_set_checksum(&to_send);
    gm_spoof_internal_tx = true;
    can_send(&to_send, 0U, false);
    gm_spoof_internal_tx = false;
  }
}

static struct {
  bool armed;
  bool active;
  bool low_speed;
  bool cruise_active;
  uint8_t pedal;
  uint8_t seen;
  uint32_t ts[4];
} gm_hold;

static void gm_hold_reset(void) {
  gm_hold.armed = false;
  gm_hold.active = false;
}

static bool gm_hold_inputs_fresh(void) {
  bool fresh = gm_hold.seen == 0xFU;
  for (int i = 0; i < 4; i++) {
    fresh = fresh && (get_ts_elapsed(microsecond_timer_get(), gm_hold.ts[i]) <= 500000U);
  }
  return fresh;
}

static void gm_hold_rx(const CANPacket_t *msg) {
  const int addr = GET_ADDR(msg);
  const int len = GET_LEN(msg);
  int input = -1;
  bool valid = true;
  if (addr == 0x34A) {
    input = 0;
    valid = len == 5;
    const int left = (GET_BYTE(msg, 0) << 8) | GET_BYTE(msg, 1);
    const int right = (GET_BYTE(msg, 2) << 8) | GET_BYTE(msg, 3);
    gm_hold.low_speed = (left <= 173) && (right <= 173);  // <= 1.5 m/s
  } else if (addr == 0xC9) {
    input = 1;
    valid = len == 8;
  } else if (addr == 0x1C4) {
    input = 2;
    valid = len == 8;
    gm_hold.cruise_active = (GET_BYTE(msg, 1) >> 5) != 0U;
  } else if (addr == 0xBE) {
    input = 3;
    valid = (len >= 6) && (len <= 8);
    gm_hold.pedal = GET_BYTE(msg, 1);
  }
  // An expired intent cannot be revived merely by receiving fresh idle inputs.
  if (!gm_hold_inputs_fresh()) {
    gm_hold_reset();
  }
  if (input >= 0) {
    if (!valid) {
      gm_hold.seen = 0U;
      gm_hold_reset();
      return;
    }
    gm_hold.seen |= (uint8_t)(1U << input);
    gm_hold.ts[input] = microsecond_timer_get();
  }
  if (!gm_hold_inputs_fresh() || safety_rx_checks_invalid || controls_allowed || gas_pressed || regen_braking ||
      gm_hold.cruise_active || !gm_hold.low_speed || (gm_hold.active && vehicle_moving)) {
    gm_hold_reset();
  } else {
    // Pedal strength and its pressed switch arrive independently. Evaluate their
    // fresh combined state, as the host parser does, rather than only on 0xBE.
    if (brake_pressed && (gm_hold.pedal >= 80U)) {
      gm_hold.armed = true;
    } else if (!gm_hold.active && !brake_pressed) {
      gm_hold.armed = false;
    }
    if (!gm_hold.active && gm_hold.armed && brake_pressed && !vehicle_moving) {
      gm_hold.active = true;
    }
    // Python cancels on a light tap. Safety retains only stationary permission:
    // independently timed pedal/switch CAN frames must not interrupt an existing hold.
  }
}

static void gm_rx_hook(const CANPacket_t *to_push) {

  const int GM_STANDSTILL_THRSLD = 10;  // 0.311kph
  // panda interceptor threshold needs to be equivalent to openpilot threshold to avoid controls mismatches
  // If thresholds are mismatched then it is possible for panda to see the gas fall and rise while openpilot is in the pre-enabled state
  const int GM_GAS_INTERCEPTOR_THRESHOLD = 550; // (675 + 355) / 2 ratio between offset and gain from dbc file
  #define GM_GET_INTERCEPTOR(msg) (((GET_BYTE((msg), 0) << 8) + GET_BYTE((msg), 1) + (GET_BYTE((msg), 2) << 8) + GET_BYTE((msg), 3)) / 2U) // avg between 2 tracks



  if (GET_BUS(to_push) == 0U) {
    int addr = GET_ADDR(to_push);

    if (addr == 0x184) {
      int torque_driver_new = ((GET_BYTE(to_push, 6) & 0x7U) << 8) | GET_BYTE(to_push, 7);
      torque_driver_new = to_signed(torque_driver_new, 11);
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // sample rear wheel speeds
    if (addr == 0x34A) {
      int left_rear_speed = (GET_BYTE(to_push, 0) << 8) | GET_BYTE(to_push, 1);
      int right_rear_speed = (GET_BYTE(to_push, 2) << 8) | GET_BYTE(to_push, 3);
      vehicle_moving = (left_rear_speed > GM_STANDSTILL_THRSLD) || (right_rear_speed > GM_STANDSTILL_THRSLD);
    }

    // ACC steering wheel buttons (GM_CAM is tied to the PCM)
    if ((addr == 0x1E1) && (!gm_pcm_cruise || gm_cc_long)) {
      int button = (GET_BYTE(to_push, 5) & 0x70U) >> 4;

      // enter controls on falling edge of set or rising edge of resume (avoids fault)
      bool set = (button != GM_BTN_SET) && (cruise_button_prev == GM_BTN_SET);
      bool res = (button == GM_BTN_RESUME) && (cruise_button_prev != GM_BTN_RESUME);
      if (set || res) {
        controls_allowed = true;
      }

      // exit controls on cancel press
      if (button == GM_BTN_CANCEL) {
        controls_allowed = false;
      }

      cruise_button_prev = button;
    }

    // Reference for brake pressed signals:
    // https://github.com/commaai/openpilot/blob/master/selfdrive/car/gm/carstate.py
    if ((addr == 0xBE) && ((gm_hw == GM_ASCM) || (gm_ascm_int && !gm_ascm_int_c9_brake))) {
      brake_pressed = GET_BYTE(to_push, 1) >= 10U;
    }
    if (addr == 0xC9) {
      if ((gm_hw == GM_CAM) && (!gm_ascm_int || gm_ascm_int_c9_brake)) {
        brake_pressed = (GET_BYTE(to_push, 5) & 0x01U) != 0U;
      }
      acc_main_on = (GET_BYTE(to_push, 3) & 0x20U) != 0U;
    }

    if (addr == 0x1C4) {
      if (!enable_gas_interceptor) {
        gas_pressed = GET_BYTE(to_push, 5) != 0U;
      }

      // enter controls on rising edge of ACC, exit controls when ACC off
      if (gm_pcm_cruise && gm_has_acc) {
        bool cruise_engaged = (GET_BYTE(to_push, 1) >> 5) != 0U;
        pcm_cruise_check(cruise_engaged);
      }
    }

    // Cruise check for CC only cars
    if ((addr == 0x3D1) && !gm_has_acc) {
      bool cruise_engaged = (GET_BYTE(to_push, 4) >> 7) != 0U;
      if (gm_cc_long) {
        pcm_cruise_check(cruise_engaged);
      } else {
        cruise_engaged_prev = cruise_engaged;
      }
    }

    if (addr == 0xBD) {
      regen_braking = (GET_BYTE(to_push, 0) >> 4) != 0U;
      if (gm_bolt_acc_pedal && (GET_LEN(to_push) == 7)) {
        if (regen_braking) {
          // The driver is pulling the real paddle: give it back. Drop both feeds so a buffered release (allowed even
          // without controls) cannot cover the driver's regen after each stock frame (not in StarPilot).
          gm_spoof_reset(&gm_paddle_spoof);
          gm_spoof_reset(&gm_prndl2_spoof);
        } else {
          gm_spoof_send_after_stock(&gm_paddle_spoof, 0xBD, 7);
        }
      }
    }

    if (gm_bolt_acc_pedal && (addr == 0x1F5) && (GET_LEN(to_push) == 8) && !regen_braking) {
      gm_spoof_send_after_stock(&gm_prndl2_spoof, 0x1F5, 8);
    }

    // Pedal Interceptor
    if ((addr == 0x201) && enable_gas_interceptor) {
      int gas_interceptor = GM_GET_INTERCEPTOR(to_push);
      // The Bolt ACC with pedal takes StarPilot's 595 (~23.6 in carstate units); its carstate threshold 23 sits just below
      const int threshold = gm_bolt_acc_pedal ? 595 : GM_GAS_INTERCEPTOR_THRESHOLD;
      gas_pressed = gas_interceptor > threshold;
      gas_interceptor_prev = gas_interceptor;
      // gm_pcm_cruise = false;
    }

    bool stock_ecu_detected = (addr == 0x180);  // ASCMLKASteeringCmd

    // Check ASCMGasRegenCmd only if we're blocking it
    if (!gm_pcm_cruise && (addr == 0x2CB)) {
      stock_ecu_detected = true;
    }
    // The Bolt ACC with pedal sends its own friction brake on the powertrain bus and blocks the camera's
    if (gm_bolt_acc_pedal && (addr == 0x315)) {
      stock_ecu_detected = true;
    }
    generic_rx_checks(stock_ecu_detected);
    if (gm_manual_hold) {
      gm_hold_rx(to_push);
    }
  }
}

static bool gm_tx_hook(const CANPacket_t *to_send) {
  const TorqueSteeringLimits GM_STEERING_LIMITS = {
    .max_steer = 300,
    .max_rate_up = 20,
    .max_rate_down = 25,
    .driver_torque_allowance = 65,
    .driver_torque_multiplier = 4,
    .max_rt_delta = 128,
    .max_rt_interval = 250000,
    .type = TorqueDriverLimited,
  };

  bool tx = true;
  int addr = GET_ADDR(to_send);

  // BRAKE: safety check
  if (addr == 0x315) {
    int brake = ((GET_BYTE(to_send, 0) & 0xFU) << 8) + GET_BYTE(to_send, 1);
    brake = (0x1000 - brake) & 0xFFF;
    if (gm_manual_hold && !gm_hold_inputs_fresh()) {
      gm_hold_reset();
    }
    // Only driver-armed stationary friction hold, never acceleration or ACC permission.
    const bool manual_hold = gm_manual_hold && gm_hold.active && gm_hold_inputs_fresh() && !safety_rx_checks_invalid &&
                             !controls_allowed && !vehicle_moving && !gas_pressed && !regen_braking && !gm_hold.cruise_active &&
                             (brake > 0) && (brake <= 400) && ((GET_BYTE(to_send, 0) >> 4) == 0xAU);
    if (!manual_hold && longitudinal_brake_checks(brake, *gm_long_limits)) {
      tx = false;
    }
  }

  // LKA STEER: safety check
  if (addr == 0x180) {
    int desired_torque = ((GET_BYTE(to_send, 0) & 0x7U) << 8) + GET_BYTE(to_send, 1);
    desired_torque = to_signed(desired_torque, 11);

    bool steer_req = GET_BIT(to_send, 3U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, GM_STEERING_LIMITS)) {
      //tx = false;
    }
  }

  // GAS: safety check (interceptor)
  if (addr == 0x200) {
    if (longitudinal_interceptor_checks(to_send)) {
      tx = 0;
    }
  }

  // GAS/REGEN: safety check
  if (addr == 0x2CB) {
    bool apply = GET_BIT(to_send, 0U);
    // SDGM transmit requests must never restore permission after disengagement, nor may the Bolt ACC with pedal's or
    // the Malibu ASCM harness's (neither sends 0x2CB nor allows it).
    if (apply && !controls_allowed && !gm_sdgm && !gm_bolt_acc_pedal && !gm_ascm_int) {
      controls_allowed = true;        
    }
    // convert float CAN signal to an int for gas checks: 22534 / 0.125 = 180272
    int gas_regen = (((GET_BYTE(to_send, 1) & 0x7U) << 16) | (GET_BYTE(to_send, 2) << 8) | GET_BYTE(to_send, 3)) - 180272U;

    bool violation = false;
    // Allow apply bit in pre-enabled and overriding states
    violation |= !controls_allowed && apply;
    violation |= longitudinal_gas_checks(gas_regen, *gm_long_limits);

    if (violation) {
      tx = false;
    }
  }

  // BUTTONS: used for resume spamming and cruise cancellation with stock longitudinal
  if (addr == 0x1E1) {
    int button = (GET_BYTE(to_send, 5) >> 4) & 0x7U;
    bool allowed_btn = (button == GM_BTN_CANCEL) && cruise_engaged_prev;

    if (!gm_pcm_cruise) {
      allowed_btn |= (button == GM_BTN_SET || button == GM_BTN_RESUME || button == GM_BTN_UNPRESS);
    }
    if (gm_cc_long) {
      allowed_btn |= cruise_engaged_prev && (button == GM_BTN_SET || button == GM_BTN_RESUME || button == GM_BTN_UNPRESS);
    }
    // The Bolt ACC with pedal cancels the camera's own ACC at any time (toward the camera) and sends no other button.
    if (gm_bolt_acc_pedal) {
      allowed_btn = (button == GM_BTN_CANCEL);
    }

    if (!allowed_btn) {
      tx = false;
    }
  }

  // REGEN PADDLE / PRNDL2 (Bolt ACC with pedal): an apply needs controls; host frames are only buffered (see above)
  if (gm_bolt_acc_pedal && ((addr == 0xBD) || (addr == 0x1F5))) {
    const int len = (addr == 0xBD) ? 7 : 8;
    bool apply;
    if (addr == 0xBD) {
      apply = (GET_BYTE(to_send, 0) & 0xF0U) != 0U;
    } else {
      const uint8_t prndl2 = GET_BYTE(to_send, 3) & 0xFU;
      apply = (prndl2 == 7U) || (prndl2 == 5U);
    }
    if (!controls_allowed && apply) {
      tx = false;
    }
    if (!gm_spoof_internal_tx) {
      // The tx hook also sees frames outside the allowlist, so check the bus and length before buffering.
      if (tx && (GET_BUS(to_send) == 0U) && (GET_LEN(to_send) == len)) {
        gm_spoof_feed((addr == 0xBD) ? &gm_paddle_spoof : &gm_prndl2_spoof, to_send, len);
      }
      tx = false;
    }
  }

  // Malibu ASCM harness: openpilot's PSCMStatus copy toward the ASCM (see gm_fwd_hook)
  if (gm_ascm_int && tx && (addr == 0x184) && (GET_BUS(to_send) == 2U) && (GET_LEN(to_send) == 8)) {
    gm_ascm_int_pscm_tx_us = microsecond_timer_get();
    gm_ascm_int_pscm_tx_seen = true;
  }

  return tx;
}

static int gm_fwd_hook(CANPacket_t* to_send) {
  const int bus_num = GET_BUS(to_send);
  const int addr = GET_ADDR(to_send);

  int bus_fwd = -1;

  if (gm_hw == GM_CAM) {
    if (bus_num == 0) {
      // block PSCMStatus; forwarded through openpilot to hide an alert from the camera
      bool is_pscm_msg = (addr == 0x184);
      if (is_pscm_msg && gm_ascm_int) {
        // Malibu ASCM harness: the real one reaches the ASCM while openpilot's copy is missing
        is_pscm_msg = gm_ascm_int_pscm_relayed();
      }
      // Bolt ACC with pedal: the camera gets no ECM cruise status (StarPilot's non-ACC rule)
      bool is_ecm_cruise_status_msg = gm_bolt_acc_pedal && (addr == 0x3D1);
      if (!is_pscm_msg && !is_ecm_cruise_status_msg) {
        bus_fwd = 2;
      }
    }

    if (bus_num == 2) {
      // block lkas message and acc messages if gm_cam_long, forward all others
      bool is_lkas_msg = (addr == 0x180);
      bool is_acc_msg = (addr == 0x315) || (addr == 0x2CB) || (addr == 0x370);
      bool block_msg = is_lkas_msg || (is_acc_msg && gm_cam_long);
      // Bolt ACC with pedal: openpilot sends its own ACC status and friction brake; the camera's gas/regen still passes
      block_msg |= gm_bolt_acc_pedal && ((addr == 0x370) || (addr == 0x315));
      if (!block_msg) {
        bus_fwd = 0;
      }
    }
  }

  return bus_fwd;
}

static safety_config gm_init(uint16_t param) {
  const uint16_t GM_PARAM_HW_CAM = 1;
  const uint16_t GM_PARAM_CC_LONG = 4;
  const uint16_t GM_PARAM_NO_CAMERA = 8;
  const uint16_t GM_PARAM_HW_ASCM_LONG = 16;
  const uint16_t GM_PARAM_NO_ACC = 32;
  const uint16_t GM_PARAM_PEDAL_LONG = 64;  // TODO: this can be inferred
  const uint16_t GM_PARAM_EV = 256;
  const uint16_t GM_PARAM_HW_SDGM = 512;
  const uint16_t GM_PARAM_MANUAL_AUTO_HOLD = 1024;
  const uint16_t GM_PARAM_HW_ASCM_INT = 4096;
  const uint16_t GM_PARAM_ASCM_INT_C9_BRAKE = 8192;

  // common safety checks assume unscaled integer values
  static const int GM_GAS_TO_CAN = 8;  // 1 / 0.125

  static const LongitudinalLimits GM_ASCM_LONG_LIMITS = {
    .max_gas = 1018 * GM_GAS_TO_CAN,
    .min_gas = -650 * GM_GAS_TO_CAN,
    .inactive_gas = -650 * GM_GAS_TO_CAN,
    .max_brake = 400,
  };

  static const CanMsg GM_ASCM_TX_MSGS[] = {{0x180, 0, 4}, {0x409, 0, 7}, {0x40A, 0, 7}, {0x2CB, 0, 8}, {0x370, 0, 6}, {0x200, 0, 6}, {0x1E1, 0, 7}, {0xBD, 0, 7},// pt bus
                                           {0xA1, 1, 7}, {0x306, 1, 8}, {0x308, 1, 7}, {0x310, 1, 2},   // obs bus
                                           {0x315, 2, 5}};  // ch bus


  static const LongitudinalLimits GM_CAM_LONG_LIMITS = {
    .max_gas = 1346 * GM_GAS_TO_CAN,
    .min_gas = -540 * GM_GAS_TO_CAN,
    .inactive_gas = -500 * GM_GAS_TO_CAN,
    .max_brake = 400,
  };

  static const CanMsg GM_CAM_LONG_TX_MSGS[] = {{0x180, 0, 4}, {0x315, 0, 5}, {0x2CB, 0, 8}, {0x370, 0, 6}, {0x200, 0, 6}, {0x1E1, 0, 7},  // pt bus
                                               {0x184, 2, 8}};  // camera bus

  // Traverse SDGM/SASCM harness: brake only on camera CAN, never on powertrain CAN.
  static const CanMsg GM_SDGM_LONG_TX_MSGS[] = {{0x180, 0, 4}, {0x2CB, 0, 8}, {0x370, 0, 6}, {0x200, 0, 6}, {0x1E1, 0, 7},
                                               {0x315, 2, 5}, {0x184, 2, 8}};

  // TODO: do checksum and counter checks. Add correct timestep, 0.1s for now.
  static RxCheck gm_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    GM_ACC_RX_CHECKS
  };

  static RxCheck gm_ev_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    GM_ACC_RX_CHECKS
    {.msg = {{0xBD, 0, 7, .ignore_checksum = true, .ignore_counter = true, .frequency = 40U}, { 0 }, { 0 }}},
  };

  static RxCheck gm_no_acc_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    {.msg = {{0x3D1, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},  // Non-ACC PCM
  };

  static RxCheck gm_no_acc_ev_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    {.msg = {{0xBD, 0, 7, .ignore_checksum = true, .ignore_counter = true, .frequency = 40U}, { 0 }, { 0 }}},
    {.msg = {{0x3D1, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},  // Non-ACC PCM
  };

  // Malibu ASCM harness without ECMAcceleratorPos: the brake comes from 0xC9, so 0xF1 replaces 0xBE (StarPilot's
  // gm_ascm_int_stock_cam_rx_checks)
  static RxCheck gm_ascm_int_c9_brake_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    {.msg = {{0xF1, 0, 6, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},
  };

  static RxCheck gm_pedal_rx_checks[] = {
    GM_COMMON_RX_CHECKS
    {.msg = {{0xBD, 0, 7, .ignore_checksum = true, .ignore_counter = true, .frequency = 40U}, { 0 }, { 0 }}},
    {.msg = {{0x3D1, 0, 8, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},  // Non-ACC PCM
    {.msg = {{0x201, 0, 6, .ignore_checksum = true, .ignore_counter = true, .frequency = 10U}, { 0 }, { 0 }}},  // pedal
  };

  static const CanMsg GM_CAM_TX_MSGS[] = {{0x180, 0, 4}, {0x200, 0, 6}, {0x1E1, 0, 7},  // pt bus
                                          {0x1E1, 2, 7}, {0x184, 2, 8}};  // camera bus

  // StarPilot's GM_CAM_BOLT_2022_PEDAL_FRICTION_TX_MSGS without 0x3D1, which openpilot never sends for this car
  static const CanMsg GM_CAM_BOLT_ACC_PEDAL_TX_MSGS[] = {{0x180, 0, 4}, {0x200, 0, 6}, {0x1E1, 0, 7}, {0x370, 0, 6},
                                                         {0x315, 0, 5}, {0xBD, 0, 7}, {0x1F5, 0, 8},  // pt bus
                                                         {0x1E1, 2, 7}, {0x184, 2, 8}};  // camera bus


  // Malibu ASCM harness with the stock ACC: openpilot's LKAS, PSCMStatus toward the ASCM and CANCEL toward the ASCM
  static const CanMsg GM_ASCM_INT_TX_MSGS[] = {{0x180, 0, 4},  // pt bus
                                               {0x184, 2, 8}, {0x1E1, 2, 7}};  // ASCM side

  // Malibu ASCM harness with a SASCM and openpilot longitudinal (StarPilot ASCM_INT alpha long): gas/regen, the
  // friction brake (relayed to the EBCM by the SASCM), ACC status and buttons on the powertrain bus, PSCMStatus toward
  // the ASCM. The ASCM's own 0x2CB/0x315/0x370 are not forwarded (gm_fwd_hook, gm_cam_long).
  static const CanMsg GM_ASCM_INT_LONG_TX_MSGS[] = {{0x180, 0, 4}, {0x315, 0, 5}, {0x2CB, 0, 8}, {0x370, 0, 6}, {0x1E1, 0, 7},  // pt bus
                                                    {0x184, 2, 8}};  // ASCM side

  static const CanMsg GM_CC_LONG_TX_MSGS[] = {{0x180, 0, 4}, {0x1E1, 0, 7},  // pt bus
                                              {0x184, 2, 8}, {0x1E1, 2, 7}};  // camera bus
  gm_hw = GET_FLAG(param, GM_PARAM_HW_CAM) ? GM_CAM : GM_ASCM;
  gm_force_ascm = GET_FLAG(param, GM_PARAM_HW_ASCM_LONG);

  if ((gm_hw == GM_ASCM) || gm_force_ascm) {
    gm_long_limits = &GM_ASCM_LONG_LIMITS;
  } else if (gm_hw == GM_CAM) {
    gm_long_limits = &GM_CAM_LONG_LIMITS;
  } else {
  }

  gm_cc_long = GET_FLAG(param, GM_PARAM_CC_LONG);
  gm_sdgm = GET_FLAG(param, GM_PARAM_HW_SDGM);
  // Only the exact layouts the host builds for this car: HW_CAM + HW_ASCM_INT, optionally + the C9 brake and + camera
  // long (SASCM). Any other mode bit alongside them makes the panda ignore both new bits, so a mixed param keeps its
  // existing behavior.
  const uint16_t GM_PARAM_ASCM_INT_CAM_LONG = 2;  // GM_PARAM_HW_CAM_LONG
  const uint16_t GM_ASCM_INT_PARAMS = GM_PARAM_HW_CAM | GM_PARAM_HW_ASCM_INT | GM_PARAM_ASCM_INT_C9_BRAKE | GM_PARAM_ASCM_INT_CAM_LONG;
  gm_ascm_int = GET_FLAG(param, GM_PARAM_HW_ASCM_INT) && GET_FLAG(param, GM_PARAM_HW_CAM) &&
                ((param & (uint16_t)(~GM_ASCM_INT_PARAMS)) == 0U);
  gm_ascm_int_c9_brake = gm_ascm_int && GET_FLAG(param, GM_PARAM_ASCM_INT_C9_BRAKE);
  gm_ascm_int_pscm_tx_seen = false;
  gm_ascm_int_pscm_tx_us = 0U;
  if (gm_ascm_int) {
    // The ASCM's 0x2CB layout and the host's limits for this car are the ASCM ones (inactive -650 Nm, max 1018 Nm)
    gm_long_limits = &GM_ASCM_LONG_LIMITS;
  }
  gm_cam_long = false;
#ifdef ALLOW_DEBUG
  const uint16_t GM_PARAM_HW_CAM_LONG = 2;
  gm_cam_long = GET_FLAG(param, GM_PARAM_HW_CAM_LONG) && !gm_cc_long;
#endif
  gm_pedal_long = GET_FLAG(param, GM_PARAM_PEDAL_LONG);
  gm_pcm_cruise = (gm_hw == GM_CAM) && (!gm_cam_long || gm_cc_long) && !gm_force_ascm && !gm_pedal_long;
  gm_skip_relay_check = GET_FLAG(param, GM_PARAM_NO_CAMERA);
  gm_has_acc = !GET_FLAG(param, GM_PARAM_NO_ACC);

  const uint16_t GM_PARAM_PEDAL_INTERCEPTOR = 128;
  enable_gas_interceptor = GET_FLAG(param, GM_PARAM_PEDAL_INTERCEPTOR);
  gm_manual_hold = GET_FLAG(param, GM_PARAM_MANUAL_AUTO_HOLD) && (gm_hw == GM_CAM) && gm_cam_long && gm_sdgm &&
                   gm_has_acc && !gm_force_ascm && !gm_pedal_long && !enable_gas_interceptor && !GET_FLAG(param, GM_PARAM_EV);
  gm_hold_reset();
  gm_hold.low_speed = false;
  gm_hold.cruise_active = false;
  gm_hold.pedal = 0U;
  gm_hold.seen = 0U;
  for (int i = 0; i < 4; i++) {
    gm_hold.ts[i] = 0U;
  }
  const uint16_t GM_PARAM_BOLT_ACC_PEDAL = 2048;
  // The raw camera-long bit is checked too: release builds (no ALLOW_DEBUG) never set gm_cam_long from it.
  const uint16_t GM_PARAM_HW_CAM_LONG_BIT = 2;
  gm_bolt_acc_pedal = GET_FLAG(param, GM_PARAM_BOLT_ACC_PEDAL) && (gm_hw == GM_CAM) && gm_pedal_long && enable_gas_interceptor &&
                      !gm_has_acc && !gm_cc_long && !gm_cam_long && !GET_FLAG(param, GM_PARAM_HW_CAM_LONG_BIT) && !gm_sdgm &&
                      !gm_force_ascm;
  gm_spoof_reset(&gm_paddle_spoof);
  gm_spoof_reset(&gm_prndl2_spoof);
  gm_spoof_internal_tx = false;
  if (enable_gas_interceptor) {
      print("GM Pedal Interceptor Enabled\n");
  }
  else print("GM Pedal Interceptor Disabled\n");

  safety_config ret;
  if (gm_hw == GM_CAM) {
    if (gm_cc_long) {
      ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_CC_LONG_TX_MSGS);
      print("GM CC Long\n");
    } else if (gm_cam_long) {
      if (gm_ascm_int) {
        ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_ASCM_INT_LONG_TX_MSGS);
      } else if (gm_sdgm) {
        ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_SDGM_LONG_TX_MSGS);
      } else {
        ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_CAM_LONG_TX_MSGS);
      }
      print("GM CAM Long\n");
    } else if (gm_bolt_acc_pedal) {
      ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_CAM_BOLT_ACC_PEDAL_TX_MSGS);
      print("GM CAM Bolt ACC pedal\n");
    } else if (gm_ascm_int) {
      ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_ASCM_INT_TX_MSGS);
      print("GM CAM ASCM intercept\n");
    } else {
      ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_CAM_TX_MSGS);
      print("GM CAM\n");
    }
  } else {
    ret = BUILD_SAFETY_CFG(gm_rx_checks, GM_ASCM_TX_MSGS);
  }

  const bool gm_ev = GET_FLAG(param, GM_PARAM_EV);
  if (enable_gas_interceptor) {
    SET_RX_CHECKS(gm_pedal_rx_checks, ret);
  } else if (!gm_has_acc && gm_ev) {
    SET_RX_CHECKS(gm_no_acc_ev_rx_checks, ret);
  } else if (!gm_has_acc && !gm_ev) {
    SET_RX_CHECKS(gm_no_acc_rx_checks, ret);
  } else if (gm_ev) {
    SET_RX_CHECKS(gm_ev_rx_checks, ret);
  } else if (gm_ascm_int_c9_brake) {
    SET_RX_CHECKS(gm_ascm_int_c9_brake_rx_checks, ret);
  } else {}

  return ret;
}

const safety_hooks gm_hooks = {
  .init = gm_init,
  .rx = gm_rx_hook,
  .tx = gm_tx_hook,
  .fwd = gm_fwd_hook,
};
