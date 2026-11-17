#include <stdbool.h>

#include "fake_stm.h"
#include "can.h"

//int safety_tx_hook(CANPacket_t *to_send) { return 1; }

#include "faults.h"
#include "safety.h"
#include "drivers/can_common.h"

#ifndef SAFETY_TEST_RECORDS_CAN_SEND
// The CAN driver is not part of this harness. Safety modes that transmit from a hook (the GM Bolt ACC with pedal
// regen paddle) get a no-op here; gm_sdgm.c defines a recording one.
void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook) {
  (void)to_push;
  (void)bus_number;
  (void)skip_tx_hook;
}
#endif

// libsafety stuff
#include "safety_helpers.h"
