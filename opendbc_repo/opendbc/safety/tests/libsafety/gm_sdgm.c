// Keep CANPacket allocation and forwarding inside C to use the firmware ABI.
#define SAFETY_TEST_RECORDS_CAN_SEND
#include "safety.c"

// Frames a safety hook transmits itself, as the CAN driver would: through the tx hook unless skipped. Accepted frames
// are kept in order for the test to pop; rejected ones are only counted.
#define GM_TEST_SENT_MAX 64
static CANPacket_t gm_test_sent[GM_TEST_SENT_MAX];
static int gm_test_sent_len = 0;
static int gm_test_sent_rejected = 0;

void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook) {
  if (skip_tx_hook || safety_tx_hook(to_push)) {
    if (gm_test_sent_len < GM_TEST_SENT_MAX) {
      gm_test_sent[gm_test_sent_len] = *to_push;
      gm_test_sent[gm_test_sent_len].bus = bus_number;
      gm_test_sent_len++;
    }
  } else {
    gm_test_sent_rejected++;
  }
}

void gm_test_sent_clear(void) {
  gm_test_sent_len = 0;
  gm_test_sent_rejected = 0;
}

int gm_test_sent_rejected_count(void) {
  return gm_test_sent_rejected;
}

// Copies frame `index` into the outputs and returns its length, or -1 when there is none.
int gm_test_sent_frame(int index, unsigned int *address, unsigned int *bus, unsigned char *data) {
  if ((index < 0) || (index >= gm_test_sent_len)) return -1;
  const CANPacket_t *frame = &gm_test_sent[index];
  *address = frame->addr;
  *bus = frame->bus;
  const int len = GET_LEN(frame);
  for (int i = 0; i < len; i++) data[i] = frame->data[i];
  return len;
}

// Normally owned by the CAN driver; no hardware driver runs in this harness.
bool safety_tx_buffered_for_fwd = false;

int gm_test_frame(int operation, unsigned int address, unsigned int bus, const unsigned char *data, unsigned int length) {
  if ((bus > 3U) || (length > 8U)) return -2;
  CANPacket_t packet = {0};
  packet.addr = address;
  packet.bus = bus;
  packet.data_len_code = length;
  for (unsigned int i = 0; i < length; i++) packet.data[i] = data[i];
  can_set_checksum(&packet);
  if (operation == 0) return safety_rx_hook(&packet);
  if (operation == 1) return safety_tx_hook(&packet);
  return safety_fwd_hook(&packet);
}
