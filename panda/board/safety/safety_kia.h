const int gas_interceptor_threshold = 328;
int gas_interceptor_detected = 1;
int brake_prev = 0;
int gas_prev = 0;
int gas_interceptor_prev = 1;
int ego_speed = 0;

void kia_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {}

int kia_ign_hook() {
  return -1; // use GPIO to determine ignition
}

// FIXME
// *** all output safety mode ***

static void kia_init(int16_t param) {
  controls_allowed = 1;
}

static int kia_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  return true;
}


const safety_hooks kia_hooks = {
  .init = kia_init,
  .rx = kia_rx_hook,
  .tx = kia_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = kia_ign_hook,
  .fwd = nooutput_fwd_hook,
};
