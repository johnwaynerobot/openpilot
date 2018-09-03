import struct

import common.numpy_fast as np
from selfdrive.config import Conversions as CV
#from selfdrive.car.honda.values import CAR, HONDA_BOSCH
from selfdrive.car.kia_soul.values import CAR, HONDA_BOSCH #2018.09.01 DV add kia soul

# *** Honda specific ***
def can_cksum(mm):
  s = 0
  for c in mm:
    c = ord(c)
    s += (c>>4)
    s += c & 0xF
  s = 8-s
  s %= 0x10
  return s


def fix(msg, addr):
  msg2 = msg[0:-1] + chr(ord(msg[-1]) | can_cksum(struct.pack("I", addr)+msg))
  return msg2


def make_can_msg(addr, dat, idx, alt):
  if idx is not None:
    dat += chr(idx << 4)
    dat = fix(dat, addr)
  return [addr, 0, dat, alt]

#2018.09.01 this one we should add fingerprint CAR.SOUL ,
#apply brake
def create_brake_command(packer, apply_brake, pcm_override, pcm_cancel_cmd, chime, fcw, idx):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
  pump_on = apply_brake > 0
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  values = {
    # "COMPUTER_BRAKE": apply_brake,   # computer
    # "BRAKE_PUMP_REQUEST": pump_on,
    "CRUISE_OVERRIDE": pcm_override,
    "CRUISE_FAULT_CMD": pcm_fault_cmd,
    "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    # "COMPUTER_BRAKE_REQUEST": brake_rq,
    "SET_ME_0X80": 0x80,
    "BRAKE_LIGHTS": brakelights,
    "CHIME": chime,
    "FCW": fcw << 1,  # TODO: Why are there two bits for fcw? According to dbc file the first bit should also work
  }
  return packer.make_can_msg("BRAKE_COMMAND", 0, values, idx)

def create_brake_command_soul(packer, apply_brake, pcm_override, pcm_cancel_cmd, chime, fcw, idx):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
  pump_on = apply_brake > 0
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False
  if brake_rq==1:
    x=0xCC05

  values = {
    "BRAKE_COMMAND_pedal_command": apply_brake,   # computer
    "BRAKE_COMMAND_magic": x ,
    # "CRUISE_OVERRIDE": pcm_override,
    # "CRUISE_FAULT_CMD": pcm_fault_cmd,
  #   "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
  #   "COMPUTER_BRAKE_REQUEST": brake_rq,
    # "SET_ME_0X80": 0x80,
  #   "BRAKE_LIGHTS": brakelights,
  #   "CHIME": chime,
  #   "FCW": fcw << 1,  # TODO: Why are there two bits for fcw? According to dbc file the first bit should also work
  }
  return packer.make_can_msg("SOUL_BRAKE_COMMAND", 0, values, idx)

def create_brake_enable_soul(packer, apply_brake, pcm_override, pcm_cancel_cmd, chime, fcw, idx):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
  pump_on = apply_brake > 0
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  if brake_rq==1:
    x=0xCC05

  values = {
    # "BRAKE_COMMAND_pedal_command": apply_brake,   # computer
    # "BRAKE_COMMAND_magic": pump_on,
    # "CRUISE_OVERRIDE": pcm_override,
    # "CRUISE_FAULT_CMD": pcm_fault_cmd,
  #   "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "BRAKE_ENABLE_magic": x,
  #   "SET_ME_0X80": 0x80,
  #   "BRAKE_LIGHTS": brakelights,
  #   "CHIME": chime,
  #   "FCW": fcw << 1,  # TODO: Why are there two bits for fcw? According to dbc file the first bit should also work
  }
  return packer.make_can_msg("SOUL_BRAKE_ENABLE", 0, values, idx)

def create_brake_disable_soul(packer, apply_brake, pcm_override, pcm_cancel_cmd, chime, fcw, idx):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
  pump_on = apply_brake > 0
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  if brake_rq==0:
    x=0xCC05

  values = {
    # "BRAKE_COMMAND_pedal_command": apply_brake,   # computer
    # "BRAKE_COMMAND_magic": pump_on,
    # "CRUISE_OVERRIDE": pcm_override,
    # "CRUISE_FAULT_CMD": pcm_fault_cmd,
  #   "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "BRAKE_DISABLE_magic": x,
  #   "SET_ME_0X80": 0x80,
  #   "BRAKE_LIGHTS": brakelights,
  #   "CHIME": chime,
  #   "FCW": fcw << 1,  # TODO: Why are there two bits for fcw? According to dbc file the first bit should also work
  }
  return packer.make_can_msg("SOUL_BRAKE_DISABLE", 0, values, idx)


def create_gas_command(packer, gas_amount, idx):
  """Creates a CAN message for the Honda DBC GAS_COMMAND."""
  enable = gas_amount > 0.001

  values = {} #initializing the value dict empty initially
  if enable ==1:
    x_gas=0xCC05 

  if enable:
    values["THROTTLE_COMMAND_magic"] = x_gas
    values["THROTTLE_COMMAND_pedal_command"] = gas_amount 

  return packer.make_can_msg("THROTTLE_COMMAND", 0, values, idx)

def create_gas_command_enable(packer, gas_amount, idx):
  """Creates a CAN message for the Honda DBC GAS_COMMAND."""
  enable = gas_amount > 0.001

  values = {} #initializing the value dict empty initially
  if enable ==1:
    x_gas_enable=0xCC05 

  if enable:
    values["THROTTLE_ENABLE_magic"] = x_gas_enable

  return packer.make_can_msg("THROTTLE_ENABLE", 0, values, idx)

def create_gas_command_disable(packer, gas_amount, idx):
  """Creates a CAN message for the Honda DBC GAS_COMMAND."""
  disable = gas_amount < 0.001

  values = {} #initializing the value dict empty initially
  if disable == 1:
    x_gas_disable=0xCC05 

  if disable:
    values["THROTTLE_DISABLE_magic"] = x_gas_disable

  return packer.make_can_msg("THROTTLE_DISABLE", 0, values, idx)


def create_steering_control(packer, apply_steer, lkas_active, car_fingerprint, idx):
  """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
  if lkas_active==1:
    x_steering_enable=0xCC05
  values = {
    "STEERING_COMMAND_magic": apply_steer if lkas_active else 0,
    "STEERING_COMMAND_pedal_command": x_steering_enable,
  }
  # Set bus 2 for accord and new crv.
  bus = 2 if car_fingerprint in HONDA_BOSCH else 0
  return packer.make_can_msg("STEERING_COMMAND", bus, values, idx)

def create_steering_control_enable(packer, apply_steer, lkas_active, car_fingerprint, idx):
  """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
  if lkas_active==1:
    x_steering_enable=0xCC05

  values = {
    "STEERING_ENABLE_magic": x_steering_enable
    }
  # Set bus 2 for accord and new crv.
  bus = 2 if car_fingerprint in HONDA_BOSCH else 0
  return packer.make_can_msg("STEERING_ENABLE", bus, values, idx)

def create_steering_control_disable(packer, apply_steer, lkas_active, car_fingerprint, idx):
  """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
  if lkas_active==0:
    x_steering_disable=0xCC05
  values = {
    "STEERING_DISABLE_magic": x_steering_disable
  }
  # Set bus 2 for accord and new crv.
  bus = 2 if car_fingerprint in HONDA_BOSCH else 0
  return packer.make_can_msg("STEERING_DISABLE", bus, values, idx)



def create_ui_commands(packer, pcm_speed, hud, car_fingerprint, idx):
  """Creates an iterable of CAN messages for the UIs."""
  commands = []
  bus = 0

  # Bosch sends commands to bus 2.
  if car_fingerprint in HONDA_BOSCH:
    bus = 2
  else:
    acc_hud_values = {
      'PCM_SPEED': pcm_speed * CV.MS_TO_KPH,
      'PCM_GAS': hud.pcm_accel,
      'CRUISE_SPEED': hud.v_cruise,
      'ENABLE_MINI_CAR': hud.mini_car,
      'HUD_LEAD': hud.car,
      'SET_ME_X03': 0x03,
      'SET_ME_X03_2': 0x03,
      'SET_ME_X01': 0x01,
    }
    commands.append(packer.make_can_msg("ACC_HUD", 0, acc_hud_values, idx))

  lkas_hud_values = {
    'SET_ME_X41': 0x41,
    'SET_ME_X48': 0x48,
    'STEERING_REQUIRED': hud.steer_required,
    'SOLID_LANES': hud.lanes,
    'BEEP': hud.beep,
  }
  commands.append(packer.make_can_msg('LKAS_HUD', bus, lkas_hud_values, idx))

  if car_fingerprint in (CAR.CIVIC, CAR.ODYSSEY):
    commands.append(packer.make_can_msg('HIGHBEAM_CONTROL', 0, {'HIGHBEAMS_ON': False}, idx))

    radar_hud_values = {
      'ACC_ALERTS': hud.acc_alert,
      'LEAD_SPEED': 0x1fe,  # What are these magic values
      'LEAD_STATE': 0x7,
      'LEAD_DISTANCE': 0x1e,
    }
    commands.append(packer.make_can_msg('RADAR_HUD', 0, radar_hud_values, idx))
  return commands


def create_radar_commands(v_ego, car_fingerprint, new_radar_config, idx):
  """Creates an iterable of CAN messages for the radar system."""
  commands = []
  v_ego_kph = np.clip(int(round(v_ego * CV.MS_TO_KPH)), 0, 255)
  speed = struct.pack('!B', v_ego_kph)

  msg_0x300 = ("\xf9" + speed + "\x8a\xd0" +
               ("\x20" if idx == 0 or idx == 3 else "\x00") +
               "\x00\x00")

  if car_fingerprint == CAR.CIVIC:
    msg_0x301 = "\x02\x38\x44\x32\x4f\x00\x00"
    idx_offset = 0xc if new_radar_config else 0x8   # radar in civic 2018 requires 0xc
    commands.append(make_can_msg(0x300, msg_0x300, idx + idx_offset, 1))
  else:
    if car_fingerprint == CAR.CRV:
      msg_0x301 = "\x00\x00\x50\x02\x51\x00\x00"
    elif car_fingerprint == CAR.ACURA_RDX:
      msg_0x301 = "\x0f\x57\x4f\x02\x5a\x00\x00"
    elif car_fingerprint == CAR.ODYSSEY:
      msg_0x301 = "\x00\x00\x56\x02\x55\x00\x00"
    elif car_fingerprint == CAR.ACURA_ILX:
      msg_0x301 = "\x0f\x18\x51\x02\x5a\x00\x00"
    elif car_fingerprint == CAR.PILOT:
      msg_0x301 = "\x00\x00\x56\x02\x58\x00\x00"
    elif car_fingerprint == CAR.PILOT_2019:
      msg_0x301 = "\x00\x00\x58\x02\x5c\x00\x00"
    elif car_fingerprint == CAR.RIDGELINE:
      msg_0x301 = "\x00\x00\x56\x02\x57\x00\x00"
    elif car_fingerprint == CAR.SOUL: #added for kia_soul radar commands
      msg_0x301 = "\x00\x00\x50\x02\x51\x00\x00"
    commands.append(make_can_msg(0x300, msg_0x300, idx, 1))

  commands.append(make_can_msg(0x301, msg_0x301, idx, 1))
  return commands

def spam_buttons_command(packer, button_val, idx):
  values = {
    'CRUISE_BUTTONS': button_val,
    'CRUISE_SETTING': 0,
  }
  return packer.make_can_msg("SCM_BUTTONS", 0, values, idx)
