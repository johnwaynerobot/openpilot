import struct

import common.numpy_fast as np
from selfdrive.config import Conversions as CV
#2018.09.14 add vehicle state to see radar, change message one with out idx
from selfdrive.car.kia.values import CAR, VEHICLE_STATE_MSG
 #2018.09.03 DV modified remove specific car

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

#2018.09.12 this one without alive and checksum
#def make_can_msg(addr, dat, alt):
  #return [addr, 0, dat, alt]

#2018.09.12 12:01PM EST this is for Honda message with alive and checksum
def make_can_msg(addr, dat, idx, alt):
  if idx is not None:
    dat += chr(idx << 4)
    dat = fix(dat, addr)
  return [addr, 0, dat, alt]

#2018.09.28 att one for link with packer
def make_can_msg2(addr, dat, idx, alt):
  if idx is not None:
    dat += chr(idx << 4)
    dat = fix(dat, addr)
  return [addr, 0, dat, alt]

### HONDA Style Begin
#2018.09.29 9:00AMEST change this to match honda this one we should add fingerprint CAR.SOUL ,
def create_brake_command(packer, apply_brake, pcm_override, pcm_cancel_cmd, chime, fcw, idx):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
  pump_on = apply_brake > 0
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  values = {
    "COMPUTER_BRAKE": apply_brake,
    "BRAKE_PUMP_REQUEST": pump_on,
    "CRUISE_OVERRIDE": pcm_override,
    "CRUISE_FAULT_CMD": pcm_fault_cmd,
    "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "COMPUTER_BRAKE_REQUEST": brake_rq,
    "SET_ME_0X80": 0x80,
    "BRAKE_LIGHTS": brakelights,
    "CHIME": chime,
    "FCW": fcw << 1,  # TODO: Why are there two bits for fcw? According to dbc file the first bit should also work
  }
  return packer.make_can_msg("BRAKE_COMMAND", 0, values, idx)


def create_gas_command(packer, gas_amount, idx):
  """Creates a CAN message for the Honda DBC GAS_COMMAND."""
  enable = gas_amount > 0.001

  values = {"ENABLE": enable}

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  return packer.make_can_msg("GAS_COMMAND", 0, values, idx)

def create_steering_control(packer, apply_steer, lkas_active, idx):
  """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
  values = {
    "STEER_TORQUE": apply_steer if lkas_active else 0,
    "STEER_TORQUE_REQUEST": lkas_active,
  }
  # Set bus 2 for accord and new crv.
  #bus = 2 if car_fingerprint in HONDA_BOSCH else 0
  bus = 0
  return packer.make_can_msg("STEERING_CONTROL", bus, values, idx)


###---- Honda Style End

## begin toyota style send angle
def create_ipas_steer_command(packer, apply_angle, enabled):
  """Creates a CAN message for the Toyota Steer Command."""
  if apply_angle < 0:
    direction = 3
  elif apply_angle > 0:
    direction = 1
  else:
    direction = 2

  mode = 3 if enabled else 1

  values = {
    "STATE": mode,
    "DIRECTION_CMD": direction,
    "ANGLE": apply_angle,
    "SET_ME_X10": 0x10,
    "SET_ME_X40": 0x40
  }
  #if apgs_enabled:
  #  return packer.make_can_msg("STEERING_IPAS", 0, values)
  #else:
  return packer.make_can_msg("STEERING_IPAS_COMMA", 0, values)
# end toyota style send angle

# def create_brake_command_soul(packer, apply_brake):
#   """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
#   brake_rq = apply_brake > 0
#   if brake_rq == True:
#     x = 0xCC05
#   else:
#     x = 0x0000
#   values = {}  # initializing the value dict empty initially
#   print("kiacan.py brake command soul apply brake")
#   print(apply_brake)
#   ## 2018.09.28 12:23PMEST
#   apply_brake_can_bytes = list(bytearray(struct.pack("=f", apply_brake)))    #converting float apply gas to byte list
#   print("kiacan.py output brake bytes array")
#   print(bytearray(struct.pack("=f", apply_brake)))
#   print("kiacan.py output apply_brake_can_bytes")
#   print(apply_brake_can_bytes)
#
#   values["BRAKE_COMMAND_magic"] = x
#   values["BRAKE_COMMAND_pedal_command"] = apply_brake * 1000000  #2018.09.28 6:49PMEST expecting float
#
#   print("kiacan.py brake command magic and brake pedal command")
#   print(values)
#   return packer.make_can_msg("SOUL_BRAKE_COMMAND", 0, values)  #remove idx no need for alive counter and checksum

def create_brake_enable_soul(packer, apply_brake):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""

  brake_rq = apply_brake > 0

  if brake_rq == True:
    x = 0xCC05
  else:
    x = 0x0000

  values = {
    "BRAKE_ENABLE_magic": x,
  }
  print("kiacan.py soul_brake_enable")
  print(values)
  return packer.make_can_msg("SOUL_BRAKE_ENABLE", 0, values) #remove idx no need for alive counter and checksum

def create_brake_disable_soul(packer, apply_brake):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
  brake_rq = apply_brake > 0

  if brake_rq == False:
    x = 0xCC05
  else:
    x = 0x0000

  values = {
    "BRAKE_DISABLE_magic": x,
  }

  print("kiacan.py brake disable soul")
  print(values)
  return packer.make_can_msg("SOUL_BRAKE_DISABLE", 0, values) #remove idx no need for alive counter and checksum


# def create_gas_command_soul(packer, gas_amount):
#   """Creates a CAN message for the Honda DBC GAS_COMMAND."""
#   enable = gas_amount > 0.001
#
#   print("kiacan.py gas amount")
#   print(gas_amount)
#   ## 2018.09.28 12:22PMEST
#   apply_gas_can_bytes = list(bytearray(struct.pack("=f", gas_amount)))    #converting float apply gas to byte list
#   print("kiacan.py output gas bytes array")
#   print(bytearray(struct.pack("=f", gas_amount)))
#   print("kiacan.py output apply_gas_can_bytes")
#   print(apply_gas_can_bytes)
#   if enable == True:
#     x_gas = 0xCC05
#   else:
#     x_gas = 0x0000
#   values = {} #initializing the value dict empty initially
#   if enable:
#     values["THROTTLE_COMMAND_magic"] = x_gas
#     values["THROTTLE_COMMAND_pedal_command"] = gas_amount * 1000000  #2018.09.28 1:53PMEST expecting float
#
#     print("kiacan.py Throttle command")
#     print(values)
#
#   return packer.make_can_msg("THROTTLE_COMMAND", 0, values) #remove idx no need for alive counter and checksum

def create_gas_command_enable(packer, gas_amount):
  """Creates a CAN message for the Honda DBC GAS_COMMAND."""
  enable = gas_amount > 0.001

  values = {} #initializing the value dict empty initially
  if enable == True:
    x_gas_enable = 0xCC05
  else:
    x_gas_enable = 0x0000

  print("kiacan.py x_gas_enable")
  print(x_gas_enable)
  if enable:
    values["THROTTLE_ENABLE_magic"] = x_gas_enable

  print("kiacan.py gas command enable")
  print(values)
  return packer.make_can_msg("THROTTLE_ENABLE", 0, values) #remove idx no need for alive counter and checksum

def create_gas_command_disable(packer, gas_amount):
  """Creates a CAN message for the Honda DBC GAS_COMMAND."""
  disable = gas_amount < 0.001

  values = {} #initializing the value dict empty initially
  if disable == True:
    x_gas_disable = 0xCC05
  else:
    x_gas_disable = 0x0000

  if disable:
    values["THROTTLE_DISABLE_magic"] = x_gas_disable
  print("kiacan.py gas command disable")
  print(values)
  return packer.make_can_msg("THROTTLE_DISABLE", 0, values) #remove idx no need for alive counter and checksum


# def create_steering_control_soul(packer, apply_steer, lkas_active):
#   """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
#
#   if lkas_active == True:
#     x_steering_enable = 0xCC05
#   else:
#     x_steering_enable = 0x0000
#   print("kiacan.py apply steer")
#   print(apply_steer)
#   print("kiacan.py lkas_active")
#   print(lkas_active)
#   ## 2018.09.28 12:25PMEST
#   apply_steer_can_bytes = list(bytearray(struct.pack("=f", apply_steer)))    #converting float apply gas to byte list
#   print("kiacan.py output apply_steer bytes array")
#   print(bytearray(struct.pack("=f", apply_steer)))
#   print("kiacan.py output apply_steer_can_bytes")
#   print(apply_steer_can_bytes)
#
#   values = {}  # initializing the value dict empty initially
#   if lkas_active:
#     values["STEERING_COMMAND_magic"] = x_steering_enable
#     values["STEERING_COMMAND_pedal_command"] = apply_steer   #2018.09.28 expecting float
#
#   print("kiacan.py Steering command pedal command")
#   print(values)
#
#   return packer.make_can_msg("STEERING_COMMAND", 0 , values) #remove idx no need for alive counter and checksum

def create_steering_control_enable(packer, lkas_active):
  """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
  #print("lkas_active kiacan.py")
  #print(lkas_active)

  if lkas_active == True:
    x_steering_control_enable = 0xCC05
  else:
    x_steering_control_enable = 0x0000

  print("kiacan.py x_steering_control_enable")
  print(x_steering_control_enable)

  values= {
    "STEERING_ENABLE_magic": x_steering_control_enable
    }
  print("kiacan.py steering control enable")
  print(values)
  return packer.make_can_msg("STEERING_ENABLE", 0, values) #remove idx no need for alive counter and checksum

def create_steering_control_disable(packer, lkas_active):
  """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
  if lkas_active == False:
    x_steering_disable = 0xCC05
  else:
    x_steering_disable = 0x0000
  values = {
    "STEERING_DISABLE_magic": x_steering_disable
  }

  print("kiacan.py steering control disable")
  print(values)
  return packer.make_can_msg("STEERING_DISABLE", 0, values) #remove idx no need for alive counter and checksum



def create_ui_commands(packer, pcm_speed, hud, idx):
  """Creates an iterable of CAN messages for the UIs."""
  commands = []

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
  commands.append(packer.make_can_msg('LKAS_HUD', 0, lkas_hud_values, idx))

 #2018.09.12 1:09PM use in honda for auto control highbeam
  #radar_hud_values = {
 #     'ACC_ALERTS': hud.acc_alert,
  #    'LEAD_SPEED': 0x1fe,  # What are these magic values
  #    'LEAD_STATE': 0x7,
  #    'LEAD_DISTANCE': 0x1e,
 # }
 # commands.append(packer.make_can_msg('RADAR_HUD', 0, radar_hud_values, idx)) #2018.09.03 change to bus 0
  return commands

#2018.09.14 10:53AM add comment, this message for radar on bus  1,
def create_radar_commands(v_ego, car_fingerprint, new_radar_config, idx):
  """Creates an iterable of CAN messages for the radar system."""
  commands = []
  v_ego_kph = np.clip(int(round(v_ego * CV.MS_TO_KPH)), 0, 255)
  speed = struct.pack('!B', v_ego_kph)

  msg_0x300 = ("\xf9" + speed + "\x8a\xd0" +
               ("\x20" if idx == 0 or idx == 3 else "\x00") +
               "\x00\x00")
  msg_0x301 = VEHICLE_STATE_MSG[car_fingerprint]

  #idx_0x300 = idx
  #if car_fingerprint == CAR.DUMMY:
    #idx_offset = 0xc if new_radar_config else 0x8   # radar in civic 2018 requires 0xc
   # idx_0x300 += idx_offset
  #2018.09.17 offset causing radar error

  commands.append(make_can_msg(0x300, msg_0x300, idx, 1))
  commands.append(make_can_msg(0x301, msg_0x301, idx, 1))
  return commands


def spam_buttons_command(packer, button_val, idx):
  values = {
    'CRUISE_BUTTONS': button_val,
    'CRUISE_SETTING': 0,
  }
  return packer.make_can_msg("SCM_BUTTONS", 0, values, idx)
