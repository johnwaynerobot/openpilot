from cereal import car
from collections import namedtuple
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.controls.lib.drive_helpers import rate_limit
from common.numpy_fast import clip, interp   #2018.09.26 add interp
from selfdrive.car import apply_std_steer_torque_limits #same as Hyundai change
from selfdrive.car.kia.values import AH, CruiseButtons, CAR, DBC  #2018.09.02 DV add for Kia soul
from selfdrive.can.packer import CANPacker
from selfdrive.car.kia import kiacan
import numpy as np
from scipy import interpolate

#2018.09.04 import from Hyundai sanfe 2019, #TODO will need to use
#define in steering for reference
#No use yet



class SteerLimitParams():
  def __init__(self, car_fingerprint):

    if car_fingerprint in (CAR.SOUL, CAR.SOUL1, CAR.SOUL2):
      self.STEER_MAX = 5  # 409 is the max, change 250 to 5 degrees
      self.STEER_DELTA_UP = 3  # torque increase per refresh
      self.STEER_DELTA_DOWN = 7   # torque decrease per refresh
      self.STEER_DRIVER_ALLOWANCE = 50  #allowed driver torque before start limiting
      self.STEER_DRIVER_MULTIPLIER = 2   # weight driver torque heavily
      self.STEER_DRIVER_FACTOR = 1   # from dbc not use

    else:
      self.STEER_MAX = 5  # 409 is the max, change 250 to 5 degrees
      self.STEER_DELTA_UP = 3  # torque increase per refresh
      self.STEER_DELTA_DOWN = 7  # torque decrease per refresh
      self.STEER_DRIVER_ALLOWANCE = 50  # allowed driver torque before start limiting
      self.STEER_DRIVER_MULTIPLIER = 2  # weight driver torque heavily
      self.STEER_DRIVER_FACTOR = 1  # from dbc not use

def actuator_hystereses(brake, braking, brake_steady, v_ego, car_fingerprint):
  # hyst params... TODO: move these to VehicleParams
  brake_hyst_on = 0.02     # to activate brakes exceed this value
  brake_hyst_off = 0.005                     # to deactivate brakes below this value
  brake_hyst_gap = 0.01                      # don't change brake command for small ocilalitons within this value

  #*** histeresis logic to avoid brake blinking. go above 0.1 to trigger
  if (brake < brake_hyst_on and not braking) or brake < brake_hyst_off:
    brake = 0.
  braking = brake > 0.

  # for small brake oscillations within brake_hyst_gap, don't change the brake command
  if brake == 0.:
    brake_steady = 0.
  elif brake > brake_steady + brake_hyst_gap:
    brake_steady = brake - brake_hyst_gap
  elif brake < brake_steady - brake_hyst_gap:
    brake_steady = brake + brake_hyst_gap
  brake = brake_steady

  if (car_fingerprint in (CAR.SOUL)) and brake > 0.0:  ##2018.09.04 TODO: need to tune for kia soul
    brake += 0.15
  
  #2018.09.02 TODO: need to tune for kia soul
  elif (car_fingerprint in (CAR.SOUL1, CAR.SOUL2)) and brake > 0.0:
    brake += 0.15
  
  return brake, braking, brake_steady


def process_hud_alert(hud_alert):
  # initialize to no alert
  fcw_display = 0
  steer_required = 0
  acc_alert = 0
  if hud_alert == AH.NONE:          # no alert
    pass
  elif hud_alert == AH.FCW:         # FCW
    fcw_display = hud_alert[1]
  elif hud_alert == AH.STEER:       # STEER
    steer_required = hud_alert[1]
  else:                             # any other ACC alert
    acc_alert = hud_alert[1]

  return fcw_display, steer_required, acc_alert


HUDData = namedtuple("HUDData",
                     ["pcm_accel", "v_cruise", "mini_car", "car", "X4",
                      "lanes", "beep", "chime", "fcw", "acc_alert", "steer_required"])


class CarController(object):
  def __init__(self, dbc_name, car_fingerprint,  enable_camera=True):
    self.braking = False
    self.brake_steady = 0.
    self.brake_last = 0.
    self.enable_camera = enable_camera
    self.packer = CANPacker(dbc_name)
    self.new_radar_config = False
    self.car_fingerprint = car_fingerprint     #2018.09.06 12:06AM borrow from subaru carcontroller.py
    self.last_angle = 0   #2018.09.25 borrow from toyota
    self.angle_control = False   #2018.09.25 borrow from toyota
   # self.steer_angle_enabled = False   #2018.09.25 borrow from toyota

    # 2018.09.06 12:09AM borrow from subaru carcontroller.py
    # Setup detection helper. Routes commands to
    # an appropriate CAN bus number.
    #self.canbus = canbus

    self.params = SteerLimitParams(car_fingerprint)  #2018.09.05 define steering paramater limt #TODO to use in code


    #use to pack the message to can bus 2018.09.06 12:24AM
    #self.packer = CANPacker(DBC[car_fingerprint]['pt'])  #2018.09.05 add this from subaru with changes, not sure will error
    #self.packer = CANPacker('kia_soul_2016_ccan.dbc')  # 2018.09.06 2:06PMEST comment




  def update(self, sendcan, enabled, CS, frame, actuators, \
             pcm_speed, pcm_override, pcm_cancel_cmd, pcm_accel, \
             radar_error, hud_v_cruise, hud_show_lanes, hud_show_car, \
             hud_alert, snd_beep, snd_chime):

    """ Controls thread """

    if not self.enable_camera:
      return

    # *** apply brake hysteresis ***
    brake, self.braking, self.brake_steady = actuator_hystereses(actuators.brake, self.braking, self.brake_steady, CS.v_ego, CS.CP.carFingerprint)

    # *** no output if not enabled ***
    if not enabled and CS.pcm_acc_status:
      # send pcm acc cancel cmd if drive is disabled but pcm is still on, or if the system can't be activated
      pcm_cancel_cmd = True

    # *** rate limit after the enable check ***
    self.brake_last = rate_limit(brake, self.brake_last, -2., 1./100)

    # vehicle hud display, wait for one update from 10Hz 0x304 msg
    if hud_show_lanes:
      hud_lanes = 1
    else:
      hud_lanes = 0

    if enabled:
      if hud_show_car:
        hud_car = 2
      else:
        hud_car = 1
    else:
      hud_car = 0

    # For lateral control-only, send chimes as a beep since we don't send 0x1fa
    if CS.CP.radarOffCan:
      snd_beep = snd_beep if snd_beep is not 0 else snd_chime

    #print chime, alert_id, hud_alert
    fcw_display, steer_required, acc_alert = process_hud_alert(hud_alert)

    hud = HUDData(int(pcm_accel), int(round(hud_v_cruise)), 1, hud_car,
                  0xc1, hud_lanes, int(snd_beep), snd_chime, fcw_display, acc_alert, steer_required)

    if not all(isinstance(x, int) and 0 <= x < 256 for x in hud):
      hud = HUDData(0xc6, 255, 64, 0xc0, 209, 0x40, 0, 0, 0, 0)

    # **** process the car messages ****

    # *** compute control surfaces ***
    #BRAKE_MAX = 1024/4 2018.09.02 move into class , 2018.09.03 change ILX to dummy
    #if CS.CP.carFingerprint in (CAR.DUMMY):
     #    STEER_MAX = 0xF00
     #    BRAKE_MAX = 1024 / 4
    if CS.CP.carFingerprint in (CAR.SOUL, CAR.SOUL1, CAR.SOUL2): # 2018.09.04 add different fingerprint messages Kia
          STEER_MAX = 0.15  #2018.09.25 steering torque TODO: need tune parameter max steering allow, value clip when coming on can
          BRAKE_MAX = 100 #2018.09.02 DV TODO: need to tune for BRAKE_COMMAND_pedal_command, but the value clip when coming out
    else:
      STEER_MAX = 0.15   #2018.09.25 test
      BRAKE_MAX = 1024 / 4

      # init safety test lines (2018.09.04 , test parameter)
    #if genericToggle:
      #  actuators.steer = 1.0
      # end safety test lines

    # steer torque is converted back to CAN reference (positive when steering right)
    apply_gas = clip(actuators.gas, 0., 1.)
    apply_brake = int(clip(self.brake_last * BRAKE_MAX, 0, BRAKE_MAX - 1))
    apply_steer = int(clip(-actuators.steer * STEER_MAX, -STEER_MAX, STEER_MAX))
    print("carcontroller.py apply gas")
    print(apply_gas)
    print("carcontroller.py apply brake")
    print(apply_brake)
    print("carcontroller.py apply steer")
    print(apply_steer)
    print("carcontroller.py brake_last, brake max")
    print(self.brake_last)
    print(BRAKE_MAX)
    print("carcontroller.py actuator gas")
    print(actuators.gas)
    print("carcontroller.py actuator.steer, steer max")
    print(actuators.steer)
    print(STEER_MAX)

    #2018.09.25 borrow from toyota
    #Steer angle limits (tested at the Crows Landing track and considered ok)
    ANGLE_MAX_BP = [0., 5.]
    ANGLE_MAX_V = [510., 300.]
    ANGLE_DELTA_BP = [0., 5., 15.]
    ANGLE_DELTA_V = [5., .8, .15]  # windup limit
    ANGLE_DELTA_VU = [5., 3.5, 0.4]  # unwind limit

    # steer angle (2018.09.25 BORROW FROM TOYOTA)
    #if self.steer_angle_enabled and CS.ipas_active:
    print("carcontroller.py actuator.steerAngle")
    print(actuators.steerAngle)
    apply_angle = actuators.steerAngle
    angle_lim = interp(CS.v_ego, ANGLE_MAX_BP, ANGLE_MAX_V)
    print("carcontroller.py angle_lim")
    print(angle_lim)
    print("carcontroller.py Angle_max_BP, Angle_max_V")
    print(ANGLE_MAX_BP)
    print(ANGLE_MAX_V)
    apply_angle = clip(apply_angle, -angle_lim, angle_lim)
    print("carcontroller.py apply angle after clip")
    print(apply_angle)

      # windup slower
    print("carcontroller.py self.last_angle")
    print(self.last_angle)
    if self.last_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_angle):
        print("carcontroller.py absolute apply_angles section")
        angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_V)
        print("carcontroller.py v_ego, Angle_DeltaBP, angle Delta V, Angle_rate Lim")
        print(CS.v_ego)
        print(ANGLE_DELTA_BP)
        print(ANGLE_DELTA_V)
        print(angle_rate_lim)
    else:
        angle_rate_lim = interp(CS.v_ego, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
        print("carcontroller.py if not self.lastangle v_ego, angle_rate_lim, Angle delta bp, angle delta vu")
        print(CS.v_ego)
        print(angle_rate_lim)
        print(ANGLE_DELTA_BP)
        print(ANGLE_DELTA_VU)

    self.last_angle = apply_angle
    print("carcontroller.py apply_angle before clip")
    print(apply_angle)
    apply_angle = clip(apply_angle, self.last_angle - angle_rate_lim, self.last_angle + angle_rate_lim)
    print("carcontroller.py apply_angle after clip, self.last angle, angle_rate_lim")
    print(apply_angle)
    print(self.last_angle)
    print(angle_rate_lim)

    #2018.09.25 python 1D interpolation function to get angle to -1 to +1
    x_angle_range = np.r[-360, 360, 1000]
    print("carcontroller.py x_angle_range")
    print(x_angle_range)
    y_apply_torque_kia = np.r[-1.0, 1.0, 1000]
    print("carcontroller.py")
    print(y_apply_torque_kia)
    x_apply_angle_input = apply_angle
    f = interpolate.interp1d(x_angle_range, y_apply_torque_kia)
    steeringkia = f(x_apply_angle_input)
    print("carcontroller.py steeringkia")
    print(steeringkia)


     #2018.09.04 hyundai make this change, but need to understand more, we don't have steer_torque driver value
    #apply_steer = actuators.steer * SteerLimitParams.STEER_MAX
   # apply_steer = apply_std_steer_torque_limits(apply_steer, self.apply_steer_last, CS.steer_torque_driver,
                                               # SteerLimitParams)

    # any other cp.vl[0x18F]['STEER_STATUS'] is common and can happen during user override. sending 0 torque to avoid EPS sending error 5
    lkas_active = enabled and not CS.steer_not_allowed  #2018.09.04 coming from steering report that driver not over
    # 2018.09.12 enabled come from planner.py
    #print("carcontroller.py lkas_active")
    #print(lkas_active)
   # print("carcontroller.py CS.steer_not_allowed")
    #print(CS.steer_not_allowed)

    # Send CAN commands.
    can_sends = []
    #canbus = self.canbus  2018.09.06 comment out canbus

    #2018.09.06 10:30PM remove canbus.powertrain
    # Send steering command.
    idx = frame % 4  #2018.09.02 this mod it get the remainder?? #2018.09.03 remove idx, 2018.09.06 12:33 AM add canbus.powertrain
                     #2018.09.11 update the argument to match for both carcontroller.py and kiacan to fix argument error
    can_sends.append(kiacan.create_steering_control_enable(self.packer, lkas_active))
    can_sends.append(kiacan.create_steering_control(self.packer, apply_steer, lkas_active))  #2018.09.26 reverse back to steer
    can_sends.append(kiacan.create_steering_control_disable(self.packer, lkas_active))

    # Send dashboard UI commands.
    if (frame % 10) == 0:
      idx = (frame/10) % 4                                #2018.09.06 12:33AM add canbus.powertrain , 2019.09.11 change to match number of argument returuning
      can_sends.extend(kiacan.create_ui_commands(self.packer, pcm_speed, hud, idx))

    if CS.CP.radarOffCan:
      # If using stock ACC, spam cancel command to kill gas when OP disengages.
      if pcm_cancel_cmd:
        can_sends.append(kiacan.spam_buttons_command(self.packer, CruiseButtons.CANCEL, idx))
      elif CS.stopped:
        can_sends.append(kiacan.spam_buttons_command(self.packer, CruiseButtons.RES_ACCEL, idx))
    else:
      # Send gas and brake commands.
      #2018.09.02 Paras already change here for oscc kia soul kit oscc doesn't need idx
      if (frame % 2) == 0:
        idx = (frame / 2) % 4
        can_sends.append(
          kiacan.create_brake_enable_soul(self.packer,  apply_brake)) #enable brake command,  #2018.09.06 12:33AM add canbus.powertrain
        can_sends.append(
          kiacan.create_brake_command_soul(self.packer,  apply_brake)) #creating brake command  #2018.09.06 12:33AM add canbus.powertrain
        can_sends.append(
          kiacan.create_brake_disable_soul(self.packer,  apply_brake)) #disable brake command #2018.09.06 12:33AM add canbus.powertrain
        can_sends.append(
          kiacan.create_brake_command(self.packer,  apply_brake, pcm_override, pcm_cancel_cmd, hud.chime, hud.fcw, idx)) #creating brake command for chime & FCW, brake command need idx
        # 2018.09.06 12:33AM add canbus.powertrain  to distinction of can bus channel

          #2018.09.02 DV TODO: need to confirm THROTTLE PEDAL command pedal command
          #2018.09.03 remove idx for oscc kit
        if CS.CP.enableGasInterceptor:
          # send exactly zero if apply_gas is zero. Interceptor will send the max between read value and apply_gas.
          # This prevents unexpected pedal range rescaling
          # 2018.09.06 12:33AM add canbus.powertrain  to distinction of can bus channel
          can_sends.append(kiacan.create_gas_command_enable(self.packer, apply_gas))
          can_sends.append(kiacan.create_gas_command(self.packer, apply_gas))
          can_sends.append(kiacan.create_gas_command_disable(self.packer, apply_gas))

          
      # radar at 20Hz, but these msgs need to be sent at 50Hz on ilx (seems like an Acura bug)
      #change 2018.09.03 change ILX to dummy
      if CS.CP.carFingerprint == CAR.DUMMY:
        radar_send_step = 2
      else:
        radar_send_step = 1  #2018.09.19 12:35PMEST try to send out 10hz for radar

      if (frame % radar_send_step) == 0:
        idx = (frame/radar_send_step) % 4
        if not self.new_radar_config:  # only change state once
          self.new_radar_config = car.RadarState.Error.wrongConfig in radar_error
        can_sends.extend(kiacan.create_radar_commands(CS.v_ego, CS.CP.carFingerprint, self.new_radar_config, idx))
            #2018.09.06 12:39AM change to just create_radar_commands to match kiacan.py

    ### Send messages to canbus
    sendcan.send(can_list_to_can_capnp(can_sends, msgtype='sendcan').to_bytes())
