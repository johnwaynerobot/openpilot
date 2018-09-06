from common.numpy_fast import interp
from common.kalman.simple_kalman import KF1D
from selfdrive.can.parser import CANParser
from selfdrive.config import Conversions as CV
from selfdrive.car.kia.values import CAR, DBC, STEER_THRESHOLD, SPEED_FACTOR # change to Kia Soul folder
import numpy as np
from cereal import car

#2018.09.04 comment out Gear shifter no use
#def parse_gear_shifter(gear, vals):
#
 # val_to_capnp = {'P': 'park', 'R': 'reverse', 'N': 'neutral',
#                  'D': 'drive', 'S': 'sport', 'L': 'low'}
#  try:
#    return val_to_capnp[vals[gear]]
#  except KeyError:
#    return "unknown"


def calc_cruise_offset(offset, speed):
  # euristic formula so that speed is controlled to ~ 0.3m/s below pid_speed
  # constraints to solve for _K0, _K1, _K2 are:
  # - speed = 0m/s, out = -0.3
  # - speed = 34m/s, offset = 20, out = -0.25
  # - speed = 34m/s, offset = -2.5, out = -1.8
  _K0 = -0.3
  _K1 = -0.01879
  _K2 = 0.01013
  return min(_K0 + _K1 * speed + _K2 * speed * offset, 0.)


#2018.09.04 change cansignal to can_parser
def get_can_signals(CP, canbus):

    # this function generates lists for signal, messages and initial values
    signals = [
           #signal name, sig_address, default
          ("VS_TCU", "TM_DATA", 0),  #2018.09.02 DV transmission vehicle speed B0_1088 TCU2
          ("WHEEL_SPEED_FL", "WHEEL_SPEEDS", 0),  #2018.09.02 DV  Wheel speed B0_1200
          ("WHEEL_SPEED_FR", "WHEEL_SPEEDS", 0),  #2018.09.02 DV  Wheel speed B0_1200
          ("WHEEL_SPEED_RL", "WHEEL_SPEEDS", 0),  #2018.09.02 DV  Wheel speed B0_1200
          ("WHEEL_SPEED_RR", "WHEEL_SPEEDS", 0),  #2018.09.02 DV  Wheel speed B0_1200
          ("STEER_ANGLE", "STEERING_SENSORS", 0), #2018.09.02 DV  B0_688
          ("STEER_ANGLE_RATE", "STEERING_SENSORS", 0), #2018.09.02 DV  B0_688
          # 2018.09.02 D.V B0_357 #TODO confirm steering_torque_sensor or use steering_report_operator_override
          #use for judgement that driver override the steering to disable
          ("STEER_TORQUE_SENSOR", "STEER_STATUS", 0),
          ("LEFT_BLINKER", "SCM_FEEDBACK", 0), #2018.09.02 D.V B0_1680 modified dbc to match
          ("RIGHT_BLINKER", "SCM_FEEDBACK", 0), #2018.09.02 D.V B0_1680 modified dbc to match
          ("GEAR", "GEARBOX", 0),  #2018.09.02 D.V B0_880 modified dbc to match
                                    #Transmission Gear (0 = N or P, 1-6 = Fwd, 14 = Rev)
          ("BRAKE_REPORT_dtcs", "BRAKE_REPORT", 0), #2018.09.02 BRAKE_ERROR1 and BRAKE_ERROR2 equal to B0_115 BRAKE_REPORT_dtcs
          # 2018.09.02 D.V add in B0_1680 modified dbc value 0 is SEATBELT_DRIVER_LATCHED same as lamp 0 is belt on 1 belt off
          ("SEATBELT_DRIVER_LAMP", "SEATBELT_STATUS", 1),
          # 2018.09.02 D.V  brake switch status push or not push
          # set brake switch same as brake pressed B0_809 ENG_INFO
          ("BRAKE_PRESSED", "ENG_INFO", 0), # initial value is 0, 2 is brake pressed
          ("CRUISE_BUTTONS", "SCM_BUTTONS", 0), #2018.09.02 use UI B0_422 (0x1A6) messages
          ("ESP_DISABLED", "VSA_STATUS", 0),  #2018.09.02 modified dbc to match use B0_339 ESP_DISABLED when VSA button push OFF
          ("HUD_LEAD", "ACC_HUD", 0), #2018.09.02 coming from EON for Lead Distance)
          # 2018.09.02 DV change USER_BRAKE to BRAKE_REPORT_operator_override B0_115
          ("BRAKE_REPORT_operator_override", "BRAKE_REPORT", 0),
          #2018.09.02 DV change STEER_STATUS to STEERING_REPORT_operator_override from B0_131 STEERING_REPORT
          ("STEERING_REPORT_operator_override", "STEERING_REPORT", 0),
          #2018.09.02 DV add modified dbc B0 1306 -TM Gear
          #("GEAR_SHIFTER", "TM_GEAR", 0), #2018.09.02 DV change gear shifter to individual message
          ("TM_PARK", "TM_GEAR", 0),
          ("TM_REVERSE", "TM_GEAR", 0),
          ("TM_NEUTRAL", "TM_GEAR", 0),
          ("TM_DRIVE", "TM_GEAR", 0),
          #2018.09.02 DV change Pedal Gas to ENG_INFO B0_809
          ("PEDAL_GAS", "ENG_INFO", 0),
          ("CRUISE_SETTING", "SCM_BUTTONS", 0),  #UI from 0x1A6
          #2018.09.02 DV change ACC_STATUS to UI Main 1
          ("MAIN_ON", "SCM_BUTTONS", 0),   #ACC_STATUS to MAIN_ON 2018.09.02 DV
          ("YAW_RATE", "IMU", 0) #2018.09.04 Input vehicle Yaw rate
      ]

    checks = [
          # address,  message frequency
          #("TCU2", 0), # 2018.09.04 dont know frequency comment out check
          ("WHEEL_SPEEDS", 50),
          ("STEERING_SENSORS", 100),
          #("SCM_FEEDBACK", 5,  #either 5 (200ms) or 10 (100ms), not sure ignore
          #("GEARBOX", 0), #2018.09.04 don't know frequency ignore
          #("STANDSTILL", 50), Standstill VSA
         # ("SEATBELT_STATUS", 0),
         #("CRUISE", 10),  #2018.09.03 remove cruise check
         # ("ENG_INFO", 100),  #2018.09.02 change POWERTRAIN DATA To ENG_INFO
          #("VSA_STATUS", 0),
          ("SCM_BUTTONS", 25), #2018.09.04 come from 0x1A6
      ]


    #if CP.radarOffCan:
    # Civic is only bosch to use the same brake message as other hondas.
    #    if CP.carFingerprint != CAR.CIVIC_HATCH:
    #      signals += [("BRAKE_PRESSED", "BRAKE_MODULE", 0)]
     #     checks += [("BRAKE_MODULE", 50)]
    #    signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
     #               ("MAIN_ON", "SCM_FEEDBACK", 0),
    #                ("EPB_STATE", "EPB_STATUS", 0),
    #                ("BRAKE_HOLD_ACTIVE", "VSA_STATUS", 0),
   #                 ("CRUISE_SPEED", "ACC_HUD", 0)]
     #   checks += [("GAS_PEDAL_2", 100)]
     # else:
        # Nidec signals.
        #signals += [("CRUISE_SPEED_PCM", "CRUISE", 0), #2018.09.02 don't have on Kia soul
              #      ("CRUISE_SPEED_OFFSET", "CRUISE_PARAMS", 0)] # 2018.09.02 DV comment out, dont have
       # checks += [("CRUISE_PARAMS", 50)]       #2018.09.02 DV comment out don't have

    if CP.carFingerprint in (CAR.DUMMY):
        signals += [("DRIVERS_DOOR_OPEN", "SCM_FEEDBACK", 1)]

    elif CP.carFingerprint == CAR.SOUL:
        signals += [("DOOR_OPEN_FL", "SCM_FEEDBACK", 1)] #2018.09.03 update

    elif CP.carFingerprint == CAR.SOUL1:
        signals += [("DOOR_OPEN_FL", "SCM_FEEDBACK", 1)]  # 2018.09.04 update

    elif CP.carFingerprint == CAR.SOUL2:
        signals += [("DOOR_OPEN_FL", "SCM_FEEDBACK", 1)]  # 2018.09.04 update

    if CP.carFingerprint == CAR.DUMMY:
        signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
                ("MAIN_ON", "SCM_FEEDBACK", 0),
                ("EPB_STATE", "EPB_STATUS", 0),
                ("BRAKE_HOLD_ACTIVE", "VSA_STATUS", 0)]
    elif CP.carFingerprint == CAR.SOUL:  # 2018.09.02 DV Kia Soul UI 0x1A6 ADAS Cruise button
        signals += [("MAIN_ON", "SCM_BUTTONS", 0)]
        signals += [("CF_Clu_CruiseSwMain", "CLU1", 0)] #2018.09.04 signal for Steering/brake/gas max test

    elif CP.carFingerprint == CAR.SOUL1:  # 2018.09.03 DV Kia Soul UI 0x1A6 ADAS Cruise button
        signals += [("MAIN_ON", "SCM_BUTTONS", 0)]
        signals += [("CF_Clu_CruiseSwMain", "CLU1", 0)]  # 2018.09.04 signal for Steering/brake/gas max test

    elif CP.carFingerprint == CAR.SOUL2:  # 2018.09.03 DV Kia Soul UI 0x1A6 ADAS Cruise button
        signals += [("MAIN_ON", "SCM_BUTTONS", 0)]
        signals += [("CF_Clu_CruiseSwMain", "CLU1", 0)]  # 2018.09.04 signal for Steering/brake/gas max test

        # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:

        signals.append(("THROTTLE_REPORT_operator_override", "THROTTLE_REPORT", 0)) #2018.09.02 DV add change for Kia soul
        #checks.append(("THROTTLE_REPORT", 50)) # signal and frequency#2018.09.02 DV add change for Kia soul
        #comment out check, frequency need to verify
    return signals, checks


def get_can_parser(CP, canbus):  #2018.09.04 combine in above
    signals, checks = get_can_signals(CP)
    #canbus.powertrain is can 0 (bus 0)
    return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, canbus.powertrain)


class CarState(object):
  def __init__(self, CP, canbus):
      #intializae can parser
    self.CP = CP
    self.car_fingerprint = CP.carFingerprint   #borrow from subaru carstate
    #.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    #2018.09.05 remove define twice
    #self.shifter_values = self.can_define.dv["GEARBOX"]["GEAR_SHIFTER"]
    #2018.09.02 DV change the value for Kia soul gear info, link in interface.py
    #self.shifter_REVERSE = self.can_define.dv["TM_GEAR"]["TM_REVERSE"]
    #self.shifter_NEUTRAL = self.can_define.dv["TM_GEAR"]["TM_NEUTRAL"]
    #self.shifter_DRIVE = self.can_define.dv["TM_GEAR"]["TM_DRIVE"]
    #if self.shifter_PARK == 1:
     #   self.gear_shifter = "park"
    #elif self.shifter_NEUTRAL == 1:
     #   self.gear_shifter = "neutral"
   # elif self.shifter_DRIVE == 1:
     #   self.gear_shifter = "drive"
    #elif self.shifter_REVERSE == 1:
    #    self.gear_shifter = "reverse"
    #else:
    #    self.gear_shifter = "unknown"
    # end of gear parse definition

    self.steer_torque_driver = 0     #borrow from subaru carstate.py
    self.steer_not_allowed = False   #borrow from subaru carstate.py
    self.main_on = False             #borrow from subaru carstate.py
    self.user_gas, self.user_gas_pressed = 0., 0
    self.brake_switch_prev = 0
    self.brake_switch_ts = 0

    self.cruise_buttons = 0
    self.cruise_setting = 0
    self.v_cruise_pcm_prev = 0
    self.blinker_on = 0

    self.left_blinker_on = 0
    self.right_blinker_on = 0

    self.stopped = 0

    # vEgo kalman filter
    dt = 0.01
    # Q = np.matrix([[10.0, 0.0], [0.0, 100.0]])
    # R = 1e3
    #self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
    #                     A=[[1.0, dt], [0.0, 1.0]],
     #                    C=[[1.0, 0.0]],
       #                  K=[[0.12287673], [0.29666309]])
    #
    #hyundai change 2018.09.04
    self.v_ego_kf = KF1D(x0=np.matrix([[0.0], [0.0]]),
                         A=np.matrix([[1.0, dt], [0.0, 1.0]]),
                         C=np.matrix([1.0, 0.0]),
                         K=np.matrix([[0.12287673], [0.29666309]]))

    self.v_ego = 0.0

  def update(self, cp):

    # copy can_valid
    self.can_valid = cp.can_valid

    # car param# 2018.09.06 12:33AM add canbus.powertrain  to distinction of can bus channels
    v_weight_v = [0., 1.]  # don't trust smooth speed at low values to avoid premature zero snapping
    v_weight_bp = [1., 6.]   # smooth blending, below ~0.6m/s the smooth speed snaps to zero

    # update prevs, update must run once per loop
    self.prev_cruise_buttons = self.cruise_buttons
    self.prev_cruise_setting = self.cruise_setting
    self.prev_blinker_on = self.blinker_on

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on

    # ******************* parse out can *******************

    #2018.09.03 Gear name define

    # can_gear_shifter = int(cp.vl["GEARBOX"]['GEAR_SHIFTER'])
    # self.gear_shifter = parse_gear_shifter(can_gear_shifter, self.shifter_values)
    # 2018.09.02 DV change gear shifter info for kia soul
    self.shifter_PARK = cp.vl["TM_GEAR"]["TM_PARK"]
    self.shifter_REVERSE = cp.vl["TM_GEAR"]["TM_REVERSE"]
    self.shifter_NEUTRAL = cp.vl["TM_GEAR"]["TM_NEUTRAL"]
    self.shifter_DRIVE = cp.vl["TM_GEAR"]["TM_DRIVE"]

    if self.shifter_PARK == 1:
        self.gear_shifter = "park"
    elif self.shifter_NEUTRAL == 1:
        self.gear_shifter = "neutral"
    elif self.shifter_DRIVE == 1:
        self.gear_shifter = "drive"
    elif self.shifter_REVERSE == 1:
        self.gear_shifter = "reverse"
    else:
        self.gear_shifter = "unknown"
    # end of gear parse definition

    if self.CP.carFingerprint == CAR.SOUL:   # 2018.09.04 add multiple soul because can change
        self.standstill = cp.vl["TM_DATA"]['VS_TCU'] < 0.1
        self.door_all_closed = not cp.vl["SCM_FEEDBACK"]['DOOR_OPEN_FL']
        self.steer_error = cp.vl["STEERING_REPORT"]['STEERING_REPORT_dtcs'] != 0
        self.steer_not_allowed = cp.vl["STEERING_REPORT"]['STEERING_REPORT_operator_override'] != 0
        self.steer_warning = cp.vl["STEERING_REPORT"]['STEERING_REPORT_operator_override'] != 0
        self.brake_error = cp.vl["BRAKE_REPORT"]['BRAKE_REPORT_dtcs']
        self.esp_disabled = cp.vl["VSA_STATUS"]['ESP_DISABLED']
        self.seatbelt = not cp.vl["SEATBELT_STATUS"]['SEATBELT_DRIVER_LAMP']  # 2018.09.04 0 is 1, 1 is off
        self.angle_steers = cp.vl["STEERING_SENSORS"]['STEER_ANGLE']
        self.angle_steers_rate = cp.vl["STEERING_SENSORS"]['STEER_ANGLE_RATE']
        self.cruise_setting = cp.vl["SCM_BUTTONS"]['CRUISE_SETTING']
        self.cruise_buttons = cp.vl["SCM_BUTTONS"]['CRUISE_BUTTONS']
        self.blinker_on = cp.vl["SCM_FEEDBACK"]['LEFT_BLINKER'] or cp.vl["SCM_FEEDBACK"]['RIGHT_BLINKER']
        self.left_blinker_on = cp.vl["SCM_FEEDBACK"]['LEFT_BLINKER']
        self.right_blinker_on = cp.vl["SCM_FEEDBACK"]['RIGHT_BLINKER']
        self.gear = cp.vl["GEARBOX"]['GEAR']  # 2018.09.04
        self.park_brake = 0  # TODO
        self.brake_hold = 0  # TODO
        self.main_on = cp.vl["SCM_BUTTONS"]['MAIN_ON']
        self.brake_switch = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] ==2 # 2018.09.02 DV "2"value is brake switch ON
        self.brake_pressed = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] ==2 # 2018.09.02 change for Kia soul
        self.brake_switch_prev = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] == 2  # 2018.09.02 DV "2"value is brake switch ON
        self.brake_switch_ts = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] == 2
        self.stopped = cp.vl["ACC_HUD"]['CRUISE_SPEED'] == 252.
        self.cruise_speed_offset = calc_cruise_offset(0, self.v_ego)
        # On set, cruise set speed pulses between 254~255 and the set speed prev is set to avoid this.
        self.v_cruise_pcm = self.v_cruise_pcm_prev if cp.vl["ACC_HUD"]['CRUISE_SPEED'] > 160.0 else cp.vl["ACC_HUD"]['CRUISE_SPEED']
        self.v_cruise_pcm_prev = self.v_cruise_pcm
        self.yaw_rate = cp.vl["IMU"]['YAW_RATE']
        self.generic_toggle = cp.vl["CLU1"]['CF_Clu_CruiseSwMain'] #2019.09.04 use stock cruise main switch for steer max test

    elif self.CP.carFingerprint in (CAR.SOUL1, CAR.SOUL2): # 2018.09.04 update
            self.standstill = cp.vl["TM_DATA"]['VS_TCU'] < 0.1
            self.door_all_closed = not cp.vl["SCM_FEEDBACK"]['DOOR_OPEN_FL']
            self.steer_error = cp.vl["STEERING_REPORT"]['STEERING_REPORT_dtcs'] != 0
            self.steer_not_allowed = cp.vl["STEERING_REPORT"]['STEERING_REPORT_operator_override'] != 0
            self.steer_warning = cp.vl["STEERING_REPORT"]['STEERING_REPORT_operator_override'] != 0
            self.brake_error = cp.vl["BRAKE_REPORT"]['BRAKE_REPORT_dtcs']
            self.esp_disabled = cp.vl["VSA_STATUS"]['ESP_DISABLED']
            self.seatbelt = not cp.vl["SEATBELT_STATUS"]['SEATBELT_DRIVER_LAMP']  # 2018.09.04 0 is 1, 1 is off
            self.angle_steers = cp.vl["STEERING_SENSORS"]['STEER_ANGLE']
            self.angle_steers_rate = cp.vl["STEERING_SENSORS"]['STEER_ANGLE_RATE']
            self.cruise_setting = cp.vl["SCM_BUTTONS"]['CRUISE_SETTING']
            self.cruise_buttons = cp.vl["SCM_BUTTONS"]['CRUISE_BUTTONS']
            self.blinker_on = cp.vl["SCM_FEEDBACK"]['LEFT_BLINKER'] or cp.vl["SCM_FEEDBACK"]['RIGHT_BLINKER']
            self.left_blinker_on = cp.vl["SCM_FEEDBACK"]['LEFT_BLINKER']
            self.right_blinker_on = cp.vl["SCM_FEEDBACK"]['RIGHT_BLINKER']
            self.gear = cp.vl["GEARBOX"]['GEAR']  # 2018.09.04
            self.park_brake = 0  # TODO
            self.brake_hold = 0  # TODO
            self.main_on = cp.vl["SCM_BUTTONS"]['MAIN_ON']  #ACC main using UI
            self.brake_switch = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] ==2 # 2018.09.02 DV "2"value is brake switch ON
            self.brake_pressed = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] ==2  # 2018.09.02 change for Kia soul
            self.brake_switch_prev = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] ==2  #2018.09.02 DV "2"value is brake switch ON
            self.brake_switch_ts = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] ==2
            self.stopped = cp.vl["ACC_HUD"]['CRUISE_SPEED'] == 252.
            self.cruise_speed_offset = calc_cruise_offset(0, self.v_ego)
            self.v_cruise_pcm = self.v_cruise_pcm_prev if cp.vl["ACC_HUD"]['CRUISE_SPEED'] > 160.0 else cp.vl["ACC_HUD"]['CRUISE_SPEED']
            self.v_cruise_pcm_prev = self.v_cruise_pcm
            self.yaw_rate = cp.vl["IMU"]['YAW_RATE']
            self.generic_toggle = cp.vl["CLU1"]['CF_Clu_CruiseSwMain']  # 2019.09.04 use stock cruise main switch for steer max test

    # calc best v_ego estimate, by averaging two opposite corners
    speed_factor = SPEED_FACTOR[self.CP.carFingerprint]
    self.v_wheel_fl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FL'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_fr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_FR'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_rl = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RL'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel_rr = cp.vl["WHEEL_SPEEDS"]['WHEEL_SPEED_RR'] * CV.KPH_TO_MS * speed_factor
    self.v_wheel = (self.v_wheel_fl+self.v_wheel_fr+self.v_wheel_rl+self.v_wheel_rr)/4.

    # blend in transmission speed at low speed, since it has more low speed accuracy
    self.v_weight = interp(self.v_wheel, v_weight_bp, v_weight_v)
    speed = (1. - self.v_weight) * cp.vl["TM_DATA"]['VS_TCU'] * CV.KPH_TO_MS * speed_factor + \
      self.v_weight * self.v_wheel      #2018.09.02 DV change ENGINE_DATA to TM_DATA and VS_TCU to match KIA SOUL

    if abs(speed - self.v_ego) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_x = [[speed], [0.0]]  #kalman filter here

    self.v_ego_raw = speed
    v_ego_x = self.v_ego_kf.update(speed)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])

    # this is a hack for the interceptor. This is now only used in the simulation
    # TODO: Replace tests by toyota so this can go away
    if self.CP.enableGasInterceptor:
      self.user_gas = cp.vl["THROTTLE_REPORT"]['THROTTLE_REPORT_operator_override'] #2018.09.02 change for Kia soul when gas being press
      self.user_gas_pressed = self.user_gas > 0 # this works because interceptor read < 0 when pedal position is 0. Once calibrated, this will change



    self.pedal_gas = cp.vl["ENG_INFO"]['PEDAL_GAS'] #2018.09.02 DV change for pedal gas
    # crv doesn't include cruise control
    if self.CP.carFingerprint in (CAR.SOUL, CAR.SOUL1):  #2018.09.04
      self.car_gas = self.pedal_gas
    elif self.CP.carFingerprint in (CAR.SOUL2): #2018.09.05 getridd of dummy put soul2 to prevent duplicate
            self.car_gas = cp.vl["ENG_INFO"]['PEDAL_GAS'] #2018.09.02 DV cruise control gas not available change to pedal gas

    #self.steer_torque_driver = cp.vl["STEER_STATUS"]['STEER_TORQUE_SENSOR'] 2018.09.02 comment out to use steering operator override
    #TODO find actual Steering Driver Torque value on CAN
    self.steer_torque_driver = cp.vl["STEERING_REPORT"]['STEERING_REPORT_operator_override']
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD[self.CP.carFingerprint] #threshold set in values.py

    #self.user_brake = cp.vl["VSA_STATUS"]['USER_BRAKE']
    self.user_brake = cp.vl["BRAKE_REPORT"]['BRAKE_REPORT_operator_override']  #2018.09.02 DV add for Kia soul
    #self.pcm_acc_status = cp.vl["POWERTRAIN_DATA"]['ACC_STATUS']
    self.pcm_acc_status = cp.vl["SCM_BUTTONS"]['MAIN_ON']   #2018.09.02 DV change to UI 0x1A6 main switch
    self.hud_lead = cp.vl["ACC_HUD"]['HUD_LEAD']


# carstate standalone tester
if __name__ == '__main__':
  import zmq
  context = zmq.Context()

  class CarParams(object):
    def __init__(self):
      self.carFingerprint = "KIA SOUL TEST"
      self.enableGasInterceptor = 1
  CP = CarParams()
  CS = CarState(CP)

  # while 1:
  #   CS.update()
  #   time.sleep(0.01)
