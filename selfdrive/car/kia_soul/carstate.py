from common.numpy_fast import interp
from common.kalman.simple_kalman import KF1D
from selfdrive.can.parser import CANParser, CANDefine
from selfdrive.config import Conversions as CV
from selfdrive.car.kia_soul.values import CAR, DBC, STEER_THRESHOLD, SPEED_FACTOR # change to Kia Soul folder

def parse_gear_shifter(gear, vals):

  val_to_capnp = {'P': 'park', 'R': 'reverse', 'N': 'neutral',
                  'D': 'drive', 'S': 'sport', 'L': 'low'}
  try:
    return val_to_capnp[vals[gear]]
  except KeyError:
    return "unknown"


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


def get_can_signals(CP):

    # this function generates lists for signal, messages and initial values
    signals = [
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
          ("BRAKE_REPORT_dtcs", "BRAKE_REPORT", 1), #2018.09.02 BRAKE_ERROR1 and BRAKE_ERROR2 equal to B0_115 BRAKE_REPORT_dtcs
          # 2018.09.02 D.V add in B0_1680 modified dbc value 0 is SEATBELT_DRIVER_LATCHED same as lamp 0 is belt on 1 belt off
          ("SEATBELT_DRIVER_LAMP", "SEATBELT_STATUS", 1),
          # 2018.09.02 D.V  brake switch status push or not push
          # set brake switch same as brake pressed B0_809 ENG_INFO
          ("BRAKE_PRESSED", "ENG_INFO", 0), # initial value is 0, 2 is brake pressed
          ("CRUISE_BUTTONS", "SCM_BUTTONS", 0), #2018.09.02 use UI B0_422 (0x1A6) messages
          ("ESP_DISABLED", "VSA_STATUS", 1),  #2018.09.02 modified dbc to match use B0_339 ESP_DISABLED when VSA button push OFF
          ("HUD_LEAD", "ACC_HUD", 0), #2018.09.02 coming from EON for Lead Distance)
          # 2018.09.02 DV change USER_BRAKE to BRAKE_REPORT_operator_override B0_115
          ("BRAKE_REPORT_operator_override", "BRAKE_REPORT", 0),
          #2018.09.02 DV change STEER_STATUS to STEERING_REPORT_operator_override from B0_131 STEERING_REPORT
          ("STEERING_REPORT_operator_override", "STEERING_REPORT", 5),
          #2018.09.02 DV add modified dbc B0 1306 -TM Gear
          #("GEAR_SHIFTER", "TM_GEAR", 0), #2018.09.02 DV change gear shifter to individual message
          ("TM_PARK", "TM_GEAR", 1),
          ("TM_REVERSE", "TM_GEAR", 0),
          ("TM_NEUTRAL", "TM_GEAR", 0),
          ("TM_DRIVE", "TM_GEAR", 0),
          #2018.09.02 DV change Pedal Gas to ENG_INFO B0_809
          ("PEDAL_GAS", "ENG_INFO", 0),
          ("CRUISE_SETTING", "SCM_BUTTONS", 0),  #UI from 0x1A6
          #2018.09.02 DV change ACC_STATUS to UI Main 1
          ("MAIN_ON", "SCM_BUTTONS", 0),   #ACC_STATUS to MAIN_ON 2018.09.02 DV
      ]

    checks = [
          ("TCU2", 100),
          ("WHEEL_SPEEDS", 50),
          ("STEERING_SENSORS", 100),
          ("SCM_FEEDBACK", 10),
          ("GEARBOX", 100),
          #("STANDSTILL", 50), Standstill VSA
          ("SEATBELT_STATUS", 10),
          ("CRUISE", 10),
          ("ENG_INFO", 100),  #2018.09.02 change POWERTRAIN DATA To ENG_INFO
          ("VSA_STATUS", 50),
          ("SCM_BUTTONS", 25), #2018.09.02 come from 0x1A6
      ]


    if CP.radarOffCan:
    # Civic is only bosch to use the same brake message as other hondas.
        if CP.carFingerprint != CAR.CIVIC_HATCH:
          signals += [("BRAKE_PRESSED", "BRAKE_MODULE", 0)]
          checks += [("BRAKE_MODULE", 50)]
        signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
                    ("MAIN_ON", "SCM_FEEDBACK", 0),
                    ("EPB_STATE", "EPB_STATUS", 0),
                    ("BRAKE_HOLD_ACTIVE", "VSA_STATUS", 0),
                    ("CRUISE_SPEED", "ACC_HUD", 0)]
        checks += [("GAS_PEDAL_2", 100)]
     # else:
        # Nidec signals.
        #signals += [("CRUISE_SPEED_PCM", "CRUISE", 0), #2018.09.02 don't have on Kia soul
              #      ("CRUISE_SPEED_OFFSET", "CRUISE_PARAMS", 0)] # 2018.09.02 DV comment out, dont have
       # checks += [("CRUISE_PARAMS", 50)]       #2018.09.02 DV comment out don't have

    if CP.carFingerprint in (CAR.ACCORD, CAR.ACCORD_15):
        signals += [("DRIVERS_DOOR_OPEN", "SCM_FEEDBACK", 1)]

    elif CP.carFingerprint == CAR.SOUL:
        signals += [("DOOR_OPEN_FL", "SCM_FEEDBACK", 1)] #2018.09.02 DV add CAR.SOUL for door status

      #else: (2018.09.02 DV comment out other
       # signals += [("DOOR_OPEN_FL", "SCM_FEEDBACK", 1), #2018.09.02 DV B0 1608 Door Front
        #            ("DOOR_OPEN_FR", "SCM_FEEDBACK", 1), #2018.09.02 DV B0 1680 passenger front door
                    #("DOOR_OPEN_RL", "DOORS_STATUS", 1), #2018.09.02 don't have rear door
                    #("DOOR_OPEN_RR", "DOORS_STATUS", 1), #2018.09.1 don't have rear door
           #         ("WHEELS_MOVING", "STANDSTILL", 1)]
       # checks += [("DOORS_STATUS", 3)]

    if CP.carFingerprint == CAR.CIVIC:
        signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
                ("MAIN_ON", "SCM_FEEDBACK", 0),
                ("EPB_STATE", "EPB_STATUS", 0),
                ("BRAKE_HOLD_ACTIVE", "VSA_STATUS", 0)]
    elif CP.carFingerprint == CAR.ACURA_ILX:
        signals += [("CAR_GAS", "GAS_PEDAL_2", 0),
                ("MAIN_ON", "SCM_BUTTONS", 0)]
    elif CP.carFingerprint in (CAR.CRV, CAR.ACURA_RDX, CAR.PILOT_2019, CAR.RIDGELINE):
        signals += [("MAIN_ON", "SCM_BUTTONS", 0)]
    elif CP.carFingerprint == CAR.ODYSSEY:
        signals += [("MAIN_ON", "SCM_FEEDBACK", 0),
                ("EPB_STATE", "EPB_STATUS", 0),
                ("BRAKE_HOLD_ACTIVE", "VSA_STATUS", 0)]
        checks += [("EPB_STATUS", 50)]
    elif CP.carFingerprint == CAR.PILOT:
        signals += [("MAIN_ON", "SCM_BUTTONS", 0),
                ("CAR_GAS", "GAS_PEDAL_2", 0)]
    elif CP.carFingerprint == CAR.SOUL:  # 2018.09.02 DV Kia Soul UI 0x1A6 ADAS Cruise button
        signals += [("MAIN_ON", "SCM_BUTTONS", 0)]

        # add gas interceptor reading if we are using it
    if CP.enableGasInterceptor:

        signals.append(("THROTTLE_REPORT_operator_override", "THROTTLE_REPORT", 0)) #2018.09.02 DV add change for Kia soul
        checks.append(("THROTTLE_REPORT", 50)) #2018.09.02 DV add change for Kia soul

    return signals, checks


def get_can_parser(CP):
  signals, checks = get_can_signals(CP)
  return CANParser(DBC[CP.carFingerprint]['pt'], signals, checks, 0)


class CarState(object):
  def __init__(self, CP):
    self.CP = CP
    self.can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    #self.shifter_values = self.can_define.dv["GEARBOX"]["GEAR_SHIFTER"]
    #2018.09.02 DV change the value for Kia soul gear info, link in interface.py
    self.shifter_PARK = self.can_define.dv["TM_GEAR"]["TM_PARK"]
    self.shifter_REVERSE = self.can_define.dv["TM_GEAR"]["TM_REVERSE"]
    self.shifter_NEUTRAL = self.can_define.dv["TM_GEAR"]["TM_NEUTRAL"]
    self.shifter_DRIVE = self.can_define.dv["TM_GEAR"]["TM_DRIVE"]

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
    self.v_ego_kf = KF1D(x0=[[0.0], [0.0]],
                         A=[[1.0, dt], [0.0, 1.0]],
                         C=[[1.0, 0.0]],
                         K=[[0.12287673], [0.29666309]])
    self.v_ego = 0.0

  def update(self, cp):

    # copy can_valid
    self.can_valid = cp.can_valid

    # car params
    v_weight_v = [0., 1.]  # don't trust smooth speed at low values to avoid premature zero snapping
    v_weight_bp = [1., 6.]   # smooth blending, below ~0.6m/s the smooth speed snaps to zero

    # update prevs, update must run once per loop
    self.prev_cruise_buttons = self.cruise_buttons
    self.prev_cruise_setting = self.cruise_setting
    self.prev_blinker_on = self.blinker_on

    self.prev_left_blinker_on = self.left_blinker_on
    self.prev_right_blinker_on = self.right_blinker_on

    # ******************* parse out can *******************

    if self.CP.carFingerprint in (CAR.ACCORD, CAR.ACCORD_15): # TODO: find wheels moving bit in dbc
      self.standstill = cp.vl["ENGINE_DATA"]['XMISSION_SPEED'] < 0.1
      self.door_all_closed = not cp.vl["SCM_FEEDBACK"]['DRIVERS_DOOR_OPEN']
    elif self.CP.carFingerprint in (CAR.SOUL): # 2018.09.02 DV add standstill for car not moving
      self.standstill = cp.vl["TM_DATA"]['VS_TCU'] < 0.1
      self.door_all_closed = not cp.vl["SCM_FEEDBACK"]['DOOR_OPEN_FL']

    else:
      self.standstill = not cp.vl["STANDSTILL"]['WHEELS_MOVING']
      self.door_all_closed = not any([cp.vl["DOORS_STATUS"]['DOOR_OPEN_FL'], cp.vl["DOORS_STATUS"]['DOOR_OPEN_FR'],
                                      cp.vl["DOORS_STATUS"]['DOOR_OPEN_RL'], cp.vl["DOORS_STATUS"]['DOOR_OPEN_RR']])
    self.seatbelt = not cp.vl["SEATBELT_STATUS"]['SEATBELT_DRIVER_LAMP'] and cp.vl["SEATBELT_STATUS"]['SEATBELT_DRIVER_LATCHED']

    if self.CP.carFingerprint in (CAR.SOUL):  #2018.09.02 separate fingerprint for kia soul
        self.steer_error = cp.vl["STEERING_REPORT"]['STEERING_REPORT_operator_override'] != 0
        self.steer_not_allowed = cp.vl["STEERING_REPORT"]['STEERING_REPORT_operator_override'] != 0
        self.steer_warning = cp.vl["STEERING_REPORT"]['STEERING_REPORT_operator_override'] != 0
        self.brake_error = cp.vl["BRAKE_REPORT"]['BRAKE_REPORT_dtcs']
        self.esp_disabled = cp.vl["VSA_STATUS"]['ESP_DISABLED']
    else:
      # 2 = temporary; 3 = TBD; 4 = temporary, hit a bump; 5 = (permanent); 6 = temporary; 7 = (permanent)
      # TODO: Use values from DBC to parse this field
      self.steer_error = cp.vl["STEER_STATUS"]['STEER_STATUS'] not in [0, 2, 3, 4, 6]
      self.steer_not_allowed = cp.vl["STEER_STATUS"]['STEER_STATUS'] != 0
      self.steer_warning = cp.vl["STEER_STATUS"]['STEER_STATUS'] not in [0, 3]   # 3 is low speed lockout, not worth a warning
      self.brake_error = cp.vl["STANDSTILL"]['BRAKE_ERROR_1'] or cp.vl["STANDSTILL"]['BRAKE_ERROR_2']
      self.esp_disabled = cp.vl["VSA_STATUS"]['ESP_DISABLED']

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
      self.v_ego_x = [[speed], [0.0]]

    self.v_ego_raw = speed
    v_ego_x = self.v_ego_kf.update(speed)
    self.v_ego = float(v_ego_x[0])
    self.a_ego = float(v_ego_x[1])

    # this is a hack for the interceptor. This is now only used in the simulation
    # TODO: Replace tests by toyota so this can go away
    if self.CP.enableGasInterceptor:
      self.user_gas = cp.vl["THROTTLE_REPORT"]['THROTTLE_REPORT_operator_override'] #2018.09.02 change for Kia soul when gas being press
      self.user_gas_pressed = self.user_gas > 0 # this works because interceptor read < 0 when pedal position is 0. Once calibrated, this will change

    self.gear = 0 if self.CP.carFingerprint == CAR.CIVIC else cp.vl["GEARBOX"]['GEAR']
    self.angle_steers = cp.vl["STEERING_SENSORS"]['STEER_ANGLE']
    self.angle_steers_rate = cp.vl["STEERING_SENSORS"]['STEER_ANGLE_RATE']

    self.cruise_setting = cp.vl["SCM_BUTTONS"]['CRUISE_SETTING']
    self.cruise_buttons = cp.vl["SCM_BUTTONS"]['CRUISE_BUTTONS']

    self.blinker_on = cp.vl["SCM_FEEDBACK"]['LEFT_BLINKER'] or cp.vl["SCM_FEEDBACK"]['RIGHT_BLINKER']
    self.left_blinker_on = cp.vl["SCM_FEEDBACK"]['LEFT_BLINKER']
    self.right_blinker_on = cp.vl["SCM_FEEDBACK"]['RIGHT_BLINKER']

    if self.CP.carFingerprint in (CAR.CIVIC, CAR.ODYSSEY, CAR.CRV_5G, CAR.ACCORD, CAR.ACCORD_15, CAR.CIVIC_HATCH):
      self.park_brake = cp.vl["EPB_STATUS"]['EPB_STATE'] != 0
      self.brake_hold = cp.vl["VSA_STATUS"]['BRAKE_HOLD_ACTIVE']
      self.main_on = cp.vl["SCM_FEEDBACK"]['MAIN_ON']
    else:
      self.park_brake = 0  # TODO
      self.brake_hold = 0  # TODO
      self.main_on = cp.vl["SCM_BUTTONS"]['MAIN_ON']

    #can_gear_shifter = int(cp.vl["GEARBOX"]['GEAR_SHIFTER'])
    #self.gear_shifter = parse_gear_shifter(can_gear_shifter, self.shifter_values)
    #2018.09.02 DV change gear shifter info for kia soul
    self.shifter_PARK = self.can_define.dv["TM_GEAR"]["TM_PARK"]
    self.shifter_REVERSE = self.can_define.dv["TM_GEAR"]["TM_REVERSE"]
    self.shifter_NEUTRAL = self.can_define.dv["TM_GEAR"]["TM_NEUTRAL"]
    self.shifter_DRIVE = self.can_define.dv["TM_GEAR"]["TM_DRIVE"]

    self.pedal_gas = cp.vl["ENG_INFO"]['PEDAL_GAS'] #2018.09.02 DV change for pedal gas
    # crv doesn't include cruise control
    if self.CP.carFingerprint in (CAR.SOUL):  #2018.09.02 DV change for Kia soul
      self.car_gas = self.pedal_gas
    else:
      self.car_gas = cp.vl["ENG_INFO"]['PEDAL_GAS'] #2018.09.02 DV cruise control gas not available change to pedal gas

    #self.steer_torque_driver = cp.vl["STEER_STATUS"]['STEER_TORQUE_SENSOR'] 2018.09.02 comment out to use steering operator override
    self.steer_torque_driver = cp.vl["STEERING_REPORT"]['STEERING_REPORT_operator_override']
    self.steer_override = abs(self.steer_torque_driver) > STEER_THRESHOLD[self.CP.carFingerprint] #threshold set in values.py

    self.brake_switch = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] not in [0, 3] #2018.09.02 DV "2"value is brake switch ON

    if self.CP.radarOffCan: #2018.09.02 this for car with bosch radar
      self.stopped = cp.vl["ACC_HUD"]['CRUISE_SPEED'] == 252.
      self.cruise_speed_offset = calc_cruise_offset(0, self.v_ego)
      if self.CP.carFingerprint == CAR.CIVIC_HATCH:
        self.brake_switch = cp.vl["POWERTRAIN_DATA"]['BRAKE_SWITCH']
        self.brake_pressed = cp.vl["POWERTRAIN_DATA"]['BRAKE_PRESSED'] or \
                          (self.brake_switch and self.brake_switch_prev and \
                          cp.ts["POWERTRAIN_DATA"]['BRAKE_SWITCH'] != self.brake_switch_ts)
        self.brake_switch_prev = self.brake_switch
        self.brake_switch_ts = cp.ts["POWERTRAIN_DATA"]['BRAKE_SWITCH']
      else:
        self.brake_pressed = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] not in [0, 3]  #2018.09.02 change for Kia soul
      # On set, cruise set speed pulses between 254~255 and the set speed prev is set to avoid this.
      self.v_cruise_pcm = self.v_cruise_pcm_prev if cp.vl["ACC_HUD"]['CRUISE_SPEED'] > 160.0 else cp.vl["ACC_HUD"]['CRUISE_SPEED']
      self.v_cruise_pcm_prev = self.v_cruise_pcm

      #  change for Kia soul #
    elif self.CP.carFingerprint in (CAR.SOUL): # 2018.09.02 DV
        self.brake_switch = cp.vl["ENG_INFO"]['BRAKE_PRESSED'] not in [0, 3] #2018.09.02 DV "2"value is brake switch ON
        self.stopped = cp.vl["ACC_HUD"]['CRUISE_SPEED'] == 252.
        self.cruise_speed_offset = calc_cruise_offset(0, self.v_ego)  #not using cruise control same as bosch honda car
        # brake switch has shown some single time step noise, so only considered when
        # switch is on for at least 2 consecutive CAN samples
        self.brake_pressed = cp.vl[["ENG_INFO"]['BRAKE_PRESSED'] not in [0, 3]] or \
                             (self.brake_switch and self.brake_switch_prev and \
                              cp.ts[["ENG_INFO"]['BRAKE_PRESSED'] not in [0, 3]] != self.brake_switch_ts)
        self.brake_switch_prev = self.brake_switch
        self.brake_switch_ts = cp.ts["ENG_INFO"]['BRAKE_PRESSED']
      # end here change for this section for kia for brake #

    else:
      self.brake_switch = cp.vl["POWERTRAIN_DATA"]['BRAKE_SWITCH']
      self.cruise_speed_offset = calc_cruise_offset(cp.vl["CRUISE_PARAMS"]['CRUISE_SPEED_OFFSET'], self.v_ego)
      self.v_cruise_pcm = cp.vl["CRUISE"]['CRUISE_SPEED_PCM']
      # brake switch has shown some single time step noise, so only considered when
      # switch is on for at least 2 consecutive CAN samples
      self.brake_pressed = cp.vl["POWERTRAIN_DATA"]['BRAKE_PRESSED'] or \
                         (self.brake_switch and self.brake_switch_prev and \
                         cp.ts["POWERTRAIN_DATA"]['BRAKE_SWITCH'] != self.brake_switch_ts)
      self.brake_switch_prev = self.brake_switch
      self.brake_switch_ts = cp.ts["POWERTRAIN_DATA"]['BRAKE_SWITCH']







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
      self.carFingerprint = "HONDA CIVIC 2016 TOURING"
      self.enableGasInterceptor = 0
  CP = CarParams()
  CS = CarState(CP)

  # while 1:
  #   CS.update()
  #   time.sleep(0.01)
