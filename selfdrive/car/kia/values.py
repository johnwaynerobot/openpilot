from selfdrive.car import dbc_dict

# Car button codes
class CruiseButtons:
  RES_ACCEL   = 4
  DECEL_SET   = 3
  CANCEL      = 2
  MAIN        = 1


#car chimes: enumeration from dbc file. Chimes are for alerts and warnings
class CM:
  MUTE = 0
  SINGLE = 3
  DOUBLE = 4
  REPEATED = 1
  CONTINUOUS = 2


#car beepss: enumeration from dbc file. Beeps are for activ and deactiv
class BP:
  MUTE = 0
  SINGLE = 3
  TRIPLE = 2
  REPEATED = 1

class AH:
  #[alert_idx, value]
  # See dbc files for info on values"
  NONE           = [0, 0]
  FCW            = [1, 1]  #2018.09.02 have
  STEER          = [2, 1]   #2018.09.02 TODO check alert values
  BRAKE_PRESSED  = [3, 10]  #2018.09.02 value does match
  GEAR_NOT_D     = [4, 6]  #2018.09.02 TODO this come from BO_927 0x39F from adas need to add in DBC
  SEATBELT       = [5, 5]  #2018.09.02 not find where it use
  SPEED_TOO_HIGH = [6, 8]  #2018.09.02 TODO this come from BO_927 0x39F from adas need to add in DBC


class CAR:
  SOUL = "KIA SOUL TEST" #Test Spoof DV 2018.09.02
  DUMMY = "DUMMY TEST SPOOF" #2018.09.03 test spoof


FINGERPRINTS = {
  # Soul Vehicle with Steering Gas and Throttle in disable state
  CAR.SOUL: [544L: 8, 131L: 8, 357L: 8, 809L: 8, 688L: 8, 790L: 8, 185L: 8
  }],
  CAR.DUMMY: [{
    57: 3, 145: 8, 228: 5, 304: 8, 316: 8, 342: 6, 344: 8, 380: 8, 398: 3, 399: 7, 419: 8, 420: 8, 422: 8, 428: 8, 432: 7, 464: 8, 476: 4, 490: 8, 506: 8, 512: 6, 513: 6, 542: 7, 545: 4, 597: 8, 660: 8, 773: 7, 777: 8, 780: 8, 800: 8, 804: 8, 808: 8, 819: 7, 821: 5, 829: 5, 882: 2, 884: 7, 887: 8, 888: 8, 892: 8, 923: 2, 929: 4, 983: 8, 985: 3, 1024: 5, 1027: 5, 1029: 8, 1030: 5, 1034: 5, 1036: 8, 1039: 8, 1057: 5, 1064: 7, 1108: 8, 1365: 5,
  }],
}


DBC = {
  CAR.SOUL: dbc_dict('kia_soul_2016_pt_generated.dbc', 'acura_ilx_2016_nidec'), # 2018.09.03 DV change to generator.py dbc
}


STEER_THRESHOLD = {
  CAR.SOUL: 0, #2018.09.02 D.V. steering override from OSCC car in carstate.py when 1 is override
}

SPEED_FACTOR = {
  CAR.SOUL: 1., #2018.09.02 DV add Soul vehicle for test
}

# TODO: get these from dbc file
HONDA_BOSCH = [CAR.DUMMY]   #2018.09.03 Define car dummy for test
