import time
from collections import defaultdict
import numbers

from selfdrive.can.libdbc_py import libdbc, ffi

class CANParser(object):
  def __init__(self, dbc_name, signals, checks=[], bus=0, sendcan=False, tcp_addr="127.0.0.1"):
    self.can_valid = True
    self.vl = defaultdict(dict)
    self.ts = defaultdict(dict)

    self.dbc_name = dbc_name
    self.dbc = libdbc.dbc_lookup(dbc_name)
    self.msg_name_to_addres = {}
    self.address_to_msg_name = {}

    #2018.09.06 3:56PMEST add print for test

    print("parser.py section dbc_name")
    print(self.dbc_name)
    print("dbc_lookup")
    print(self.dbc)
    print("parser.py self.dbc[0]")
    print(self.dbc[0])
    print("self.dbc[0].msgs[1]")
    print(self.dbc[0].msgs[1])
    print("self.dbc[0].msgs[0]")
    print(self.dbc[0].msgs[0])
    num_msgs = self.dbc[0].num_msgs
    print("parser.py num_msgs")
    print(num_msgs)
    for i in range(num_msgs):
      msg = self.dbc[0].msgs[i]
      #print("parser.py msgs[i]")
      #print(msg[i])
      print("parser.py msg")
      print(msg)

      name = ffi.string(msg.name)
      print("parser.py ffi.string(msg.name)")
      print(msg.name)
      print("parser.py name")
      print(name)
      print("parser.py address")
      address = msg.address
      print(address)


      self.msg_name_to_addres[name] = address
      self.address_to_msg_name[address] = name

      print("msg_name_to_adress[name]")
      print(self.msg_name_to_addres)
      print("address to msg name")
      print(self.address_to_msg_name)

    #len return the number of entries in dictionary
    # Convert message names into addresses
    for i in range(len(signals)):
      s = signals[i]
      if not isinstance(s[1], numbers.Number):
        s = (s[0], self.msg_name_to_addres[s[1]], s[2])
        signals[i] = s

    print("parsery.py range")
    print(range)
    print("parser.py signals")
    print(signals)
    print("parser.py len(signals)")
    print(len(signal))
    print("parser.py s")
    print(s)
    print("parsery.py numbers.number")
    print(numbers.Number)

    for i in range(len(checks)):
      c = checks[i]
      if not isinstance(c[0], numbers.Number):
        c = (self.msg_name_to_addres[c[0]], c[1])
        checks[i] = c

    print("parser.py checks")
    print(checks)
    print("parser.py len(check)")
    print(len(checks))
    print("parser.py c")
    print(c)

    sig_names = dict((name, ffi.new("char[]", name)) for name, _, _ in signals)

    print("parser.py sig_names")
    print(sig_name)
    print("parsery.py ffi.new(char ,name)")
    print(ffi.new("char[]", name))
    print("dict (name, ffi.new)")
    print(dict)
    signal_options_c = ffi.new("SignalParseOptions[]", [
      {
        'address': sig_address,
        'name': sig_names[sig_name],
        'default_value': sig_default,
      } for sig_name, sig_address, sig_default in signals])

    print("parser.py signal_optionsc")
    print(signal_options_c)
    print("parser.py sig_address")
    print(sig_address)
    print("parser.py sig_name")
    print(sig_name)
   # print("parser.py sig_names[sig_name]")
    #print(sig_names[sig_name])
    print("parser.py sig_default")


    message_options = dict((address, 0) for _, address, _ in signals)
    message_options.update(dict(checks))

    print("parser.py message_options")
    print(message_options)
    print("parser.py dict((address,0) ")
    print(dict((address, 0)))

    message_options_c = ffi.new("MessageParseOptions[]", [
      {
        'address': msg_address,
        'check_frequency': freq,
      } for msg_address, freq in message_options.iteritems()])

    print("parser.py message_options_c")
    print(message_options_c)
    print("parser.py msg_address under message optionc")
    print(msg_address)
    print("parser.py freq")
    print(freq)
    print("message_options.iteritems")
    print(message_options.iteritem())

    self.can = libdbc.can_init(bus, dbc_name, len(message_options_c), message_options_c,
                               len(signal_options_c), signal_options_c, sendcan, tcp_addr)



    print("parser.py self.can")
    print(self.can)
    print("parser.py bus")
    print(bus)
    print("parser.py len(message_options_c)")
    print(len(message_options_c))
    print("parser.py signal_options_c")
    print(signal_options_c)
    print("tcp_addr")
    print(tcp_addr)
    print("parser.py sendcan")
    print(sendcan)


    self.p_can_valid = ffi.new("bool*")

    print("parser.py self.p_can_valid")
    print(p.can_valid)

    value_count = libdbc.can_query(self.can, 0, self.p_can_valid, 0, ffi.NULL)

    print("parser.py value_count")
    print(value_count)
    self.can_values = ffi.new("SignalValue[%d]" % value_count)
   # print("parser.py % value_count")
   # print(% value_count)
    print("parser.py self.can_values")
    print(self.can_values)
    self.update_vl(0)
    # print "==="
    print("parser.py self.update_vl(0)")

  def update_vl(self, sec):

    can_values_len = libdbc.can_query(self.can, sec, self.p_can_valid, len(self.can_values), self.can_values)

    print("parser.py len(self.can_values)")
    print(len(self.can_values))
    print("parser.py can_values_len")
    print(can_values_len)
    print("parser.py sec")
    print(sec)
    assert can_values_len <= len(self.can_values)


    self.can_valid = self.p_can_valid[0]
    print("parser.py self.can_valid")
    print(self.can_valid)
    print("parser.py self.p_can_valid[0]")
    print(self.p_can_valid[0])
    print("parser.py self.p_can_valid[1]")
    print(self.p_can_valid[1])

    # print can_values_len
    ret = set()
    print("parser.py set()")
    print(set())
    print("parser.py ret")
    print(ret)
    #for i in xrange(can_values_len):  (#2018.09.06 3:19PM remove typo xrange)
    for i in range(can_values_len):
      print("parser.py range(can_values_len)")
      print(range(can_values_len))
      cv = self.can_values[i]
      print("parser.py self.can_values[i]")
      print(self.can_values[i])
      print("parser.py cv")
      print(cv)
      address = cv.address
      print("parser.py cv.address")
      print(cv.address)
      print("parser.py address")
      print(address)
      # print hex(cv.address), ffi.string(cv.name)
      print("parser.py ffi.string(cv.name")
      print(ffi.string(cv.name))
      name = ffi.string(cv.name)
      print("parser.py name under ffi.string(cv.name)")
      self.vl[address][name] = cv.value
      print("parser.py cv.value")
      print(cv.value)
      print("parser.py self.vl[address][name]")
      print(self.vl[address][name])
      self.ts[address][name] = cv.ts
      print("parser.py cv.ts")
      print(cv.ts)
      print("parser.py self.ts[address][name]")
      print(self.ts[address][name])

      sig_name = self.address_to_msg_name[address]
      print("parser.py under sign_name")
      print(self.address_to_msg_name[address])
      print("parser.py sig_name")
      print(sig_name)
      self.vl[sig_name][name] = cv.value
      print("parser.py cv.value for self.vl[sig_name][name]")
      print(self.vl[sig_name][name])
      self.ts[sig_name][name] = cv.ts
      print("parser.py cv.ts")
      print(cv.ts)
      print("parser.py self.ts[sig_name][name]")
      print(self.ts[sig_name][name])
      print("parser.py ret.add(address")
      ret.add(address)
      print(ret.add(address))
      print("parser.py end of canparser ret)")
      print(ret)
    return ret

  def update(self, sec, wait):
    libdbc.can_update(self.can, sec, wait)
    print("parser.py def update")
    print(self.can)
    print("parser.py sec")
    print(sec)
    print("parser.py wait")
    print(wait)
    print("parser.py libdbc.can_update")
    print(libdbc.can_update)
    print("parser.py self.update(sec)")
    print(self.update_vl(sec))
    return self.update_vl(sec)


class CANDefine(object):
  def __init__(self, dbc_name):
    self.dv = defaultdict(dict)
    self.dbc_name = dbc_name
    self.dbc = libdbc.dbc_lookup(dbc_name)

    num_vals = self.dbc[0].num_vals

    self.address_to_msg_name = {}
    num_msgs = self.dbc[0].num_msgs
    for i in range(num_msgs):
      msg = self.dbc[0].msgs[i]
      name = ffi.string(msg.name)
      address = msg.address
      self.address_to_msg_name[address] = name

    for i in range(num_vals):
      val = self.dbc[0].vals[i]

      sgname = ffi.string(val.name)
      address = val.address
      def_val = ffi.string(val.def_val)

      #separate definition/value pairs
      def_val = def_val.split()
      values = [int(v) for v in def_val[::2]]
      defs = def_val[1::2]

      if address not in self.dv:
        self.dv[address] = {}
        msgname = self.address_to_msg_name[address]
        self.dv[msgname] = {}

      # two ways to lookup: address or msg name
      self.dv[address][sgname] = {v: d for v, d in zip(values, defs)} #build dict
      self.dv[msgname][sgname] = self.dv[address][sgname]


if __name__ == "__main__":
  from common.realtime import sec_since_boot

  radar_messages = range(0x430, 0x43A) + range(0x440, 0x446)
  signals = zip(['LONG_DIST'] * 16 + ['NEW_TRACK'] * 16 + ['LAT_DIST'] * 16 +
                ['REL_SPEED'] * 16, radar_messages * 4,
                 [255] * 16 + [1] * 16 + [0] * 16 + [0] * 16)
  checks = zip(radar_messages, [20]*16)

  cp = CANParser("acura_ilx_2016_nidec", signals, checks, 1)


  # signals = [
  #   ("XMISSION_SPEED", 0x158, 0), #sig_name, sig_address, default
  #   ("WHEEL_SPEED_FL", 0x1d0, 0),
  #   ("WHEEL_SPEED_FR", 0x1d0, 0),
  #   ("WHEEL_SPEED_RL", 0x1d0, 0),
  #   ("STEER_ANGLE", 0x14a, 0),
  #   ("STEER_TORQUE_SENSOR", 0x18f, 0),
  #   ("GEAR", 0x191, 0),
  #   ("WHEELS_MOVING", 0x1b0, 1),
  #   ("DOOR_OPEN_FL", 0x405, 1),
  #   ("DOOR_OPEN_FR", 0x405, 1),
  #   ("DOOR_OPEN_RL", 0x405, 1),
  #   ("DOOR_OPEN_RR", 0x405, 1),
  #   ("CRUISE_SPEED_PCM", 0x324, 0),
  #   ("SEATBELT_DRIVER_LAMP", 0x305, 1),
  #   ("SEATBELT_DRIVER_LATCHED", 0x305, 0),
  #   ("BRAKE_PRESSED", 0x17c, 0),
  #   ("CAR_GAS", 0x130, 0),
  #   ("CRUISE_BUTTONS", 0x296, 0),
  #   ("ESP_DISABLED", 0x1a4, 1),
  #   ("HUD_LEAD", 0x30c, 0),
  #   ("USER_BRAKE", 0x1a4, 0),
  #   ("STEER_STATUS", 0x18f, 5),
  #   ("WHEEL_SPEED_RR", 0x1d0, 0),
  #   ("BRAKE_ERROR_1", 0x1b0, 1),
  #   ("BRAKE_ERROR_2", 0x1b0, 1),
  #   ("GEAR_SHIFTER", 0x191, 0),
  #   ("MAIN_ON", 0x326, 0),
  #   ("ACC_STATUS", 0x17c, 0),
  #   ("PEDAL_GAS", 0x17c, 0),
  #   ("CRUISE_SETTING", 0x296, 0),
  #   ("LEFT_BLINKER", 0x326, 0),
  #   ("RIGHT_BLINKER", 0x326, 0),
  #   ("COUNTER", 0x324, 0),
  #   ("ENGINE_RPM", 0x17C, 0)
  # ]
  # checks = [
  #   (0x14a, 100), # address, frequency
  #   (0x158, 100),
  #   (0x17c, 100),
  #   (0x191, 100),
  #   (0x1a4, 50),
  #   (0x326, 10),
  #   (0x1b0, 50),
  #   (0x1d0, 50),
  #   (0x305, 10),
  #   (0x324, 10),
  #   (0x405, 3),
  # ]

  # cp = CANParser("honda_civic_touring_2016_can_generated", signals, checks, 0)


 # signals = [
    # sig_name, sig_address, default
   # ("GEAR", 956, 0x20),
   # ("BRAKE_PRESSED", 548, 0),
    #("GAS_PEDAL", 705, 0),

    #("WHEEL_SPEED_FL", 170, 0),
    #("WHEEL_SPEED_FR", 170, 0),
    #("WHEEL_SPEED_RL", 170, 0),
    #("WHEEL_SPEED_RR", 170, 0),
    #("DOOR_OPEN_FL", 1568, 1),
    #("DOOR_OPEN_FR", 1568, 1),
    #("DOOR_OPEN_RL", 1568, 1),
    #("DOOR_OPEN_RR", 1568, 1),
    #("SEATBELT_DRIVER_UNLATCHED", 1568, 1),
    #("TC_DISABLED", 951, 1),
    #("STEER_ANGLE", 37, 0),
    #("STEER_FRACTION", 37, 0),
    #("STEER_RATE", 37, 0),
    #("GAS_RELEASED", 466, 0),
    #("CRUISE_STATE", 466, 0),
    #("MAIN_ON", 467, 0),
    #("SET_SPEED", 467, 0),
   # ("STEER_TORQUE_DRIVER", 608, 0),
    #("STEER_TORQUE_EPS", 608, 0),
    #("TURN_SIGNALS", 1556, 3),   # 3 is no blinkers
    #("LKA_STATE", 610, 0),
  #]
  #checks = [
   # (548, 40),
    #(705, 33),

   # (170, 80),
   # (37, 80),
   # (466, 33),
   # (608, 50),
 # ]

  #cp = CANParser("toyota_rav4_2017_pt_generated", signals, checks, 0)

  # print cp.vl

  while True:
    cp.update(int(sec_since_boot()*1e9), True)
    # print cp.vl
    print(cp.ts)
    print(cp.can_valid)
    time.sleep(0.01)
