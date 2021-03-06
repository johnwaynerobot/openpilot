import struct
from selfdrive.can.libdbc_py import libdbc, ffi


class CANPacker(object):
  def __init__(self, dbc_name):

    #2018.09.06 9:19PM Print dbc_name test
    #print("packer.py debug")
   # print(dbc_name)
    #print("packer.py packer check")
    self.packer = libdbc.canpack_init(dbc_name)
   # print("packer.py packer check")
   # print(self.packer)
    self.dbc = libdbc.dbc_lookup(dbc_name)
   # print("packer.py packer self.dbc lookup dbc_name")
   # print(self.dbc)

    self.sig_names = {}
   # print("packer.py sig_names")
    #print(self.sig_names)
    self.name_to_address_and_size = {}
    #print("packer.py names to address size")
    #print(self.name_to_address_and_size)

    num_msgs = self.dbc[0].num_msgs
    #print("packer.py num_msgs")
    #print(num_msgs)

    for i in range(num_msgs):
      msg = self.dbc[0].msgs[i]
      #print("packer.py .msg[i]")
      #print(msg[i])
      #print("packer.py self.dbc[0]")
     # print(self.dbc[0])
     # print("packer.py msg")
     # print(msg)

      name = ffi.string(msg.name)
      address = msg.address
      self.name_to_address_and_size[name] = (address, msg.size)
      self.name_to_address_and_size[address] = (address, msg.size)

  def pack(self, addr, values, counter):
    values_thing = []
    for name, value in values.iteritems():
      if name not in self.sig_names:
        self.sig_names[name] = ffi.new("char[]", name)

      values_thing.append({
        'name': self.sig_names[name],
        'value': value
      })

    print("definition pack in packer.py values_things")
    print(values_thing)
    values_c = ffi.new("SignalPackValue[]", values_thing)
    print("packer.py values_c under pack")
    print(values_c)


    return libdbc.canpack_pack(self.packer, addr, len(values_thing), values_c, counter)

 # print("packery.py libdbc.canpack_pack function")
  #print(addr)
  #print(len(values_thing))
  #print(values_c)
  #print(counter)

  def pack_bytes(self, addr, values, counter=-1):
    addr, size = self.name_to_address_and_size[addr]

    print("packer.py pack_bytes addr")
    print(addr)
    print("packer.py pack_bytes values")
    print(values)
    print("packer.py pack_bytes counter")
    print(counter)
    val = self.pack(addr, values, counter)
    print("packer.py pack_bytes val")
    print(val)
    r = struct.pack(">Q", val)
    print("packer.py pack_bytes struck pack")
    print(r)

    return addr, r[:size]

  #2018.09.28 1:47PMEST try bytes conversion in packer.py
  def pack_bytes2(self, addr, values, counter=-1):
    addr, size = self.name_to_address_and_size[addr]

    print("packer.py pack_bytes2 addr")
    print(addr)
    print("packer.py pack_bytes2 values")
    print(values)
    print("packer.py pack_bytes2 counter")
    print(counter)
    val = values
    print("packer.py pack_bytes2")
    print(val)
    r2 = struct.pack("=f", val)   # 2018.09.28 wont work because expect single float not full value
    print("packer.py pack_bytes2 r")
    print(r2)
    return addr, r2[:size]

  #2018.09.28 2:01PMEST add make_can_msg2 for test
  def make_can_msg2(self, addr, bus, values, counter=-1):
    addr, msg = self.pack_bytes2(addr, values, counter)
    return [addr, 0, msg, bus]

  def make_can_msg(self, addr, bus, values, counter=-1):
    addr, msg = self.pack_bytes(addr, values, counter)
    return [addr, 0, msg, bus]



   # print("packer.py make_can_msg")
   # print(addr)


# 2018.09.04 Kia Soul don't need alive counter and checksum, just reading
#if __name__ == "__main__":
  ## little endian test
#  cp = CANPacker("hyundai_santa_fe_2019_ccan")
 # s = cp.pack_bytes(0x340, {
 #   "CR_Lkas_StrToqReq": -0.06,
  #  #"CF_Lkas_FcwBasReq": 1,
  #  "CF_Lkas_MsgCount": 7,
   # "CF_Lkas_HbaSysState": 0,
    #"CF_Lkas_Chksum": 3,
 # })
 # s = cp.pack_bytes(0x340, {
#    "CF_Lkas_MsgCount": 1,
 # })
  # big endian test
  #cp = CANPacker("honda_civic_touring_2016_can_generated")
  #s = cp.pack_bytes(0xe4, {
  #  "STEER_TORQUE": -2,
  #})
  #print ([hex(ord(v)) for v in s[1]])
 # print(s[1].encode("hex"))
