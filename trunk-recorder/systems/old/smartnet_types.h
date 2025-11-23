// datatypes for smartnet decoder

struct smartnet_packet {
  unsigned int address;
  bool groupflag;
  unsigned int command;
  unsigned int crc;
};
