#ifndef   CONF_HPP
#define   CONF_HPP

enum Status {
  PREPARE = 0,
  STARTTOPASS1 = 1,
  WAITPASS1 = 11,
  PASS1 = 21,
  PASS1TOSHOT1 = 31,
  SHOT1 = 41,
  SHOT1TOPASS2 = 51,
  WAITPASS2 = 61,
  PASS2 = 71,
  PASS2TOSHOT2 = 81,
  SHOT2 = 91,
  SHOT2TOPASS3 = 101,
  WAITPASS3 = 111,
  PASS3 = 121,
  PASS3TOSHOT3 = 131,
  SHOT3 = 141,
  // end: after care.
  SHOT3TOPASS4 = 151,    
};

enum Tasks {
  CORRECTSPACE_BEGIN = 1,
  CORRECTSPACE_END = 2,
  ODOMRUN_BEGIN = 11,
  ODOMRUN_END = 12,  
  ODOMRUN_LOGGING_BEGIN = 21,
  ODOMRUN_LOGGING_END = 22,
  WAITPASS_BEGIN = 31,
  WAITPASS_END = 32,
  PASS1_BEGIN = 41,
  PASS1_END = 42,
  PASS2_BEGIN = 51,
  PASS2_END = 52,
  PASS3_BEGIN = 61,
  PASS3_END = 62,
  SHOT1_BEGIN = 71,
  SHOT1_END = 72,
  SHOT2_BEGIN = 81,
  SHOT2_END = 82,
  SHOT3_BEGIN = 91,
  SHOT3_END = 92,
  ODOMRUN_BEGIN_STARTTOPASS1 = 111,
  ODOMRUN_BEGIN_PASS1TOSHOT1 = 112,
  ODOMRUN_BEGIN_SHOT1TOPASS2 = 113,
  ODOMRUN_BEGIN_PASS2TOSHOT2 = 114,
  ODOMRUN_BEGIN_SHOT2TOPASS3 = 115,
  ODOMRUN_BEGIN_PASS3TOSHOT3 = 116,
  ODOMRUN_BEGIN_SHOT3TOPASS4 = 117,  
};



#endif  //CONF_HPP
