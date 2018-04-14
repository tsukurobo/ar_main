#ifndef   CONF_HPP
#define   CONF_HPP

enum ServoStatus {
  SERVO_WAIT = 1,
  SERVO_PREPARE = 2,
  SERVO_TZ1SHOT = 3,
  SERVO_TZ2SHOT = 4,
  SERVO_TZ3SHOT = 5,
  SERVO_FREE = 6,
  SERVO_PICK = 7,
  SERVO_CLOSE = 8,
};

enum ArmStatus{
	ARM_PREPARE = 2,
	ARM_PRE_FRONT= 3,
	ARM_PRE_BACK= 13,
	ARM_RETURN= 5,
	ARM_PASS= 6,
	ARM_INIT= 8,
};



#endif  //CONF_HPP
