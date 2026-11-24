#ifndef GO1_SERIAL_HPP
#define GO1_SERIAL_HPP
#include "go1_motor.h"
#include <string>
namespace go1 {
using MotorCmd = Go1MotorCmd;
using MotorData = Go1MotorData;
constexpr unsigned BRAKE = GO1_MODE_STOP, FOC = GO1_MODE_FOC;
inline float queryGearRatio() { return go1_query_gear_ratio(); }
/* Blocking, one outstanding request per port. Throws on I/O or feedback timeout. */
class SerialPort {
public:
 explicit SerialPort(const std::string& path, unsigned baud=GO1_BAUD);
 ~SerialPort();
 SerialPort(const SerialPort&)=delete; SerialPort& operator=(const SerialPort&)=delete;
 bool sendRecv(const MotorCmd* cmd, MotorData* data, int timeout_ms=50);
private: int fd_;
};
}
#endif
