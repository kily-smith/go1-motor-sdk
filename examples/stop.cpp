#include "go1_serial.hpp"
#include <cstdlib>
#include <iostream>
int main(int argc,char **argv) {
 if(argc!=3){std::cerr<<"usage: go1_stop /dev/ttyUSB0 ID\n";return 2;}
 char *end=nullptr;long id=std::strtol(argv[2],&end,10);if(*end||id<0||id>14)return 2;
 try {go1::SerialPort serial(argv[1]);go1::MotorCmd cmd{};go1::MotorData data{};cmd.id=static_cast<uint8_t>(id);cmd.mode=go1::BRAKE;
 serial.sendRecv(&cmd,&data);std::cout<<"ID="<<int(data.motor_id)<<" mode="<<int(data.mode)<<" q="<<data.q<<" temp="<<int(data.temp)<<" error="<<int(data.merror)<<"\n";return data.mode==go1::BRAKE?0:1;
 } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
