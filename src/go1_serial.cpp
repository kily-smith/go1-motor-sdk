#include "go1_serial.hpp"
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <stdexcept>
#include <system_error>
namespace go1 {
static void fail(const char* msg) { throw std::system_error(errno,std::generic_category(),msg); }
SerialPort::SerialPort(const std::string& path,unsigned baud):fd_(-1) {
 fd_=open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC); if(fd_<0) fail("open serial");
 try { if(ioctl(fd_,TIOCEXCL)<0) fail("exclusive serial"); struct termios2 t{};if(ioctl(fd_,TCGETS2,&t)<0) fail("TCGETS2");
 t.c_iflag=0;t.c_oflag=0;t.c_lflag=0;t.c_cflag=(t.c_cflag & ~(CBAUD|CSIZE|PARENB|CSTOPB|CRTSCTS))|BOTHER|CS8|CREAD|CLOCAL;
 t.c_ispeed=baud;t.c_ospeed=baud;t.c_cc[VMIN]=0;t.c_cc[VTIME]=0;if(ioctl(fd_,TCSETS2,&t)<0) fail("TCSETS2");
 } catch(...) {close(fd_);fd_=-1;throw;}
}
SerialPort::~SerialPort(){if(fd_>=0) close(fd_);}
bool SerialPort::sendRecv(const MotorCmd* cmd,MotorData* data,int timeout_ms) {
 if(!cmd||!data||timeout_ms<=0) throw std::invalid_argument("cmd/data/timeout");
 uint8_t tx[GO1_TX_SIZE]; if(go1_pack(cmd,tx)) throw std::invalid_argument("invalid motor command"); *data=MotorData{};
 /* Clear already-buffered responses before request; responses from a prior in-flight request cannot be distinguished on wire. */
 if(ioctl(fd_,TCFLSH,0)<0) fail("flush input");
 auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
 size_t sent=0;Go1Parser parser{};go1_parser_init(&parser);
 while(std::chrono::steady_clock::now()<end) {
  auto left=std::chrono::duration_cast<std::chrono::milliseconds>(end-std::chrono::steady_clock::now()).count()+1;
  struct pollfd p{fd_,static_cast<short>(sent<sizeof(tx)?POLLOUT:POLLIN),0};int r=poll(&p,1,static_cast<int>(left));
  if(r<0){if(errno==EINTR)continue;fail("poll");} if(r==0) break;
  if(p.revents&(POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("serial disconnected");
  if(sent<sizeof(tx) && (p.revents&POLLOUT)){ssize_t n=write(fd_,tx+sent,sizeof(tx)-sent);if(n>0)sent+=(size_t)n;else if(n<0&&errno!=EAGAIN&&errno!=EINTR)fail("write");}
  else if(p.revents&POLLIN){uint8_t buf[256];ssize_t n=read(fd_,buf,sizeof(buf));if(n<0&&errno!=EAGAIN&&errno!=EINTR)fail("read");for(ssize_t i=0;i<n;i++)if(go1_parser_feed(&parser,buf[i],cmd->id,data))return true;}
 }
 throw std::runtime_error("motor feedback timeout");
}
}
