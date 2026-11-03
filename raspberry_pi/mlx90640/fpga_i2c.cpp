// MLX90640 register transport through FPGA E13/E14, not Pi GPIO.
#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <process.h>
using SocketHandle=SOCKET;
using SocketLength=int;
static const SocketHandle invalid_socket=INVALID_SOCKET;
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/file.h>
#include <fcntl.h>
#include <cstdio>
using SocketHandle=int;
using SocketLength=socklen_t;
static const SocketHandle invalid_socket=-1;
#endif
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <cstdint>
#include "MLX90640_I2C_Driver.h"
static SocketHandle transport=invalid_socket;
static sockaddr_in peer;
#ifndef _WIN32
static int owner_lock=-1;
#endif
static uint32_t sequence;
static void put16(uint8_t *p,uint16_t v){p[0]=v>>8;p[1]=v;}
static void put32(uint8_t *p,uint32_t v){put16(p,v>>16);put16(p+2,v);}
static uint16_t get16(const uint8_t*p){return (uint16_t(p[0])<<8)|p[1];}
static uint32_t get32(const uint8_t*p){return (uint32_t(get16(p))<<16)|get16(p+2);}
void thermal_i2c_close(){
 if(transport!=invalid_socket){
#ifdef _WIN32
  closesocket(transport);
#else
  close(transport);
#endif
 }
 transport=invalid_socket;
#ifndef _WIN32
 if(owner_lock>=0)close(owner_lock);
 owner_lock=-1;
#endif
}
void thermal_i2c_pause(){
#ifdef _WIN32
 Sleep(3);
#else
 std::this_thread::sleep_for(std::chrono::milliseconds(3));
#endif
}
static int ensure_open(){
 if(transport!=invalid_socket)return 0;
#ifdef _WIN32
 static bool started=false;if(!started){WSADATA data;if(WSAStartup(MAKEWORD(2,2),&data))return -1;started=true;}
#endif
 transport=socket(AF_INET,SOCK_DGRAM,0);if(transport==invalid_socket)return -1;
#ifdef _WIN32
 DWORD timeout=120;
#else
 timeval timeout={0,120000};
#endif
 setsockopt(transport,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<const char*>(&timeout),sizeof(timeout));
 sockaddr_in local={};local.sin_family=AF_INET;local.sin_addr.s_addr=htonl(INADDR_ANY);
 if(bind(transport,reinterpret_cast<sockaddr*>(&local),sizeof(local))){thermal_i2c_close();return -1;}
 peer={};peer.sin_family=AF_INET;
 const char *ip=std::getenv("MLX_FPGA_IP"),*port=std::getenv("MLX_FPGA_PORT");
 if(inet_pton(AF_INET,ip?ip:"192.168.10.2",&peer.sin_addr)!=1){thermal_i2c_close();return -1;}
 peer.sin_port=htons(port?std::atoi(port):5001);
#ifndef _WIN32
 char lock_path[128];
 std::snprintf(lock_path,sizeof(lock_path),"/tmp/mlx90640-fpga-%u-%08x-%u.lock",unsigned(getuid()),unsigned(peer.sin_addr.s_addr),unsigned(ntohs(peer.sin_port)));
 owner_lock=open(lock_path,O_RDWR|O_CREAT|O_CLOEXEC|O_NOFOLLOW,0600);
 if(owner_lock<0||flock(owner_lock,LOCK_EX|LOCK_NB)){
  const int saved=errno;thermal_i2c_close();errno=saved;return -1;
 }
#endif
 sequence=uint32_t(std::chrono::steady_clock::now().time_since_epoch().count());
 return 0;
}
static int rpc(uint8_t op,uint16_t address,uint8_t count,uint16_t value,uint16_t *out){
 if(ensure_open())return -1;
 sockaddr_in local={};SocketLength size=sizeof(local);getsockname(transport,reinterpret_cast<sockaddr*>(&local),&size);
 uint8_t request[20]={ 'F','T','I','Q',1,op,count,0 };
 const uint32_t id=++sequence;put32(request+8,id);put16(request+12,address);put16(request+14,value);put16(request+16,ntohs(local.sin_port));
 for(int attempt=0;attempt<3;++attempt){
  if(sendto(transport,reinterpret_cast<const char*>(request),20,0,reinterpret_cast<sockaddr*>(&peer),sizeof(peer))!=20)return -1;
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(120);
  do{
   uint8_t response[80];sockaddr_in sender={};SocketLength sender_size=sizeof(sender);
   const int n=recvfrom(transport,reinterpret_cast<char*>(response),sizeof(response),0,reinterpret_cast<sockaddr*>(&sender),&sender_size);
   if(n<0)break;
   if(sender.sin_addr.s_addr!=peer.sin_addr.s_addr||n<16||std::memcmp(response,"FTRP",4)||response[4]!=1||get32(response+8)!=id||get16(response+12)!=address)continue;
   const unsigned words=get16(response+6);
   if(response[5]||words!=(op==1?count:0)||n!=int(16+2*words)){errno=EIO;return -1;}
   for(unsigned i=0;i<words;++i)out[i]=get16(response+16+2*i);
   return 0;
  }while(std::chrono::steady_clock::now()<deadline);
 }
 errno=ETIMEDOUT;return -1;
}
void MLX90640_I2CInit(){ensure_open();}
int MLX90640_I2CRead(uint8_t device,uint16_t start,uint16_t words,uint16_t *data){
 if(device!=0x33||!data||!words)return -1;
 for(unsigned offset=0;offset<words;){const uint8_t count=words-offset>32?32:words-offset;
  if(rpc(1,start+offset,count,0,data+offset))return -1;
  offset+=count;
 }
 return 0;
}
int MLX90640_I2CWrite(uint8_t device,uint16_t address,uint16_t value){return device==0x33?rpc(2,address,1,value,nullptr):-1;}
