#!/usr/bin/env python3
"""Exercise the actual firmware HTTP writer against stalled local sockets."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=(root/'CO_2_Detectior_v3.ino').read_text()
writer=s[s.index('class BoundedWebServer'):s.index('}; // bounded HTTP transport')+2]
prefix=r'''
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <cassert>
#include <cstdio>
#include <string>
#include <fcntl.h>
#include <csignal>
using PGM_P=const char*;
uint32_t millis(){return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
void delay(int ms){std::this_thread::sleep_for(std::chrono::milliseconds(ms));}
struct Client {int socket=-1;bool connected(){return socket>=0;}int fd(){return socket;}void stop(){if(socket>=0)close(socket);socket=-1;}};
class WebServer {public:WebServer(int){} Client _currentClient;protected:virtual size_t _currentClientWrite(const char*,size_t){return 0;}virtual size_t _currentClientWrite_P(PGM_P,size_t){return 0;}};
'''
test=r'''
struct TestServer:BoundedWebServer {using BoundedWebServer::BoundedWebServer;using BoundedWebServer::_currentClientWrite;};
int connect_test(TestServer& s){int f[2];assert(socketpair(AF_UNIX,SOCK_STREAM,0,f)==0);int n=1024;setsockopt(f[0],SOL_SOCKET,SO_SNDBUF,&n,sizeof(n));fcntl(f[0],F_SETFL,O_NONBLOCK);s._currentClient.socket=f[0];s.begin_cycle();return f[1];}
int main(){signal(SIGPIPE,SIG_IGN);TestServer s(80);int peer=connect_test(s);assert(s._currentClientWrite("ok",2)==2);char b[2];assert(read(peer,b,2)==2&&b[0]=='o');s._currentClient.stop();close(peer);
peer=connect_test(s);std::string payload(1024*1024,'x');auto started=millis();auto n=s._currentClientWrite(payload.data(),payload.size());assert(n<payload.size());assert(millis()-started>=450&&millis()-started<1500);assert(s.aborted_responses==1&&!s._currentClient.connected());close(peer);
peer=connect_test(s);std::atomic<bool> stop{false};std::thread reader([&]{char buf[256];while(!stop){recv(peer,buf,sizeof(buf),MSG_DONTWAIT);delay(20);}});started=millis();n=s._currentClientWrite(payload.data(),payload.size());stop=true;reader.join();assert(n<payload.size());assert(millis()-started>=2000&&millis()-started<3000);assert(s.aborted_responses==2);close(peer);
peer=connect_test(s);assert(s._currentClientWrite("new",3)==3);s._currentClient.stop();close(peer);peer=connect_test(s);close(peer);assert(s._currentClientWrite("gone",4)==0);assert(!s._currentClient.connected());
peer=connect_test(s);s.response_active=true;s.response_started=millis()-3000;assert(s._currentClientWrite("expired",7)==0);close(peer);
puts("PASS: actual HTTP writer, normal send, stalled peer, trickle peer, response deadline and next request recovery");}
'''
with tempfile.TemporaryDirectory(prefix='co2-network-') as tmp:
 p=Path(tmp);(p/'test.cpp').write_text(prefix+writer+test)
 subprocess.run(['clang++','-std=c++17','-Wall','-Wextra','-Werror','-pthread',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True,timeout=10)
