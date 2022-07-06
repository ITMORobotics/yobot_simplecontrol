// C library headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstring>

// C library headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstring>

#include "CppLinuxSerial/SerialPort.hpp"

using namespace mn::CppLinuxSerial;

bool command(int serial_port, std::string msg){
  return true;
}


int main(){
  std::string deviceName = "/dev/USB0";

  SerialPort serialPort(std::string deviceName, BaudRate::B_115200);
  serialPort.SetTimeout(-1);
  serialPort.Open();
  serialPort.Write("M106 S250\r\n");

  while(true){
    std::string readData;
    serialPort.Read(readData, 0.001);
    if(readData != ""){
      std::cout<<readData<<std::cout;
      serialPort.Write("M107\r\n");
    }
    sleep(0.01);
  }
  return 0;
}
