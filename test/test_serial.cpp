// C library headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstring>
#include <fstream>

// C library headers
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <chrono>

#include "CppLinuxSerial/SerialPort.hpp"

using namespace mn::CppLinuxSerial;

bool command(SerialPort & serialPort, std::string msg){
  auto start = std::chrono::system_clock::now().time_since_epoch();  
  std::string readData;
  serialPort.Write(msg + "\r\n");
  while(true){
    if(serialPort.Available() !=0){
      serialPort.Read(readData);
      std::cout<<readData.c_str()<<std::endl;
      if(readData.find("ok") != std::string::npos){
        return true;
      }
      usleep(1000);
    }
  }
}


bool read_all(SerialPort & serialPort){
  std::string readData;
  while(serialPort.Available() !=0){
    serialPort.Read(readData);
    usleep(100);
  }
  return true;
}

int main(){
  std::string deviceName = "/dev/ttyUSB0";

  SerialPort serialPort(deviceName, BaudRate::B_115200);
  serialPort.SetTimeout(-1);
  serialPort.Open();
  bool ok = true;
  
  read_all(serialPort);
  command(serialPort, "M400 \r\n");
  command(serialPort, "M106 S250\r\n");
  // printf("%s", "s");

  std::ifstream file;
  file.open("../test_cube.gcode", std::ios::in);
  std::string line;
  while (std::getline(file, line)) {
    std::cout<< line<< std::endl;
    if(line.front() == ';'){
      continue;
    }
    else{
      ok = command(serialPort, line);
    }
  }
  file.close();

  // for(int i=0; i< 10; i++){
  //   ok = command(serialPort, "M106 S250\r\n");
  //   usleep(1000000);
  //   ok = command(serialPort, "M107 \r\n");
  //   usleep(1000000);
  // }
  
  return 0;
}
