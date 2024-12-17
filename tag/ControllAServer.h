#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <ArduinoJson.h>
#include <ArduinoJson.h>

#include "Utils.h"
#include "WIFI.h"

SocketIOclient controlServer;


int control_received = 0;
int control_run_mode = -1;

StaticJsonDocument<128> receive_doc;
void ControlServer_event(socketIOmessageType_t type, uint8_t * payload, size_t length) {
  print_INFO("CONTROL_SERVER",String("receving data:" + String((char*)payload)).c_str());
  switch (type) {
    case sIOtype_DISCONNECT:
      print_INFO("CONTROL_SERVER","Disconnected!");
      break;
    case sIOtype_CONNECT:
      print_INFO("CONTROL_SERVER","Connected!");
      controlServer.send(sIOtype_CONNECT, "/iot");
      break;
    case sIOtype_EVENT:
    {
      uint8_t index = 0;
      String payload_str = String((char*)payload);
      index = payload_str.indexOf(',');
      payload_str.remove(0,index + 1);
      deserializeJson(receive_doc,payload_str.c_str());
      String event = String(receive_doc[0]);
      String cmd = String(receive_doc[1]);
      if(event == "iot_event"){
        if(cmd == "turn_on"){
          Serial.println("true");
          led_power = 255;
        }
        if(cmd == "turn_off"){
          Serial.println("false");
          led_power = 0;
        }
      }
    }
      Serial.print("[WSc] get text:");
      Serial.println((char *)payload);
      break;
    case sIOtype_ACK:
    case sIOtype_ERROR:
    case sIOtype_BINARY_EVENT:
    case sIOtype_BINARY_ACK:
      break;
  }
}

void socket_init_ControlServer(String host,int port){
  print_INFO("Controll_Server",String(host + ":" + String(port)).c_str());
  controlServer.begin(host.c_str(),port,"/ws/socket.io/?EIO=4");
  controlServer.onEvent(ControlServer_event);
}

void ControlServer_loop(){
  controlServer.loop();
  if(control_received){
    control_received = 0;
  }
}

