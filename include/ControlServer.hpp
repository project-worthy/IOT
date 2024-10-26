#ifndef CONTROL_SERVER_HPP
#define CONTROL_SERVER_HPP

#include <ArduinoJson.h>
#include <SocketIOclient.h>
#include <WebSocketsClient.h>

#include "Utils.h"
#include "WIFI_utils.hpp"

extern uint8_t led_power;

void ControlServer_event(socketIOmessageType_t type, uint8_t *payload, size_t length);

void socket_init_ControlServer(String host, int port);

void ControlServer_loop();

#endif