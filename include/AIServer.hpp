
#ifndef AISERVER_H
#define AISERVER_H
#include <ArduinoJson.h>
#include <SocketIOclient.h>
#include <WebSocketsClient.h>

#include "UWB.hpp"
#include "Utils.h"
#include "WIFI_utils.hpp"
static uint8_t poll_msg[12] = {0};
static uint8_t resp_msg[20] = {0};

extern uint8_t poll_msg[12];
extern uint8_t resp_msg[20];

void AIServer_sendEvent(String event, JsonDocument json);

void set_run_mode(int mode);

void AIServer_event(socketIOmessageType_t type, uint8_t* payload, size_t length);

void socket_init_AIServer(String host, int port);

void AIServer_sendEvent(String event, JsonDocument json);

void DistanceServer_sendEvent(String event, JsonDocument json);

void AIServer_loop();

void AISerer_disconnect();

#endif
