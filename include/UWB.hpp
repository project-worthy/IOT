#ifndef UWB_H
#define UWB_H
#include <ArduinoJson.h>
#include <SocketIOclient.h>
#include <WebSocketsClient.h>

#include "AIServer.hpp"
#include "Utils.h"
#include "WIFI_utils.hpp"
#include "dw3000.h"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 3000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define ALL_MSG_COMMON_LEN 10  // Length of the common part of the message

#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

#define RX_BUF_LEN 20

#define POLL_TX_TO_RESP_RX_DLY_UUS 1720  // 1720
#define RESP_RX_TIMEOUT_UUS 250

#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

#define DEFAULT_DEVICE 1  // 0 anchor, 1 tag

extern int anchor_ready;
extern int ai_run_mode;

extern int antena_delay_index;
extern int antena_delay_receive[3];
extern int antena_delay_self;

extern double this_anchor_target_distance;
extern int sending_mode;

extern dwt_txconfig_t txconfig_options;

typedef struct MSG_STRUCT msg_struct;
typedef struct UWB_DATA uwb_data;

void uwb_set_sending_mode(int mode);

void uwb_default_init();

void uwb_run_as_anchor();

void uwb_run_as_tag();

void uwb_calibrate(float dist);

void uwb_anchor_is_ready();

void uwb_calibrate_rcv_loop();

void uwb_calibrate_send_loop();

void uwb_loop();

void uwb_sendDistance();

#endif