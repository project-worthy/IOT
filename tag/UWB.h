#pragma once
// #ifndef UWB_H
// #define UWB_H
#include "dw3000.h"
#include "Utils.h"
#include "WIFI.h"

extern uint8_t poll_msg[12];
extern uint8_t resp_msg[20];

// connection pins
typedef struct {
  uint8_t rst; // reset
  uint8_t irq; // interupt
  uint8_t ss; // cs
} uwb_data;

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

uwb_data uwb_board = {
  .rst = 6,
  .irq = 7,
  .ss = 10,
};



/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 3000

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

#define POLL_TX_TO_RESP_RX_DLY_UUS 1720 // 1720
#define RESP_RX_TIMEOUT_UUS 250

#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

#define DEFAULT_DEVICE 1 // 0 anchor, 1 tag

extern dwt_txconfig_t txconfig_options;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;
double distances[3] = {0};
float last_delta = 0.0;
float this_anchor_target_distance = 1.17;
uint16_t Adelay_delta = 100;
uint16_t this_anchor_Adelay = 16600;

uint8_t sending_mode = 0;

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

extern int ai_run_mode;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
uint16_t frame_len; //for tag

int send_loop_count = 0;
extern int anchor_ready = 0;

typedef struct MSG_STRUCT{
  uint8_t tx_poll_msg[12];
  uint8_t rx_resp_msg[20];
} msg_struct;

msg_struct msg_collection[3] = {
  {
    {0x41, 0x88, 0, 0xCA, 0xDE, 'S', 'I', 'J', 'A', 0xE0, 0, 0},
    {0x41, 0x88, 0, 0xCA, 0xDE, 'J', 'A', 'S', 'I', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  },
  {
    {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0},
    {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  },
  {
    {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'S', 'B', 'C', 0xE0, 0, 0},
    {0x41, 0x88, 0, 0xCA, 0xDE, 'B', 'C', 'D', 'S', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  }
};

void uwb_set_sending_mode(int mode){
  sending_mode = mode;
  this_anchor_Adelay = 16600;
  Adelay_delta = 100;
  if(mode == 2){
    send_loop_count = 0;
  }
  if(mode == 1){
    anchor_ready = 0;
  }
}

void uwb_default_init() {
  dwt_softreset();

  spiBegin(uwb_board.irq,uwb_board.rst);
  spiSelect(uwb_board.ss);

  delay(200); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  delay(200);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }
  
  // Configure DW IC. See NOTE 5 below.
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }
  delay(200); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
  
}

void uwb_run_as_anchor() {
  uwb_default_init();

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  dwt_setrxaftertxdelay(0);
  dwt_setrxtimeout(0);

  dwt_setrxantennadelay(this_anchor_Adelay);
  dwt_settxantennadelay(this_anchor_Adelay);
}

void uwb_run_as_tag() {
  uwb_default_init();

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  dwt_configuretxrf(&txconfig_options);

  dwt_setrxantennadelay(this_anchor_Adelay);
  dwt_settxantennadelay(this_anchor_Adelay);

  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}

void uwb_calibrate(float dist) {
  // float dist = ranging(&device1, 100);
  if (Adelay_delta < 2) { // Antenna delay is close enough
    Serial.print("final Adelay ");
    Serial.println(this_anchor_Adelay);
    return;
  }
  // error in measured distance
  float this_delta = dist - this_anchor_target_distance;
  if (this_delta * last_delta < 0.0) {
    // sign changed, reduce step size
    Adelay_delta = Adelay_delta / 2;  
  }
  last_delta = this_delta;
  if (this_delta > 0.0) {
    //new trial Adelay
    this_anchor_Adelay += Adelay_delta;
  }
  else this_anchor_Adelay -= Adelay_delta;
  // Set antenna delay to new delay
  dwt_setrxantennadelay(this_anchor_Adelay);
  dwt_settxantennadelay(this_anchor_Adelay);
}

void uwb_anchor_is_ready(){
  JsonDocument doc;
  AIServer_sendEvent("ready_anchor",doc);
}

int uwb_receiver_on = 0;
int uwb_received = 0;
void uwb_calibrate_rcv_loop(){
  if(!uwb_receiver_on){
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    memset(rx_buffer, 0, sizeof(rx_buffer));
    uwb_receiver_on = 1;
  }

  if (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
    return;
  
  status_reg = dwt_read32bitreg(SYS_STATUS_ID);
  // if(!status_reg & SYS_STATUS_ALL_RX_ERR) return;
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
      // uwb_re
      uint32_t frame_len;
      
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
      uwb_receiver_on = 0;
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
      
      if (frame_len <= sizeof(rx_buffer))
      {
          dwt_readrxdata(rx_buffer, frame_len, 0);

          rx_buffer[ALL_MSG_SN_IDX] = 0;

          if (memcmp(rx_buffer, poll_msg, ALL_MSG_COMMON_LEN) == 0)
          {

              uint32_t resp_tx_time;
              int ret;

              poll_rx_ts = get_rx_timestamp_u64();

              resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
              dwt_setdelayedtrxtime(resp_tx_time);

              resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

              resp_msg_set_ts(&resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
              resp_msg_set_ts(&resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

              resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
              dwt_writetxdata(sizeof(resp_msg), resp_msg, 0); /* Zero offset in TX buffer. */
              dwt_writetxfctrl(sizeof(resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
              ret = dwt_starttx(DWT_START_TX_DELAYED);

              if (ret == DWT_SUCCESS)
              {
                  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                  { };

                  Serial.println("frame received");
                  uwb_received= 1;

                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

                  frame_seq_nb++;
              }
          }
      }
  }
  else
  {
      /* Clear RX error events in the DW IC status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
      uwb_receiver_on = 0;
  }
}

void uwb_calibrate_send_loop(){
  if(send_loop_count >= 100) {
    set_run_mode(0); //make to default
    JsonDocument send_doc;
    send_doc["delay"] = this_anchor_Adelay;
    AIServer_sendEvent("calibrate",send_doc);

  };
    poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(poll_msg), poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

  
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { };

    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        uint32_t frame_len;

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        Serial.println(frame_len);
        if (frame_len <= sizeof(rx_buffer))
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer,  resp_msg, ALL_MSG_COMMON_LEN) == 0)
            // if (memcmp(rx_buffer,  tx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                int32_t rtd_init, rtd_resp;
                double clockOffsetRatio ;

                poll_tx_ts = dwt_readtxtimestamplo32();
                resp_rx_ts = dwt_readrxtimestamplo32();

                int16_t a = dwt_readclockoffset();

                clockOffsetRatio =  a * (double)1/ ((int32_t)1<<26);

                resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                rtd_init = resp_rx_ts - poll_tx_ts;
                rtd_resp = resp_tx_ts - poll_rx_ts;
                tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;
                Serial.println(distance);
                uwb_calibrate(distance);
                /* setting distance */
                // distances[i] = distance;
            }
        }
    }
    else
    {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
  send_loop_count++;
}

void uwb_loop(){
  for(uint8_t i = 0; i < 3; i++){
    Serial.println(antena_delay_receive[i]);
    // dwt_setrxantennadelay(antena_delay_receive[i]);
    // dwt_settxantennadelay(antena_delay_receive[i]);
    dwt_setrxantennadelay(16310);
    dwt_settxantennadelay(16310);
    //TODO: 이거 값이 제대로 설정이 안되는거 같음 하드로 하면 괜찮은데 다이나믹으로 하면 값이 이상하게 튐
  /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
    msg_collection[i].tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(msg_collection[i].tx_poll_msg), msg_collection[i].tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(msg_collection[i].tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
      * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    { };

    /* Increment frame sequence number after transmission of the poll message (modulo 256). */
    frame_seq_nb++;

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
    {
        uint32_t frame_len;

        /* Clear good RX frame event in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer))
        {
            dwt_readrxdata(rx_buffer, frame_len, 0);

            /* Check that the frame is the expected response from the companion "SS TWR responder" example.
              * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, msg_collection[i].rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
                int32_t rtd_init, rtd_resp;
                double clockOffsetRatio ;

                /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
                poll_tx_ts = dwt_readtxtimestamplo32();
                resp_rx_ts = dwt_readrxtimestamplo32();

                /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
                int16_t a = dwt_readclockoffset();

                clockOffsetRatio =  a * (double)1/ ((int32_t)1<<26);

                /* Get timestamps embedded in response message. */
                resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
                resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

                /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
                rtd_init = resp_rx_ts - poll_tx_ts;
                rtd_resp = resp_tx_ts - poll_rx_ts;
                tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                distance = tof * SPEED_OF_LIGHT;
                Serial.println(String(i) + ":" + String(distance));
                // Serial.println(distance);
                /* setting distance */
                distances[i] = distance;
            }
        }
    }
    else
    {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
  }
}

// #endif