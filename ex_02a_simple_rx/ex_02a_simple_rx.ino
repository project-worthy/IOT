#include "dw3000.h"

#define APP_NAME "SIMPLE RX v1.1"


#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

static uint8_t frame_seq_nb = 0;

#define RNG_DELAY_MS 1000
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 400

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */

#define POLL_RX_TO_RESP_TX_DLY_UUS 2000

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

#define IDLE_MODE 0x00

/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 5 below. */
// extern dwt_txconfig_t txconfig_options;

// connection pins
typedef struct {
  uint8_t rst; // reset
  uint8_t irq; // interupt
  uint8_t ss; // cs
  dwt_txconfig_t txconfig;
} uwb_data;

uwb_data uwb_board[3] = {
  {
    .rst = 6,
    .irq = 7,
    .ss = 5,
    .txconfig = {0x34,0xfdfdfdfd,0x0}
  },
  {
    .rst = 22,
    .irq = 24,
    .ss = 26,
    .txconfig = {0x34,0xfdfdfdfd,0x0}
  },
  {
    .rst = 34,
    .irq = 36,
    .ss = 38,
    .txconfig = {0x34,0xfdfdfdfd,0x0}
  }
};

// uwb_board[0].txconfig.PGdly = 0x34;  /* PG delay. */
// uwb_board[0].txconfig.power = 0xfdfdfdfd; /* TX power. */
// uwb_board[0].txconfig.PGcount = 0x0;  /*PG count*/

// uwb_board[1].txconfig.PGdly = 0x34;  /* PG delay. */
// uwb_board[1].txconfig.power = 0xfdfdfdfd; /* TX power. */
// uwb_board[1].txconfig.PGcount = 0x0;  /*PG count*/

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

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
uint32_t status_reg;
/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
uint16_t frame_len;

void setup()
{
  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  for(int i = 0; i < 3; i++){
    Serial.print("check rc: ");
    Serial.println(i);

    spiBegin(uwb_board[i].irq,uwb_board[i].rst);
    spiSelect(uwb_board[i].ss);

    delay(200); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

    while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
    {
      UART_puts("IDLE FAILED\r\n");
      while (1)
        ;
    }

    dwt_softreset();
    delay(200);

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
      UART_puts("INIT FAILED\r\n");
      while (1)
        ;
    }

    // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
    // dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE); 

    // Configure DW IC. See NOTE 5 below.
    if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
    {
      UART_puts("CONFIG FAILED\r\n");
      while (1)
        ;
    }
    // Serial.print(uwb_board[i].txconfig.PGcount);
    // dwt_configuretxrf(&(uwb_board[i].txconfig));

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
  }

 
  
}

void select_board(int pin_number){
  digitalWrite(uwb_board[pin_number].ss,LOW);
}

void unselect_board(int pin_number){
  digitalWrite(uwb_board[pin_number].ss,HIGH);
}
uint8_t board_num = 0;
char a[10] = {0};
void loop()
{
  /* TESTING BREAKPOINT LOCATION #1 */
  
  sprintf(a,"%d",board_num);
  test_run_info(a);
  /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
   * the RX buffer.
   * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
   * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
  memset(rx_buffer, 0, sizeof(rx_buffer));

  /* Activate reception immediately. See NOTE 2 below. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  /* Poll until a frame is properly received or an error/timeout occurs. See NOTE 3 below.
   * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
   * function to access it. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
  {
  };
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

          /* Check that the frame is a poll sent by "SS TWR initiator" example.
            * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
          rx_buffer[ALL_MSG_SN_IDX] = 0;
          if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
          {
              uint32_t resp_tx_time;
              int ret;

              /* Retrieve poll reception timestamp. */
              poll_rx_ts = get_rx_timestamp_u64();

              /* Compute response message transmission time. See NOTE 7 below. */
              resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
              dwt_setdelayedtrxtime(resp_tx_time);

              /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
              resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

              /* Write all timestamps in the final message. See NOTE 8 below. */
              resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
              resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

              /* Write and send the response message. See NOTE 9 below. */
              tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
              dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
              dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
              ret = dwt_starttx(DWT_START_TX_DELAYED);

              /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
              if (ret == DWT_SUCCESS)
              {
                  /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
                  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                  { };

                  /* Clear TXFRS event. */
                  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

                  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                  frame_seq_nb++;
              }
          }
      }
  }
  else
  {
      /* Clear RX error events in the DW IC status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
  select_device(uwb_board[board_num].ss,uwb_board[board_num].irq,uwb_board[board_num].rst);

  board_num++;
  if(board_num > 2) board_num =0;
  
  // delay(200);
}
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. In this example, maximum frame length is set to 127 bytes which is 802.15.4 UWB standard maximum frame length. DW IC supports an extended
 *    frame length (up to 1023 bytes long) mode which is not used in this example.
 * 2. Manual reception activation is performed here but DW IC offers several features that can be used to handle more complex scenarios or to
 *    optimise system's overall performance (e.g. timeout after a given time, automatic re-enabling of reception in case of errors, etc.).
 * 3. We use polled mode of operation here to keep the example as simple as possible, but RXFCG and error/timeout status events can be used to generate
 *    interrupts. Please refer to DW IC User Manual for more details on "interrupts".
 ****************************************************************************************************************************************************/
