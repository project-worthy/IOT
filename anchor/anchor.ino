#include <Thread.h>
#include <ThreadController.h>
#include <ArduinoJson.h>

#include "dw3000.h"

#include "global.h"
#include "AIServer.h"
#include "UWB.h"
#include "WIFI.h"
#include "Utils.h"
#include "arduino_secrets.h"


#define APP_NAME "SIMPLE RX v1.1"

Thread uwbMainThread = Thread();
Thread uwbSendThread = Thread();
Thread socketThread = Thread();


int loop_count = 0;
void setup(){
  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  wifi_init();
  delay(3000);


  socket_init_AIServer(AI_SOCK_IP,AI_SOCK_PORT);
  // setting up thread

  uwbSendThread.onRun(uwb_calibrate_send_loop);
  uwbSendThread.setInterval(400);

  // uwb_run_as_tag();
  // ai_run_mode = 2;
}

// int anchor_ready = 0;
void loop(){
  if(ai_run_mode > -1){
    print_INFO("Arduino",String("starting with mode: " + String(ai_run_mode)).c_str());
    uwb_set_sending_mode(ai_run_mode);
    if(ai_run_mode == 1)
      uwb_run_as_anchor();
    else if(ai_run_mode == 2){
      uwb_run_as_tag();
    }
    else if(ai_run_mode == 3){
      uwb_run_as_anchor();
    }
    set_run_mode(-1); 
  }
  // uwb_calibrate_send_loop();
  // delay(300);
  if(loop_count > 1000){
    AIServer_loop();
    loop_count = 0;
  }
  if(sending_mode == 1){ // calibrate anchor
    uwb_calibrate_rcv_loop();
    if(anchor_ready == 0){
      uwb_anchor_is_ready();
      anchor_ready = 1;
    }
  }
  else if(sending_mode == 2){
    uwbSendThread.run();
    delay(100);
  }
  else if(sending_mode == 3){ // default mode
    uwb_loop();
  }
  loop_count++;
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
