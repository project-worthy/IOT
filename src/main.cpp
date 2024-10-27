#include "main.hpp"

#define APP_NAME "SIMPLE RX v1.1"

Thread uwbMainThread = Thread();
Thread distanceThread = Thread();
Thread uwbSendThread = Thread();
Thread socketThread = Thread();
ThreadController controll = ThreadController();

int loop_count = 0;
void setup()
{
    UART_init();
    test_run_info((unsigned char *)APP_NAME);

    wifi_init();
    delay(3000);

    socket_init_AIServer(AI_SOCK_IP, AI_SOCK_PORT);
    // socket_init_ControlServer(CONTROL_SOCKET_IP,CONTROL_SOCKET_PORT);
    // setting up thread

    uwbSendThread.onRun(uwb_calibrate_send_loop);
    uwbSendThread.setInterval(400);

    distanceThread.onRun(uwb_sendDistance);
    distanceThread.setInterval(4000);

    socketThread.onRun(uwb_loop);
    socketThread.setInterval(4000);

    controll.add(&socketThread);
    controll.add(&distanceThread);

    // uwb_run_as_tag();
    ai_run_mode = 3;
}

void loop()
{
    if (ai_run_mode > -1)
    {
        print_INFO("Arduino", String("starting with mode: " + String(ai_run_mode)).c_str());
        uwb_set_sending_mode(ai_run_mode);
        if (ai_run_mode == 1)
            uwb_run_as_anchor();
        else if (ai_run_mode == 2)
        {
            uwb_run_as_tag();
        }
        else if (ai_run_mode == 3)
        {
            uwb_run_as_tag();
        }
        set_run_mode(-1);
    }

    if (loop_count > 500)
    {
        // ControlServer_loop();
    }
    if (loop_count > 500)
    {
        AIServer_loop();
        loop_count = 0;
    }

    if (sending_mode == 1)
    { // calibrate anchor
        uwb_calibrate_rcv_loop();
        if (anchor_ready == 0)
        {
            uwb_anchor_is_ready();
            anchor_ready = 1;
        }
    }
    else if (sending_mode == 2)
    {
        uwb_calibrate_send_loop();
    }
    else if (sending_mode == 3)
    { // default mode
        controll.run();
    }
    loop_count++;
}