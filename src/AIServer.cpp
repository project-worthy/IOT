#include "AIServer.hpp"

SocketIOclient socket;

int received = 0;
int ai_run_mode = -1;

void set_run_mode(int mode) {
    ai_run_mode = mode;
}

void AIServer_event(socketIOmessageType_t type, uint8_t* payload, size_t length) {
    print_INFO("AI_Server", String("receving data:" + String((char*)payload)).c_str());
    switch (type) {
        case sIOtype_DISCONNECT:
            print_INFO("AI_Server", "Disconnected!");
            break;
        case sIOtype_CONNECT: {
            print_INFO("AI_Server", "Connected!");
            socket.send(sIOtype_CONNECT, "/iot_calibrate");
            socket.send(sIOtype_CONNECT, "/coordinate");
            JsonDocument doc;
            doc["mac"] = get_MAC();
            doc["type"] = TAG;
            doc["id"] = 0;
            for (int i = 0; i < 12; i++) {
                doc["poll_msg"][i] = rx_poll_msg[i];
            }
            for (int i = 0; i < sizeof(tx_resp_msg); i++) {
                doc["resp_msg"][i] = tx_resp_msg[i];
            }
            // Serial.println(doc);
            AIServer_sendEvent("initial", doc);
        } break;
        case sIOtype_EVENT: {
            JsonDocument receive_doc;
            uint8_t index = 0;
            String payload_str = String((char*)payload);
            index = payload_str.indexOf(',');
            payload_str.remove(0, index + 1);
            deserializeJson(receive_doc, payload_str.c_str());

            String event = String(receive_doc[0]);
            if (event == "calibrate") {
                ai_run_mode = receive_doc[1]["run_type"];
                this_anchor_target_distance = receive_doc[1]["distance"];

                for (int i = 0; i < 12; i++) {
                    poll_msg[i] = receive_doc[1]["poll_msg"][i].as<uint8_t>();
                }
                for (int i = 0; i < sizeof(tx_resp_msg); i++) {
                    resp_msg[i] = receive_doc[1]["resp_msg"][i].as<uint8_t>();
                }
                received = 1;
            }
            if (event == "end_calibrate") {
                ai_run_mode = 0;
            }
            if (event == "terminate_calibrate") {
                ai_run_mode = 3;
                antena_delay_index = receive_doc[1]["index"];
                Serial.println(antena_delay_index);
                antena_delay_self = receive_doc[1]["delays"][antena_delay_index];

                // need to change dynamically
                int index = 0;
                for (int i = 0; i < 4; i++) {
                    if (antena_delay_index != i) {
                        antena_delay_receive[index] = receive_doc[1]["delays"][index];
                        index++;
                    }
                }
                // socket.disconnect();
            }
        } break;
        case sIOtype_ACK:
        case sIOtype_ERROR:
        case sIOtype_BINARY_EVENT:
        case sIOtype_BINARY_ACK:
            break;
    }
}

void socket_init_AIServer(String host, int port) {
    print_INFO("AI_Server", String(host + ":" + String(port)).c_str());
    socket.begin(host.c_str(), port, "/ws/socket.io/?EIO=4");
    socket.onEvent(AIServer_event);
}

void AIServer_sendEvent(String event, JsonDocument json) {
    DynamicJsonDocument doc(128);
    JsonArray array = doc.to<JsonArray>();

    // add evnet name
    array.add(event);
    array.add(json);

    // JSON to String (serializion)
    String jsonoutput;
    serializeJson(doc, jsonoutput);
    size_t buf_size = jsonoutput.length() + 2;
    uint8_t* buffer = new uint8_t[buf_size];
    String payload = String("/iot_calibrate," + jsonoutput);
    print_INFO("AI_Server", String("sending Data: " + payload).c_str());
    socket.send(sIOtype_EVENT, payload);
    delete buffer;
}

void DistanceServer_sendEvent(String event, JsonDocument json) {
    DynamicJsonDocument doc(128);
    JsonArray array = doc.to<JsonArray>();

    // add evnet name
    array.add(event);
    array.add(json);

    // JSON to String (serializion)
    String jsonoutput;
    serializeJson(doc, jsonoutput);
    size_t buf_size = jsonoutput.length() + 2;
    uint8_t* buffer = new uint8_t[buf_size];
    String payload = String("/coordinate," + jsonoutput);
    print_INFO("AI_Server", String("sending Data: " + payload).c_str());
    socket.send(sIOtype_EVENT, payload);
    delete buffer;
}

void AIServer_loop() {
    socket.loop();
    if (received) {
        received = 0;
    }
}

void AISerer_disconnect() {
    socket.disconnect();
}
