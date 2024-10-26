#ifndef WIFI_H
#define WIFI_H

#include "Utils.h"
#include "WiFiS3.h"
#include "arduino_secrets.h"

void wifi_init();

String get_MAC();

#endif