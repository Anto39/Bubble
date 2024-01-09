/**
 * @file ProtocolI2C.h
 * @author Felix Parent (fparent@cimeq.qc.ca)
 * @brief
 * @version 0.1
 * @date 2023-06-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <stdio.h>
#include <vector>
#include <optional>
#include "TBaseCrc.h"


#define TRAME_BEGIN 60
#define TRAME_END 62
#define MAX_PAYLOAD_SIZE 10

using namespace std;

namespace ProtocolI2C
{
    enum StateReception{WAIT, ID_PACKET, SIZE_PACKET, DATA_PACKET, CHECKSUM1, CHECKSUM2, VALIDATE};

    enum TypesCommande{
        TS_ADD_BUTTON,
        TS_CHECK_BUTTON_STATE,
        TS_CHECK_BUTTON_NUMBER,
        TS_ADD_STRING,
        TS_ADD_TITLE,
        TS_ADD_LOGO,

        WIFI_CONNECT,
        WIFI_INFOS,

        MQTT_PUBLISH,
        MQTT_SUBSCRIBE,
        MQTT_CONNECT
    };

    struct InfosCommande{
        uint8_t id;
        uint8_t size;
        vector<uint8_t> data;
    };

    std::optional<InfosCommande> protocolI2cDecode(char * rxBuffer, uint8_t size);
    void protocolI2cEncode(char * txBuffer, uint8_t * data, uint8_t id, uint8_t size);

};
