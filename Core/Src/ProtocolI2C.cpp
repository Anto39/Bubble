/**
 * @file I2C.cpp
 * @author Felix Parent (fparent@cimeq.qc.ca)
 * @brief
 * @version 0.1
 * @date 2023-06-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "ProtocolI2C.h"
using namespace ProtocolI2C;



bool stateMachine(char * buffer, uint8_t sizeBuffer, uint8_t& idPacket, uint8_t& sizePacket, vector<uint8_t>& data);

/**
 * @brief Decode the data received from the i2c
 *
 * @param buffer
 * @param size
 */
optional<InfosCommande> ProtocolI2C::protocolI2cDecode(char * buffer, uint8_t sizeBuffer)
{
    std::optional<InfosCommande> commande;
    InfosCommande infosCommande = {
        .id = 0,
        .size = 0,
    };

    if(stateMachine(buffer, sizeBuffer, infosCommande.id, infosCommande.size, infosCommande.data))
        commande = infosCommande;

    return commande;
}

/**
 * @brief Encode the data to trasnmit on the i2c
 *
 * @param txBuffer
 * @param data
 * @param size
 */
void ProtocolI2C::protocolI2cEncode(char * txBuffer, uint8_t * data, uint8_t id, uint8_t size)
{
    TBaseCrc checksum;
    checksum.calc(id);
    checksum.calc(size);

    txBuffer[0] = TRAME_BEGIN;
    txBuffer[1] = id;
    txBuffer[2] = size;

    for(int i = 0; i < size; i++)
    {
        txBuffer[i+3] = data[i];
        checksum.calc(data[i]);
    }
    txBuffer[size + 3] = (uint8_t)(checksum.get() >> 8);
    txBuffer[size + 4] = (uint8_t)checksum.get();
    txBuffer[size + 5] = TRAME_END;
}

/**
 * @brief Parse the data received on the i2c bus
 *
 * @param buffer data buffer
 * @param sizeBuffer size of the buffer
 * @param idPacket the id of the command
 * @param sizePacket the size of the packet
 * @param data the data received
 * @return true
 * @return false
 */
bool stateMachine(char * buffer, uint8_t sizeBuffer, uint8_t& idPacket, uint8_t& sizePacket, vector<uint8_t>& data)
{
    TBaseCrc checksumTest;
    StateReception state = WAIT;
    bool newValidPacket = false;
    uint8_t checksum = 0;
    uint8_t index = 0;

    for(int i = 0; i < sizeBuffer; i++)
    {
        switch(state)
        {
            case WAIT:
                index = 0;
                state = buffer[i] == TRAME_BEGIN ? ID_PACKET : WAIT;
                break;
            case ID_PACKET:
                idPacket = buffer[i];
                checksumTest.calc(idPacket);
                state = SIZE_PACKET;
                break;
            case SIZE_PACKET:
                sizePacket = buffer[i];
                checksumTest.calc(sizePacket);
                state = DATA_PACKET;
                break;
            case DATA_PACKET:
                data.push_back(buffer[i]);
                checksumTest.calc(buffer[i]);
                index++;
                state = (index == sizePacket)? CHECKSUM1 : DATA_PACKET;
                break;
            case CHECKSUM1:
                checksum = (uint8_t)(buffer[i]);
                state = (checksum == (uint8_t)(checksumTest.get() >> 8))? CHECKSUM2 : WAIT;
                break;
            case CHECKSUM2:
                checksum = (uint8_t)buffer[i];
                state = (checksum == (uint8_t)checksumTest.get())? VALIDATE : WAIT;
                break;
            case VALIDATE:
                newValidPacket = buffer[i] == TRAME_END ? true : false;
                state = WAIT;
                break;
        }
        if(newValidPacket)
            return newValidPacket;
    }
    return 0;
}

