/*
 * EspSlave.h
 *
 *  Created on: Jul 10, 2023
 *  Author: fparent
 *  Brief: Create an object to interact with the esp slave and send command on the i2c bus
 */

#ifndef SRC_ESPSLAVE_H_
#define SRC_ESPSLAVE_H_

#include "ProtocolI2C.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include <string>

#define STR_DELIMITER 5
using namespace std;
using namespace ProtocolI2C;

class EspSlave {
public:
	struct InfosWifi
	{
		std::string ssid;
		std::string ip;
		uint8_t rssi;
	};

	EspSlave(uint16_t slaveAdress, I2C_HandleTypeDef& i2c);
	virtual ~EspSlave();

	bool putNewButton(uint16_t posx, uint16_t posy, uint16_t width, uint16_t height, uint16_t fillColor, uint16_t outlineColor);
	bool putNewString(std::string str, uint16_t posx, uint16_t posy, uint8_t textSize, uint16_t textColor, uint16_t bgColor);
	bool putNewTitle(std::string title);
	bool putNewJpeg(std::string filename, uint16_t posx, uint16_t posy, uint16_t maxLength, uint16_t maxHeight);
	bool putNewLogo(std::string filename);
	bool mqttConnect(std::string hostname, std::string username, std::string password, uint8_t qos);
	bool wifiConnect(std::string ssid, std::string password);
	bool mqttSubscribe(std::string topic);
	bool mqttPublish(std::string topic, std::string name, std::string value);
	bool setLabel(std::string idLabel, std::string str, uint16_t posx, uint16_t posy, uint8_t textSize, uint16_t textColor, uint16_t bgColor,uint8_t font, uint8_t outputState);

	optional<vector<uint8_t>> getButtonStates();
	optional<InfosWifi> getWifiInfos();
private:
	uint16_t slaveAdress;
	I2C_HandleTypeDef& i2c;
};

#endif /* SRC_ESPSLAVE_H_ */
