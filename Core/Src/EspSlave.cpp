/*
 * EspSlave.cpp
 *
 *  Created on: Jul 10, 2023
 *      Author: fparent
 */

#include "EspSlave.h"

/**
 * @brief Default constructor of the object allowing to interact with the esp32 slave
 *
 * @param slave adress, the esp32 slave adress
 * @param i2c instance
 */
EspSlave::EspSlave(uint16_t slaveAdress, I2C_HandleTypeDef& i2c):slaveAdress(slaveAdress),i2c(i2c) {

}

/**
 * @brief Destructor of espSlave
 *
 */
EspSlave::~EspSlave() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief Put a new button on the touch screen controlled by the esp
 *
 * @param posx
 * @param posy
 * @param width
 * @param height
 * @param fillColor
 * @param outlineColor
 * @return true, new button rendered correctly
 * @return false, new button not rendered correctly or didn't receive response from the esp slave
 */
bool EspSlave::putNewButton(uint16_t posx, uint16_t posy, uint16_t width, uint16_t height, uint16_t fillColor, uint16_t outlineColor)
{
	uint8_t txBuffer[20];
	uint8_t dataButton[12] = {(uint8_t)(posx >> 8), (uint8_t)(posx), (uint8_t)(posy >> 8), (uint8_t)(posy),
		  	                  (uint8_t)(height >> 8), (uint8_t)(height), (uint8_t)(width >> 8), (uint8_t)(width),
		  	                  (uint8_t)(fillColor >> 8), (uint8_t)(fillColor), (uint8_t)(outlineColor >> 8), (uint8_t)(outlineColor)};
	protocolI2cEncode((char*)txBuffer,dataButton,0,12);
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 0 && reponseCommande.value().data.size() == 1)
		return reponseCommande.value().data[0];
	return 0;
}

/**
 * @brief Put a new string on the touch screen controlled by the esp
 *
 * @param str
 * @param posx
 * @param posy
 * @param textSize
 * @param textColor
 * @param bgColor
 * @return true, new string rendered correctly
 * @return false, new string not rendered correctly or didn't receive response from the esp slave
 */
bool EspSlave::putNewString(std::string str, uint16_t posx, uint16_t posy, uint8_t textSize, uint16_t textColor, uint16_t bgColor)
{
	uint8_t txBuffer[40];
	uint8_t dataStr[9 + str.length()] = {(uint8_t)(posx >> 8), (uint8_t)(posx), (uint8_t)(posy >> 8), (uint8_t)(posy),textSize,
		  	                             (uint8_t)(textColor >> 8), (uint8_t)(textColor), (uint8_t)(bgColor >> 8), (uint8_t)(bgColor)};
	for(int i = 0; i < str.length(); i++)
	{
		dataStr[9 + i] = str[i];
	}
	protocolI2cEncode((char*)txBuffer,dataStr,3,str.length() + 9);
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 40, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 3 && reponseCommande.value().data.size() == 1)
		return reponseCommande.value().data[0];
	return 0;
}
/**
 * @brief Set the parameters of an existing label on the screen with the corresponding id.
 * 			If the label doesn't exist, create the label with the given id and parameter
 *
 * @param str
 * @param posx
 * @param posy
 * @param textSize
 * @param textColor
 * @param bgColor
 * @return true, new label rendered correctly
 * @return false, new label not rendered correctly or didn't receive response from the esp slave
 */
bool EspSlave::setLabel(std::string idLabel, std::string str, uint16_t posx, uint16_t posy, uint8_t textSize, uint16_t textColor, uint16_t bgColor,uint8_t font, uint8_t outputState)
{
	uint8_t txBuffer[40];
	uint8_t dataLabel[str.length() + idLabel.length() + 12] = {(uint8_t)(posx >> 8), (uint8_t)(posx), (uint8_t)(posy >> 8), (uint8_t)(posy),textSize,
			  	                             (uint8_t)(textColor >> 8), (uint8_t)(textColor), (uint8_t)(bgColor >> 8), (uint8_t)(bgColor), font, outputState};
	for(int i = 0; i < str.length(); i++)
	{
		dataLabel[11 + i] = str[i];
	}
	dataLabel[str.length()+11] = STR_DELIMITER;
	for(int i = 0; i < idLabel.length(); i++)
	{
		dataLabel[str.length()+ 12 + i] = idLabel[i];
	}
	protocolI2cEncode((char*)txBuffer,dataLabel,12,str.length() + idLabel.length() + 12);
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 40, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 12 && reponseCommande.value().data.size() == 1)
		return reponseCommande.value().data[0];
	return 0;

}

/**
 * @brief Put a new title on the touch screen controlled by the esp
 *
 * @param title
 * @return true, new title rendered correctly
 * @return false, new title not rendered correctly or didn't receive response from the esp slave
 */
bool EspSlave::putNewTitle(std::string title)
{
	uint8_t txBuffer[40];
	uint8_t dataTitle[title.length()];
	for(int i = 0; i < title.length(); i++)
	{
	  	dataTitle[i] = title[i];
	}
	protocolI2cEncode((char*)txBuffer,dataTitle,4,title.length());
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 40, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 4 && reponseCommande.value().data.size() == 1)
		return reponseCommande.value().data[0];
	return 0;
}

/**
 * @brief Put a new jpeg on the touch screen controlled by the esp
 *
 * @param filename
 * @param posx
 * @param posy
 * @param maxLength
 * @param maxHeight
 * @return true, new jpeg rendered correctly
 * @return false, new jpeg not rendered correctly or didn't receive response from the esp slave
 */
bool EspSlave::putNewJpeg(std::string filename, uint16_t posx, uint16_t posy, uint16_t maxLength, uint16_t maxHeight)
{
	uint8_t txBuffer[40];
	uint8_t dataJpeg[8 + filename.length()] = {(uint8_t)(posx >> 8), (uint8_t)(posx), (uint8_t)(posy >> 8), (uint8_t)(posy),
			  	                                (uint8_t)(maxLength >> 8), (uint8_t)(maxLength), (uint8_t)(maxHeight >> 8), (uint8_t)(maxHeight)};
	for(int i = 0; i < filename.length(); i++)
	{
		dataJpeg[8 + i] = filename[i];
	}
	protocolI2cEncode((char*)txBuffer,dataJpeg,11,filename.length() + 8);
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 40, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 11 && reponseCommande.value().data.size() == 1)
		return reponseCommande.value().data[0];
	return 0;
}

/**
 * @brief Put a new logo on the touch screen controlled by the esp
 *
 * @param filename
 * @return true, new logo rendered correctly
 * @return false, new logo not rendered correctly or didn't receive response from the esp slave
 */
bool EspSlave::putNewLogo(std::string filename)
{
	uint8_t txBuffer[20];
	uint8_t dataLogo[filename.length()];
	for(int i = 0; i < filename.length(); i++)
	{
		dataLogo[i] = filename[i];
	}
	protocolI2cEncode((char*)txBuffer,dataLogo,5,filename.length());
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 5 && reponseCommande.value().data.size() == 1)
		return reponseCommande.value().data[0];
	return 0;
}

/**
 * @brief Connect the esp slave to a mqtt host
 *
 * @param hostname
 * @param username
 * @param password
 * @param qos
 * @return true, connect to mqtt host correctly
 * @return false, not connected to mqtt host or didn't get response from esp slave
 */
bool EspSlave::mqttConnect(std::string hostname, std::string username, std::string password, uint8_t qos)
{
	uint8_t txBuffer[60];
	uint8_t dataMqtt[hostname.length() + username.length() + password.length() + 3];
	dataMqtt[0] = 0;
	for(int i = 0; i < hostname.length(); i++)
	{
		dataMqtt[i + 1] = hostname[i];
	}
	dataMqtt[hostname.length()+1] = STR_DELIMITER;
	for(int i = 0; i < username.length(); i++)
	{
		dataMqtt[hostname.length() + i + 2] = username[i];
	}
	dataMqtt[hostname.length()+2+username.length()] = STR_DELIMITER;
	for(int i = 0; i < password.length(); i++)
	{
		dataMqtt[hostname.length() + username.length() + i + 3] = password[i];
	}
	protocolI2cEncode((char*)txBuffer,dataMqtt,10,hostname.length() + username.length() + password.length() + 3);
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 60, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 10 && reponseCommande.value().data.size() == 1)
		return reponseCommande.value().data[0];
	return 0;
}

/**
 * @brief Connect the esp slave to wifi
 *
 * @param ssid
 * @param password
 * @return true, wifi connected correctly
 * @return false, wifi not connected or didn't receive response from the esp slave
 */
bool EspSlave::wifiConnect(std::string ssid, std::string password)
{
	uint8_t txBuffer[40];
	uint8_t dataWifi[ssid.length() + password.length() + 1];

	for(int i = 0; i < ssid.length(); i++)
	{
		dataWifi[i] = ssid[i];
	}
	dataWifi[ssid.length()] = STR_DELIMITER;
	for(int i = 0; i < password.length(); i++)
	{
		dataWifi[ssid.length() + i + 1] = password[i];
	}
	protocolI2cEncode((char*)txBuffer,dataWifi,6,ssid.length() + password.length() + 1);
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 40, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 6 && reponseCommande.value().data.size() == 1)
		return reponseCommande.value().data[0];
	return 0;
}

/**
 * @brief Connect the esp slave to wifi
 *
 * @param ssid
 * @param password
 * @return true, wifi connected correctly
 * @return false, wifi not connected or didn't receive response from the esp slave
 */
bool EspSlave::mqttSubscribe(std::string topic)
{
	uint8_t txBuffer[40];
	uint8_t dataSub[topic.length()];
	for(int i = 0; i < topic.length(); i++)
	{
	    dataSub[i] = topic[i];
	}
	protocolI2cEncode((char*)txBuffer,dataSub,9,topic.length());
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 40, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 9 && reponseCommande.value().data.size() == 1)
		return reponseCommande.value().data[0];
	return 0;
}

/**
 * @brief Connect the esp slave to wifi
 *
 * @param ssid
 * @param password
 * @return true, wifi connected correctly
 * @return false, wifi not connected or didn't receive response from the esp slave
 */
bool EspSlave::mqttPublish(std::string topic, std::string name, std::string value)
{
	uint8_t txBuffer[60];
	uint8_t dataPub[topic.length() + name.length() + value.length() + 2];
	for(int i = 0; i < topic.length(); i++)
	{
		dataPub[i] = topic[i];
	}
	dataPub[topic.length()] = STR_DELIMITER;
	for(int i = 0; i < name.length(); i++)
	{
		dataPub[topic.length() + i + 1] = name[i];
	}
	dataPub[topic.length()+1+name.length()] = STR_DELIMITER;
	for(int i = 0; i < value.length(); i++)
	{
		dataPub[topic.length() + name.length() + i + 2] = value[i];
	}
	protocolI2cEncode((char*)txBuffer,dataPub,8,topic.length()+name.length()+value.length()+2);
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 60, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 8 && reponseCommande.value().data.size() == 1)
		return reponseCommande.value().data[0];
	return 0;
}

/**
 * @brief return the states of the button on the touch screen
 *
 * @param ssid
 * @param password
 * @return true, wifi connected correctly
 * @return false, wifi not connected or didn't receive response from the esp slave
 */
optional<vector<uint8_t>> EspSlave::getButtonStates()
{
	uint8_t txBuffer[40];
	uint8_t data;
	optional<vector<uint8_t>> buttonStates;
	protocolI2cEncode((char*)txBuffer,&data,1,0);
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 40, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);

	if(reponseCommande.has_value() && reponseCommande.value().id == 1)
	{
		buttonStates = reponseCommande.value().data;
		return buttonStates;
	}
	return std::nullopt;
}

/**
 * @brief Return the infos of the wifi the esp slave is connected to
 *
 * @return ssid, ip and rssi of the wifi (if connected)
 */
optional<EspSlave::InfosWifi> EspSlave::getWifiInfos()
{
	uint8_t txBuffer[40];
	uint8_t data;
	optional<InfosWifi> wifiInfos;
	InfosWifi infosWifi;
	protocolI2cEncode((char*)txBuffer,&data,7,0);
	HAL_I2C_Master_Transmit(&i2c, slaveAdress, txBuffer, 20, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&i2c, slaveAdress, txBuffer, 40, HAL_MAX_DELAY);
	optional<InfosCommande> reponseCommande = protocolI2cDecode((char*)txBuffer,20);
	if(reponseCommande.has_value() && reponseCommande.value().id == 7)
	{
		infosWifi.rssi = reponseCommande.value().data[0];
		for(int i = 1; (i < reponseCommande.value().data.size()) && reponseCommande.value().data[i] != STR_DELIMITER; i++)
		{
			infosWifi.ip.push_back(reponseCommande.value().data[i]);
		}
		for(int i = infosWifi.ssid.length() + 2; (i < reponseCommande.value().data.size()) && reponseCommande.value().data[i] != STR_DELIMITER; i++)
		{
			infosWifi.ssid.push_back(reponseCommande.value().data[i]);
		}
		wifiInfos = infosWifi;
		return wifiInfos;
	}
	return std::nullopt;
}
