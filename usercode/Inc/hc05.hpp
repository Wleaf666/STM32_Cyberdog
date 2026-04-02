#pragma once

#include "main.h"
#include "cmsis_os2.h"
#include <vector>

struct RobotCommand
{
    uint8_t cmd_type;
    float param1;
    float param2;
    float param3;
};


class HC05
{
    private:
        UART_HandleTypeDef *huart;
        
        osMutexId_t txMutex;
        osMessageQueueId_t rxQueue;

        std::vector<uint8_t> rxBufffer;
        uint8_t rxByte;

        bool parasePacket(const std::vector<uint8_t> &packet, RobotCommand &out_cmd);

    public:
        HC05(UART_HandleTypeDef *_huart,osMutexId_t _txMutex):huart(_huart),txMutex(_txMutex){}

        bool Init(osMessageQueueId_t _rxQueue);

        void sendString(const char *str);

        void sendBytes(const uint8_t *data, uint16_t len);

        void onRxCpltCallback();

        bool getCommand(RobotCommand &cmd);
};

void HC05_RxCallback_Wrapper(UART_HandleTypeDef *huart);