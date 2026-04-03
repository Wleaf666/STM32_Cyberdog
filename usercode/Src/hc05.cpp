#include "hc05.hpp"
#include "cstring"



bool HC05::Init(osMessageQueueId_t _rxQueue)
{
    rxQueue = _rxQueue;
    rxBufffer.reserve(64);

    if (HAL_UART_RegisterCallback(huart, HAL_UART_RX_COMPLETE_CB_ID, HC05_RxCallback_Wrapper) !=HAL_OK)
    {
        return false;
    }

    HAL_UART_Receive_IT(huart, &rxByte, 1);
    return true;
}

void HC05::sendString(const char *str)
{
    if (txMutex != nullptr && osMutexAcquire(txMutex, osWaitForever) == osOK)
    {
        HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), 100);
        osMutexRelease(txMutex);
    }
}

void HC05::sendBytes(const uint8_t *data, uint16_t len)
{
    if (txMutex != nullptr && osMutexAcquire(txMutex, osWaitForever) == osOK)
    {
        HAL_UART_Transmit(huart, data, len, 100);
        osMutexRelease(txMutex);
    }
}

void HC05::onRxCpltCallback()
{
    if (rxQueue != nullptr)
    {
        osMessageQueuePut(rxQueue, &rxByte, 0, 0);
    }

    HAL_UART_Receive_IT(huart, &rxByte,1);
}

bool HC05::getCommand(RobotCommand &cmd)
{
    uint8_t byte;
    while (osMessageQueueGet(rxQueue, &byte, NULL, 1) == osOK)
    {
        rxBufffer.push_back(byte);

        if (rxBufffer.front() != 0xAA)
        {
            rxBufffer.clear();
            continue;
        }

        if (byte == 0x55)
        {
            bool valid = parasePacket(rxBufffer, cmd);
            rxBufffer.clear();
            return valid;
        }

        if (rxBufffer.size() > 64)
        {
            rxBufffer.clear();
        }
        
    }
    return false;
}

bool HC05::parasePacket(const std::vector<uint8_t> &packet, RobotCommand &out_cmd)
{

    return true;
}
