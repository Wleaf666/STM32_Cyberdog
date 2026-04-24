#include "hc05.hpp"
#include "cstring"



bool HC05::Init(osMessageQueueId_t _rxQueue)
{
    rxQueue = _rxQueue;
    rxBuffer.reserve(64);

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
    while (osMessageQueueGet(rxQueue, &byte, NULL, 0) == osOK)
    {
        // 🛡️ 终极装甲：在塞入数据前检查！只要满了，立刻清空，绝对不给 vector 扩容的机会！
        if (rxBuffer.size() >= 64)
        {
            rxBuffer.clear();
        }

        rxBuffer.push_back(byte);

        if (rxBuffer.front() != 0xAA)
        {
            rxBuffer.clear();
            continue;
        }

        if (byte == 0x55)
        {
            bool valid = parsePacket(rxBuffer, cmd);
            rxBuffer.clear();
            return valid;
        }
    }
    return false;
}

bool HC05::parsePacket(const std::vector<uint8_t> &packet, RobotCommand &out_cmd)
{
    if (packet.size() == 3 && packet[0] == 0xAA && packet[2] == 0x55)
    {
        out_cmd.cmd_type = packet[1]; // 提取中间的控制码 (比如 0x01)
        return true;
    }
    return false;
}
