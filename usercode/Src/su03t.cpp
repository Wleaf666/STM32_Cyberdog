#include "su03t.hpp"

SU03T::SU03T(UART_HandleTypeDef *_huart, osMutexId_t _txMutex)
    : huart(_huart), txMutex(_txMutex)
{
}

bool SU03T::Init(osMessageQueueId_t _rxQueue)
{
    rxQueue = _rxQueue;
    rxBuffer.reserve(16); // 语音指令极短，16 字节缓存足够防溢出了

    // 注册 HAL 库的接收完成回调
    if (HAL_UART_RegisterCallback(huart, HAL_UART_RX_COMPLETE_CB_ID, SU03T_RxCallback_Wrapper) != HAL_OK)
    {
        return false;
    }

    // 开启第一次中断接收
    HAL_UART_Receive_IT(huart, &rxByte, 1);
    return true;
}

void SU03T::onRxCpltCallback()
{
    if (rxQueue != nullptr)
    {
        // 中断里极速办事：扔进队列
        osMessageQueuePut(rxQueue, &rxByte, 0, 0);
    }
    // 再次开启中断
    HAL_UART_Receive_IT(huart, &rxByte, 1);
}

VoiceCmd SU03T::getCommand()
{
    uint8_t byte;

    // 从队列抽水，超时为 0，绝对不阻塞当前任务
    while (osMessageQueueGet(rxQueue, &byte, NULL, 0) == osOK)
    {
        rxBuffer.push_back(byte);

        // 1. 找包头 0xAA，不是就抛弃
        if (rxBuffer.front() != 0xAA)
        {
            rxBuffer.clear();
            continue;
        }

        // 2. 找包尾 0x55，并且长度必须大于等于 3 (AA XX 55)
        if (byte == 0x55 && rxBuffer.size() >= 3)
        {
            VoiceCmd cmd = parsePacket(rxBuffer);
            rxBuffer.clear(); // 解析完立刻清空
            return cmd;
        }

        // 3. 缓冲区防爆破保护
        if (rxBuffer.size() > 16)
        {
            rxBuffer.clear();
        }
    }
    return VoiceCmd::NONE;
}

VoiceCmd SU03T::parsePacket(const std::vector<uint8_t> &packet)
{
    // 严格校验包长和头尾
    if (packet.size() == 3 && packet[0] == 0xAA && packet[2] == 0x55)
    {
        // 提取第 [1] 个字节作为命令码，转换为枚举返回
        return static_cast<VoiceCmd>(packet[1]);
    }
    return VoiceCmd::NONE;
}

void SU03T::playVoice(VoicePlay audio_id)
{
    // 组装发送给语音模块的指令包
    uint8_t txData[3] = {0xAA, static_cast<uint8_t>(audio_id), 0x55};

    if (txMutex != nullptr && osMutexAcquire(txMutex, osWaitForever) == osOK)
    {
        // 发送给模块，让它说话
        HAL_UART_Transmit(huart, txData, 3, 100);
        osMutexRelease(txMutex);
    }
}