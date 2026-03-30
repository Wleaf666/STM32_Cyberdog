#pragma once
#include <cstddef>
#include <cstdint>

template <std::size_t Size>
class StaticArena
{
private:
    uint8_t buffer[Size];
    std::size_t offset = 0;

public:
    void *allocate(std::size_t size)
    {
        // 简单的按 4 字节对齐
        std::size_t aligned_size = (size + 3) & ~3;
        if (offset + aligned_size <= Size)
        {
            void *ptr = &buffer[offset];
            offset += aligned_size;
            return ptr;
        }
        return nullptr; // 内存不足
    }
};