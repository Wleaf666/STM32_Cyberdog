/**
 * @file    arena.cpp
 */
#include "static_arena.hpp"
#include <cstdio>

// 定义一个 16KB 的静态分配器放在 .bss 区
static StaticArena<6 * 1024> g_boot_arena;

void *operator new(const std::size_t size)
{
    void *ptr = g_boot_arena.allocate(size);
    if (!ptr)
    {
        while (true)
        {
        } // 内存耗尽，直接挂起等待看门狗复位
    }
    return ptr;
}

void operator delete(void *p) noexcept {}

void *operator new[](const std::size_t size)
{
    return ::operator new(size);
}

void operator delete[](void *p) noexcept
{
    ::operator delete(p);
}