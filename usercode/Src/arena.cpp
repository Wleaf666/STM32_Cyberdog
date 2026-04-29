/**
 * @file    arena.cpp
 */
#include "static_arena.hpp"
#include <cstdio>


static StaticArena<3 * 1024> g_boot_arena;

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

// 调试用：检测内存使用情况
extern "C" bool arena_is_full()
{
    return g_boot_arena.used() >= g_boot_arena.capacity();
}

extern "C" void arena_reset()
{
    g_boot_arena.reset();
}

void operator delete(void *p) noexcept
{
    (void)p; // 静态 arena 不支持单独释放，此函数仅为链接器提供符号
}

void *operator new[](const std::size_t size)
{
    return ::operator new(size);
}

void operator delete[](void *p) noexcept
{
    ::operator delete(p);
}