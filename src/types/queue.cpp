//------------------------------------------ Includes ----------------------------------------------

#include "queue.h"
#include "platform/debug.h"

using namespace IslSdk;

constexpr uint_t memoryAligment = sizeof(uint_t);

//--------------------------------------------------------------------------------------------------
Queue::Queue(uint_t size)
{
    m_bufSize = size;
    m_buf = new uint8_t[m_bufSize];
    reset();
}
//--------------------------------------------------------------------------------------------------
Queue::~Queue()
{
    if (m_buf)
    {
        delete[] m_buf;
    }
}
//--------------------------------------------------------------------------------------------------
void Queue::reset()
{
    m_head = m_buf;
    m_tail = m_buf;
    m_end = nullptr;
    m_headTemp = m_buf;
    m_itemsPushed = 0;
    m_itemsPopped = 0;
}
//--------------------------------------------------------------------------------------------------
uint_t Queue::itemCount()
{
    return m_itemsPushed - m_itemsPopped;        // works when numbers rollover because they are uint_t
}
//--------------------------------------------------------------------------------------------------
void* Queue::newItem(uint_t size)
{
    uint8_t* mem = nullptr;
    uint8_t* end;
    uint_t freeMem;

    if (size)
    {
        push();

        if (m_head >= m_tail)
        {
            end = &m_buf[m_bufSize - 1];
        }
        else
        {
            end = m_tail;
        }

        freeMem = (uint_t)(end - m_head);

        size += sizeof(uint_t);
        size += (~size + 1) & (memoryAligment - 1);

        if ((size >= freeMem) && (m_head >= m_tail))
        {
            freeMem = (uint_t)(m_tail - m_buf);
            if (size < freeMem)
            {
                m_end = m_head.load();
                m_headTemp = m_buf;
            }
        }

        if (size < freeMem)
        {
            mem = m_headTemp;
            m_headTemp += size;
            *((uint_t*)mem) = size;
            mem += sizeof(uint_t);
        }
        else
        {
            //debugLog("Queue", "out of memory - buf: %p, head: %p, headTemp: %p, tail: %p, end: %p, SIZE: %u, freeMem: %u", m_buf, m_head, m_headTemp, m_tail, m_end, size, freeMem);
        }
    }
    return (void*)mem;
}
//--------------------------------------------------------------------------------------------------
void Queue::cancelNewItem()
{
    m_headTemp = m_head.load();
}
//--------------------------------------------------------------------------------------------------
bool_t Queue::reduceSize(uint_t newSize)
{
    int_t dif = -1;
    uint_t* size;

    if (m_head != m_headTemp)
    {
        newSize += sizeof(uint_t);
        newSize += (~newSize + 1) & (memoryAligment - 1);

        if (m_end == m_head)
        {
            size = (uint_t*)m_buf;
        }
        else
        {
            size = (uint_t*)m_head.load();
        }

        dif = *size - newSize;

        if (dif > 0)
        {
            m_headTemp -= dif;
            *size -= dif;
        }
    }

    return dif >= 0;
}
//--------------------------------------------------------------------------------------------------
void Queue::push()
{
    if (m_head != m_headTemp)
    {
        m_head = m_headTemp;
        m_itemsPushed++;
    }
}
//--------------------------------------------------------------------------------------------------
void* Queue::peekNextItem()
{
    void* mem = nullptr;

    if (m_head != m_tail)
    {
        if (m_tail == m_end)
        {
            m_tail = m_buf;
            m_end = nullptr;
        }

        mem = (void*)(m_tail + sizeof(uint_t));
    }

    return mem;
}
//--------------------------------------------------------------------------------------------------
void* Queue::peekNextItem(void* item)
{
    void* mem = nullptr;

    if (item)
    {
        uint8_t* ptr = (uint8_t*)item - sizeof(uint_t);

        if (ptr != nullptr)
        {
            ptr += *((uint_t*)ptr);

            if (ptr == m_end)
            {
                ptr = m_buf;
            }

            if (ptr != m_head)
            {
                mem = (ptr + sizeof(uint_t));
            }
        }
    }
    else
    {
        mem = peekNextItem();
    }

    return mem;
}
//--------------------------------------------------------------------------------------------------
void Queue::pop()
{
    if (m_head != m_tail)
    {
        if (m_tail == m_end)
        {
            m_tail = m_buf;
            m_end = nullptr;
        }

        m_tail += *((uint_t*)m_tail.load());
        m_itemsPopped++;
    }
}
//--------------------------------------------------------------------------------------------------
