#ifndef QUEUE_H_
#define QUEUE_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include <atomic>

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
    class Queue
    {
    public:
        Queue(uint_t size);
        ~Queue();
        void reset();
        void* newItem(uint_t size);
        void cancelNewItem();
        bool_t reduceSize(uint_t newSize);
        void push();
        void* peekNextItem();
        void* peekNextItem(void* item);
        void pop();
        uint_t itemCount();

    private:
        uint_t m_bufSize;
        uint8_t* m_buf;
        std::atomic<uint8_t*> m_head;
        std::atomic<uint8_t*> m_tail;
        std::atomic<uint8_t*> m_end;
        uint8_t* m_headTemp;
        std::atomic<uint_t> m_itemsPushed;
        std::atomic<uint_t> m_itemsPopped;
    };
}
//--------------------------------------------------------------------------------------------------
#endif
