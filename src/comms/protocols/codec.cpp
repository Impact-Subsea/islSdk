//------------------------------------------ Includes ----------------------------------------------

#include "comms/protocols/codec.h"

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
Codec::Codec(uint_t bufSize, Type type) : type(type), m_frameBuf(bufSize)
{
}
//--------------------------------------------------------------------------------------------------
Codec::~Codec()
{
}
//--------------------------------------------------------------------------------------------------
