#ifndef PCPSERVICES_H_
#define PCPSERVICES_H_

//------------------------------------------ Includes ----------------------------------------------

#include "types/sdkTypes.h"
#include "platform/uart.h"
#include "comms/connectionMeta.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace IslSdk
{
	class PcpServices
	{
	public:
		virtual void open() = 0;
		virtual void close() = 0;
		virtual bool_t write(const uint8_t* data, uint_t size) = 0;
		virtual void setSerial(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits) = 0;
		virtual void setSerial(uint32_t baudrate) = 0;
		virtual bool_t setPower(bool_t on) = 0;
		virtual void setMode(Uart::Mode mode) = 0;
		virtual uint32_t getDeviceId() const = 0;
		virtual uint_t getIndex() const = 0;
		virtual ConnectionMeta getDeviceConnectionMeta() const = 0;
	};
}

//--------------------------------------------------------------------------------------------------
#endif
