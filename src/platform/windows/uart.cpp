//------------------------------------------ Includes ----------------------------------------------

#include "uart.h"
#include <aclapi.h>
#include <winerror.h>
#include <array>

#pragma comment(lib, "advapi32.lib")

using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
std::vector<std::string> Uart::getNames()
{
    HKEY regKey;
    LSTATUS moreData = 1;
    DWORD idx = 0;
    std::vector<std::string> nameList;

    nameList.clear();

    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_QUERY_VALUE, &regKey) == ERROR_SUCCESS)
    {
        while (moreData)
        {
            WCHAR portName[32];
            std::array<WCHAR, 64> name;
            DWORD nameSize = static_cast<DWORD>(name.size());
            DWORD portNameSize = static_cast<DWORD>(sizeof(portName));

            moreData = RegEnumValueW(regKey, idx, &name[0], &nameSize, 0, 0, (LPBYTE)&portName[0], &portNameSize) == ERROR_SUCCESS;
            idx++;

            if ((portNameSize != sizeof(portName)))
            {
                uint8_t utf8Buf[16];
                portNameSize = WideCharToMultiByte(CP_UTF8, WC_NO_BEST_FIT_CHARS, &portName[0], -1, (LPSTR)&utf8Buf[0], sizeof(utf8Buf), NULL, NULL);
                nameList.emplace_back((const char*)&utf8Buf[0]);
            }
        }
        RegCloseKey(regKey);
    }

    return nameList;
}
//--------------------------------------------------------------------------------------------------
Uart::Uart(const std::string& name) : name(name)
{
    m_rxBuf = nullptr;
    m_rxBufSize = 0;
    m_txBuf = nullptr;
    m_txBufSize = 0;
    m_txBaudrate = 9600;
    m_isSending = false;
    m_portHandle = INVALID_HANDLE_VALUE;
    m_txStartEvent = INVALID_HANDLE_VALUE;
    m_baudrate = m_txBaudrate;
    m_dataBits = 8;
    m_parity = Uart::Parity::None;
    m_stopBits = Uart::StopBits::One;
}
//--------------------------------------------------------------------------------------------------
Uart::~Uart()
{
    close();
}
//--------------------------------------------------------------------------------------------------
bool_t Uart::open()
{
    HANDLE handle;
    std::array<WCHAR, 32> str = { L"\\\\.\\" };

    if (!MultiByteToWideChar(CP_UTF8, MB_PRECOMPOSED, name.c_str(), -1, &str[4], static_cast<int>(str.size() - 4)))
    {
        return false;
    }

    handle = CreateFileW(&str[0], GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);

    if (handle != INVALID_HANDLE_VALUE)
    {
        m_portHandle = handle;
        COMMTIMEOUTS commTimeOut;
        commTimeOut.ReadIntervalTimeout = 1;
        commTimeOut.ReadTotalTimeoutConstant = 0;
        commTimeOut.ReadTotalTimeoutMultiplier = 0;
        commTimeOut.WriteTotalTimeoutMultiplier = 2;
        commTimeOut.WriteTotalTimeoutConstant = 500;
        SetCommTimeouts(m_portHandle, &commTimeOut);

        DCB dcb;
        dcb.DCBlength = sizeof(DCB);
        GetCommState(m_portHandle, &dcb);
        dcb.BaudRate = m_baudrate;
        dcb.fBinary = 1;
        dcb.fParity = m_parity != Uart::Parity::None;
        dcb.fOutxCtsFlow = 0;
        dcb.fOutxDsrFlow = 0;
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fDsrSensitivity = 0;
        dcb.fTXContinueOnXoff = 0;
        dcb.fOutX = 0;
        dcb.fInX = 0;
        dcb.fErrorChar = 0;
        dcb.fNull = 0;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        dcb.fAbortOnError = 0;
        dcb.ByteSize = m_dataBits;
        dcb.Parity = static_cast<BYTE>(m_parity);
        dcb.StopBits = static_cast<BYTE>(m_stopBits);

        SetCommState(m_portHandle, &dcb);
        SetCommMask(m_portHandle, EV_RXCHAR);

        m_txStartEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
        m_threadRxTx = std::thread(&Uart::threadRxTx, this);
    }

    return handle != INVALID_HANDLE_VALUE;
}
//--------------------------------------------------------------------------------------------------
void Uart::close()
{
    if (m_portHandle != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_portHandle);
        CloseHandle(m_txStartEvent);
        m_threadRxTx.join();
        m_portHandle = INVALID_HANDLE_VALUE;
        m_rxBuf = nullptr;
        m_rxBufSize = 0;
        m_txBuf = nullptr;
        m_txBufSize = 0;
        m_isSending = false;
    }
}
//--------------------------------------------------------------------------------------------------
bool_t Uart::config(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits)
{
    DCB dcbCommPort;
    dcbCommPort.DCBlength = sizeof(DCB);
    GetCommState(m_portHandle, &dcbCommPort);
    dcbCommPort.BaudRate = baudrate;
    dcbCommPort.fParity = parity != Uart::Parity::None;
    dcbCommPort.Parity = static_cast<BYTE>(parity);
    dcbCommPort.ByteSize = dataBits;
    dcbCommPort.StopBits = static_cast<BYTE>(stopBits);

    bool_t ok = SetCommState(m_portHandle, &dcbCommPort) != 0;
    PurgeComm(m_portHandle, PURGE_RXCLEAR);

    if (ok)
    {
        m_baudrate = baudrate;
        m_dataBits = dataBits;
        m_parity = parity;
        m_stopBits = stopBits;
    }

    return ok;
}
//--------------------------------------------------------------------------------------------------
bool_t Uart::write(const uint8_t* buf, uint_t size, uint32_t baudrate)
{
    if (m_portHandle != INVALID_HANDLE_VALUE && !m_isSending)
    {
        m_txBuf = buf;
        m_txBufSize = size;
        m_txBaudrate = baudrate;
        m_isSending = true;
        SetEvent(m_txStartEvent);
        return true;
    }

    return false;
}
//--------------------------------------------------------------------------------------------------
void Uart::setRxBuffer(uint8_t* buf, uint_t size)
{
    m_rxBuf = buf;
    m_rxBufSize = size;
}
//--------------------------------------------------------------------------------------------------
void Uart::threadRxTx()
{
    OVERLAPPED ovWaitOnRx = {0};
    OVERLAPPED ovRead = { 0 };
    OVERLAPPED ovWrite = { 0 };
    std::array<HANDLE, 4> events;
    bool_t error = false;
    DWORD commEvent = 0;
    DWORD bytesRead = 0;
    DWORD byteWritten = 0;
    bool_t setRxListenEvent = true;

    ovWaitOnRx.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    ovRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    ovWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    events[0] = ovWaitOnRx.hEvent;
    events[1] = ovRead.hEvent;
    events[2] = m_txStartEvent;
    events[3] = ovWrite.hEvent;

    while (m_portHandle != INVALID_HANDLE_VALUE && !error)
    {
        if (setRxListenEvent)
        {
            setRxListenEvent = false;
            if (WaitCommEvent(m_portHandle, &commEvent, &ovWaitOnRx))
            {
                if (commEvent & EV_RXCHAR)
                {
                    SetEvent(ovWaitOnRx.hEvent);
                }
            }
            else if (GetLastError() != ERROR_IO_PENDING)
            {
                error = true;
                break;
            }
        }

        DWORD result = WaitForMultipleObjects(static_cast<DWORD>(events.size()), &events[0], FALSE, INFINITE);

        switch (result)
        {
        case WAIT_OBJECT_0:

            if (GetOverlappedResult(m_portHandle, &ovWaitOnRx, &bytesRead, FALSE))
            {
                resetOv(ovWaitOnRx);
                setRxListenEvent = true;
                if ((commEvent & EV_RXCHAR) && m_rxBuf && m_rxBufSize)
                {
                    if (ReadFile(m_portHandle, m_rxBuf, static_cast<DWORD>(m_rxBufSize), &bytesRead, &ovRead))
                    {
                        resetOv(ovRead);
                        rxDataEvent(m_rxBuf, bytesRead, m_baudrate);
                    }
                    else
                    {
                        if (GetLastError() != ERROR_IO_PENDING)
                        {
                            error = true;
                        }
                        else
                        {
                            setRxListenEvent = false;
                        }
                    }
                }
            }
            else
            {
                error = true;
            }
            break;

        case WAIT_OBJECT_0 + 1:

            setRxListenEvent = true;
            if (GetOverlappedResult(m_portHandle, &ovRead, &bytesRead, FALSE))
            {
                resetOv(ovRead);
                rxDataEvent(m_rxBuf, bytesRead, m_baudrate);
            }
            else
            {
                error = true;
            }
            break;

        case WAIT_OBJECT_0 + 2:
            ResetEvent(m_txStartEvent);
            if (m_txBaudrate != m_baudrate)
            {
                if (!config(m_txBaudrate, 8, Uart::Parity::None, Uart::StopBits::One))
                {
                    error = true;
                    break;
                }
            }
            if (m_txBuf && m_txBufSize)
            {
                if (WriteFile(m_portHandle, m_txBuf, static_cast<DWORD>(m_txBufSize), &byteWritten, &ovWrite))
                {
                    resetOv(ovWrite);
                    m_isSending = false;
                    txCompleteEvent(m_txBuf, byteWritten);
                }
                else if (GetLastError() != ERROR_IO_PENDING)
                {
                    error = true;
                }
            }
            else
            {
                m_isSending = false;
            }
            break;

        case WAIT_OBJECT_0 + 3:
            if (GetOverlappedResult(m_portHandle, &ovWrite, &byteWritten, FALSE))
            {
                resetOv(ovWrite);
                m_isSending = false;
                txCompleteEvent(m_txBuf, byteWritten);
            }
            else
            {
                error = true;
            }
            break;

        case WAIT_TIMEOUT:
            break;
        }
    }

    if (error)
    {
        errorEvent();
    }

    CloseHandle(ovWaitOnRx.hEvent);
    CloseHandle(ovRead.hEvent);
    CloseHandle(ovWrite.hEvent);
}
//--------------------------------------------------------------------------------------------------
void Uart::resetOv(OVERLAPPED& ov)
{
    ov.Internal = 0;
    ov.InternalHigh = 0;
    ov.Offset = 0;
    ov.OffsetHigh = 0;
    ov.Pointer = nullptr;

    if (ov.hEvent != INVALID_HANDLE_VALUE)
    {
        ResetEvent(ov.hEvent);
    }
}
//--------------------------------------------------------------------------------------------------
