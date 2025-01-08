//------------------------------------------ Includes ----------------------------------------------

#include "serialPort.h"
#include "platform/debug.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <poll.h>
#include <cstring>

// Ensure this app has permission to access serial ports. Add the user to the dialout group. "sudo usermod -a -G dialout USER_NAME"

using namespace IslSdk;

int_t strCopyMax(char* dst, const char* src, int_t maxLength);
speed_t convertToUnixBaudRate(uint32_t baudrate);

//--------------------------------------------------------------------------------------------------
std::vector<std::string> SerialPort::getNames()
{
    std::vector<std::string> nameList;
    char path[1024] = "/sys/class/tty";

    DIR* dirh = opendir(&path[0]);
    if (dirh != nullptr)
    {
        char* ttyName = &path[std::strlen(&path[0])];
        strCopyMax(ttyName, "/", 2);
        ttyName++;
        struct dirent* fileInfo;

        while ((fileInfo = readdir(dirh)) != 0)
        {
            if (fileInfo->d_name[0] == 't' && fileInfo->d_name[1] == 't' && fileInfo->d_name[2] == 'y')
            {
                uint_t ttyNameSize = strCopyMax(ttyName, fileInfo->d_name, sizeof(path) - (ttyName - &path[0]));
                char* str = ttyName + ttyNameSize;
                str += strCopyMax(str, "/device", sizeof(path) - (str - &path[0]));

                struct stat st;
                if (lstat(&path[0], &st) == 0 && S_ISLNK(st.st_mode))
                {
                    str += strCopyMax(str, "/driver", sizeof(path) - (str - &path[0]));
                    int_t length = readlink(&path[0], str, sizeof(path) - (str - &path[0]) - 1);

                    if (length > 0)
                    {
                        str[length] = 0;
                        str = std::strrchr(str, '/');
                        if (str++ != nullptr)
                        {               
                            std::string portName = "/dev/" + std::string(ttyName, ttyNameSize);
                            int_t fd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);

                            if (fd < 0)
                            {
                                continue;
                            }

                            struct termios tty;
                            if (tcgetattr(fd, &tty) < 0)
                            {
                                ::close(fd);
                                continue;
                            }

                            ::close(fd);

                            nameList.push_back(portName);
                        }
                    }
                }
            }
            *ttyName = 0;
        }
        closedir(dirh);
    }

    return nameList;
}
//--------------------------------------------------------------------------------------------------
int_t strCopyMax(char* dst, const char* src, int_t maxLength)
{
    int_t count = 0;

    while (*src && (count < maxLength))
    {
        *dst++ = *src++;
        count++;
    }
    *dst = 0;

    return count;
}
//--------------------------------------------------------------------------------------------------
SerialPort::SerialPort(const std::string& name) : Uart(name)
{
    m_portHandle = -1;
    m_rxBuf = nullptr;
    m_rxBufSize = 0;
    m_txBaudrate = 9600;
    m_baudrate = m_txBaudrate;
    m_dataBits = 8;
    m_parity = Uart::Parity::None;
    m_stopBits = Uart::StopBits::One;
    m_runRxthread = false;
}
//--------------------------------------------------------------------------------------------------
SerialPort::~SerialPort()
{
    if (m_portHandle >= 0)
    {
        m_runRxthread = false;
        if (m_threadRx.joinable())
        {
            m_threadRx.join();
        }
        ::close(m_portHandle);
        m_portHandle = -1;
        m_rxBuf = nullptr;
        m_rxBufSize = 0;
    }
    uartEvent(Uart::Events::Close);
}
//--------------------------------------------------------------------------------------------------
bool_t SerialPort::open()
{
    if (m_portHandle >= 0)
    {
        close();
    }
	
    int_t fd = ::open(name.c_str(), O_RDWR | O_NOCTTY);

    if (fd >= 0)
    {
        m_portHandle = fd;
        if (config(m_txBaudrate, m_dataBits, m_parity, m_stopBits))
        {
            m_runRxthread = true;
            m_threadRx = std::thread(&SerialPort::threadRx, this);
        }
        else
        {
            ::close(m_portHandle);
            m_portHandle = -1;
            fd = -1;
        }
    }
    else
    {
        debugLog("Uart", "Error opening uart %s: %s", name.c_str(), strerror(errno));
        debugLog("Uart", "Ensure this app has permission to access serial ports.Add the user to the dialout group. sudo usermod - a - G dialout USER_NAME");
    }

    uartEvent(Uart::Events::Open);
    return fd >= 0;
}
//--------------------------------------------------------------------------------------------------
void SerialPort::close()
{
    if (m_portHandle >= 0)
    {
        m_runRxthread = false;
        if (m_threadRx.joinable())
        {
            m_threadRx.join();
        }
        ::close(m_portHandle);
        m_portHandle = -1;
        m_rxBuf = nullptr;
        m_rxBufSize = 0;
    }
    uartEvent(Uart::Events::Close);
}
//--------------------------------------------------------------------------------------------------
bool_t SerialPort::isOpen() const
{
    return m_portHandle >= 0;
}
//--------------------------------------------------------------------------------------------------
bool_t SerialPort::config(uint32_t baudrate, uint8_t dataBits, Uart::Parity parity, Uart::StopBits stopBits)
{
    struct termios tty;
    uint32_t parityLut[] = { 0, PARENB | PARODD, PARENB, 0 , 0 };
    uint32_t dataBitsLut[] = { CS5, CS6, CS7, CS8 };
    uint32_t stopBitsLut[] = { 0, CSTOPB, CSTOPB };

    if (dataBits < 5 || dataBits > 8)
    {
        dataBits = 8;
    }
    dataBits -= 5;

    if (tcgetattr(m_portHandle, &tty) < 0)
    {
        debugLog("Uart", "Error from uart tcgetattr: %s", strerror(errno));
        return false;
    }

    speed_t baud = convertToUnixBaudRate(baudrate);
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);
    tty.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_cflag |= CLOCAL | CREAD | dataBitsLut[dataBits] | parityLut[static_cast<uint_t>(parity)] | stopBitsLut[static_cast<uint_t>(stopBits)];
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(m_portHandle, TCSANOW, &tty))
    {
        debugLog("Uart", "Error from uart tcsetattr: %s", strerror(errno));
        return false;
    }

    m_baudrate = baudrate;
    m_dataBits = dataBits;
    m_parity = parity;
    m_stopBits = stopBits;

    return true;
}
//--------------------------------------------------------------------------------------------------
speed_t convertToUnixBaudRate(uint32_t baudrate)
{
    if (baudrate == 115200) return B115200;
    if (baudrate == 57600) return B57600;
    if (baudrate == 38400) return B38400;
    if (baudrate == 19200) return B19200;
    if (baudrate == 9600) return B9600;
    if (baudrate == 4800) return B4800;
    if (baudrate == 2400) return B2400;
    if (baudrate == 1800) return B1800;
    if (baudrate == 1200) return B1200;
    if (baudrate == 600) return B600;
    if (baudrate == 300) return B300;
    if (baudrate == 200) return B200;
    if (baudrate == 150) return B150;
    if (baudrate == 134) return B134;
    if (baudrate == 110) return B110;
    if (baudrate == 75) return B75;
    if (baudrate == 50) return B50;
    return B0;
}
//--------------------------------------------------------------------------------------------------
bool_t SerialPort::write(const uint8_t* buf, uint_t size, uint32_t baudrate)
{
    if (m_portHandle >= 0)
    {
        if (baudrate && baudrate != m_baudrate)
        {
            if (!config(baudrate, m_dataBits, m_parity, m_stopBits))
            {
                close();
                uartEvent(Uart::Events::Error);
                return false;
            }
        }

        if (buf && size)
        {
            int_t bytesWritten = ::write(m_portHandle, buf, size);
            if (bytesWritten < 0)
            {
                close();
                uartEvent(Uart::Events::Error);
                return false;
            }
            txCompleteEvent(buf, bytesWritten);
        }

        return true;
    }

    return false;
}
//--------------------------------------------------------------------------------------------------
void SerialPort::setRxBuffer(uint8_t* buf, uint_t size)
{
    m_rxBuf = buf;
    m_rxBufSize = size;
}
//--------------------------------------------------------------------------------------------------
void SerialPort::threadRx()
{
    bool_t error = false;
    struct pollfd ufds;
    ufds.fd = m_portHandle;
    ufds.events = POLLIN;

    while (m_runRxthread && !error)
    {
        int_t res = poll(&ufds, 1, 50);
        if (res > 0 && m_rxBuf && m_rxBufSize)
        {
            int_t bytesRead = read(m_portHandle, m_rxBuf, m_rxBufSize);

            if (bytesRead > 0)
            {
                rxDataEvent(m_rxBuf, bytesRead, m_baudrate);
            }
            else if (bytesRead < 0)
            {
                error = true;
            }
        }
        else if (res < 0)
        {
            error = true;
        }
    }

    if (error)
    {
        close();
        uartEvent(Uart::Events::Error);
    }
}
//--------------------------------------------------------------------------------------------------
