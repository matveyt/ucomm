//
// uComm
// A very minimal cross-platform serial port library
//
// https://github.com/matveyt/ucomm
//

#include "ucomm.h"

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#elif defined(__unix__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

static unsigned baudrate(int baud)
{
    if (baud < 0)
        baud = -baud;
#if defined(_WIN32)
    return baud ? baud : 115200;
#elif defined(__unix__)
    if (baud != 0) {
        static const int ubr[] = {
            1200, B1200, 2400, B2400, 4800, B4800, 9600, B9600, 19200, B19200,
            38400, B38400, 57600, B57600, 115200, B115200, 230400, B230400,
            460800, B460800, 921600, B921600,
        };
        for (size_t i = 0; i < sizeof(ubr) / sizeof(ubr[0]); i += 2)
            if (baud <= ubr[i])
                return ubr[i + 1];
    }
    return B115200;
#endif
}

intptr_t ucomm_open(const char* port, int baud)
{
#if defined(_WIN32)
    char fullname[11];  // \\\\.\\COMxxx
    if (port == NULL) {
        // arbitrary value
        port = "\\\\.\\COM3";
    } else if (lstrlenA(port) <= 6) {
        // COMx => \\\\.\\COMx
        lstrcpyA(fullname, "\\\\.\\");
        port = lstrcatA(fullname, port);
    }

    HANDLE fd = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
        0, NULL);
    if (fd != INVALID_HANDLE_VALUE) {
        DCB dcb = {
            .DCBlength = sizeof(DCB),
            .BaudRate = baudrate(baud),
            .fBinary = 1,
            .fParity = (baud < 0),
            .fDtrControl = DTR_CONTROL_DISABLE,
            .fRtsControl = RTS_CONTROL_DISABLE,
            .ByteSize = 8,
            .Parity = (baud < 0) ? EVENPARITY : NOPARITY,
            .StopBits = ONESTOPBIT,
            .XonLim = 256,
            .XoffLim = 256,
        };
        SetupComm(fd, 1024, 1024);
        SetCommState(fd, &dcb);
        ucomm_timeout((intptr_t)fd, 0);
    }
#elif defined(__unix__)
    // arbitrary value
    if (port == NULL)
        port = "/dev/ttyUSB0";

    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd != -1) {
        struct termios tio;
        tcgetattr(fd, &tio);

        //cfmakeraw(&tio);
        tio.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | INPCK | ISTRIP | INLCR | IGNCR |
            ICRNL | IXON | PARMRK);
        tio.c_oflag &= ~(OPOST);
        tio.c_cflag &= ~(CSIZE | PARENB | PARODD);
        tio.c_lflag &= ~(ISIG | ICANON | ECHO | ECHONL | IEXTEN);
        tio.c_cflag |= (CS8 | CREAD | CLOCAL);
        if (baud < 0) {
            // EVEN parity
            tio.c_iflag |= INPCK;
            tio.c_cflag |= PARENB;
        }
        tio.c_cc[VMIN] = tio.c_cc[VTIME] = 0;
        unsigned ubr = baudrate(baud);
        cfsetispeed(&tio, ubr);
        cfsetospeed(&tio, ubr);
        tcsetattr(fd, TCSANOW, &tio);
    }
#endif

    return (intptr_t)fd;
}

bool ucomm_timeout(intptr_t fd, unsigned ms)
{
#if defined(_WIN32)
    COMMTIMEOUTS timeouts = {
        .ReadIntervalTimeout = ms ? ms : MAXDWORD,
        .ReadTotalTimeoutConstant = ms,
        .ReadTotalTimeoutMultiplier = 0,
        .WriteTotalTimeoutConstant = 0,
        .WriteTotalTimeoutMultiplier = 0,
    };
    return SetCommTimeouts((HANDLE)fd, &timeouts);
#elif defined(__unix__)
    struct termios tio;
    tcgetattr(fd, &tio);
    tio.c_cc[VTIME] = (ms / 100) + !!(ms % 100);
    return (tcsetattr(fd, TCSANOW, &tio) == 0);
#endif
}

bool ucomm_close(intptr_t fd)
{
#if defined(_WIN32)
    return CloseHandle((HANDLE)fd);
#elif defined(__unix__)
    return (close(fd) == 0);
#endif
}

bool ucomm_drain(intptr_t fd)
{
#if defined(_WIN32)
    return FlushFileBuffers((HANDLE)fd);
#elif defined(__unix__)
    return (tcdrain(fd) == 0);
#endif
}

bool ucomm_purge(intptr_t fd)
{
#if defined(_WIN32)
    return PurgeComm((HANDLE)fd, PURGE_RXCLEAR | PURGE_TXCLEAR);
#elif defined(__unix__)
    return (tcflush(fd, TCIOFLUSH) == 0);
#endif
}

bool ucomm_dtr(intptr_t fd, bool pulldown)
{
#if defined(_WIN32)
    return EscapeCommFunction((HANDLE)fd, pulldown ? SETDTR : CLRDTR);
#elif defined(__unix__)
    return (ioctl(fd, pulldown ? TIOCMBIS : TIOCMBIC, &(int){TIOCM_DTR}) == 0);
#endif
}

bool ucomm_rts(intptr_t fd, bool pulldown)
{
#if defined(_WIN32)
    return EscapeCommFunction((HANDLE)fd, pulldown ? SETRTS : CLRRTS);
#elif defined(__unix__)
    return (ioctl(fd, pulldown ? TIOCMBIS : TIOCMBIC, &(int){TIOCM_RTS}) == 0);
#endif
}

size_t ucomm_available(intptr_t fd)
{
#if defined(_WIN32)
    COMSTAT stat;
    return ClearCommError((HANDLE)fd, NULL, &stat) ? stat.cbInQue : 0;
#elif defined(__unix__)
    int avail = 0;
    ioctl(fd, FIONREAD, &avail);
    return avail;
#endif
}

int ucomm_getc(intptr_t fd)
{
    uint8_t b;
#if defined(_WIN32)
    DWORD part;
    ReadFile((HANDLE)fd, &b, sizeof(b), &part, NULL);
#elif defined(__unix__)
    ssize_t part = read(fd, &b, sizeof(b));
#endif
    return (part == sizeof(b)) ? (int)b : -1;
}

size_t ucomm_read(intptr_t fd, void* buffer, size_t length)
{
    size_t sz = 0;
    while (sz < length) {
#if defined(_WIN32)
        DWORD part;
        ReadFile((HANDLE)fd, (uint8_t*)buffer + sz, length - sz, &part, NULL);
#elif defined(__unix__)
        ssize_t part = read(fd, (uint8_t*)buffer + sz, length - sz);
#endif
        if (part > 0)
            sz += part;
        else
            break;
    }
    return sz;
}

bool ucomm_putc(intptr_t fd, int ch)
{
    uint8_t b = (uint8_t)ch;
#if defined(_WIN32)
    return WriteFile((HANDLE)fd, &b, sizeof(b), &(DWORD){0}, NULL);
#elif defined(__unix__)
    return (write(fd, &b, sizeof(b)) == sizeof(b));
#endif
}

size_t ucomm_write(intptr_t fd, const void* buffer, size_t length)
{
    size_t sz = 0;
    while (sz < length) {
#if defined(_WIN32)
        DWORD part;
        WriteFile((HANDLE)fd, (uint8_t*)buffer + sz, length - sz, &part, NULL);
#elif defined(__unix__)
        ssize_t part = write(fd, (uint8_t*)buffer + sz, length - sz);
#endif
        if (part > 0)
            sz += part;
        else
            break;
    }
    return sz;
}
