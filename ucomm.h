//
// uComm
// A very minimal cross-platform serial port library
//
// https://github.com/matveyt/ucomm
//

#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// open port
// note: 8-N-1, no read timeout (immediate return)
// note: negative baud for EVEN parity
intptr_t ucomm_open(const char* port, int baud);
// intptr_t fd = ucomm_open("/dev/ttyUSB0", 115200);
// assert(fd != -1);

// close port
bool ucomm_close(intptr_t fd);
// assert(ucomm_close(fd));

// set read timeout
// note: on __unix__ timeout is rounded up to 100 ms
bool ucomm_timeout(intptr_t fd, unsigned ms);
// assert(ucomm_timeout(fd, 100));

// wait until output buffer is written
bool ucomm_drain(intptr_t fd);
// assert(ucomm_drain(fd));

// discard I/O buffers
bool ucomm_purge(intptr_t fd);
// assert(ucomm_purge(fd));

// set DTR and RTS (cf. "set" means pulldown)
bool ucomm_dtr(intptr_t fd, bool pulldown);
bool ucomm_rts(intptr_t fd, bool pulldown);
// ucomm_dtr(fd, true);
// ucomm_dtr(fd, false);

// get number of bytes in the input buffer
size_t ucomm_available(intptr_t fd);
// while (ucomm_available(fd) == 0)
//     do_other_things();

// read one byte from port
int ucomm_getc(intptr_t fd);
// int ch = ucomm_getc(fd);
// if (ch == -1)
//     fputs("Timeout\n", stderr);

// read data from port
size_t ucomm_read(intptr_t fd, void* buffer, size_t length);
// size_t cb = ucomm_read(fd, buf, sizeof(buf));
// if (cb == 0)
//     fputs("Timeout: no response\n", stderr);
// else if (cb < sizeof(buf))
//     fputs("Timeout: partial response\n", stderr);

// write one byte to port
bool ucomm_putc(intptr_t fd, int ch);
// assert(ucomm_putc(fd, 'A'));

// write data to port
size_t ucomm_write(intptr_t fd, const void* buffer, size_t length);
// assert(ucomm_write(fd, "Hello world!\n", 13) == 13);
