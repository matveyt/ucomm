This is a bare minimum cross-platform serial port library. It is intended to work with
USB-TTL programmers only. Almost no features and no documentation.

Complete function list:
```
intptr_t ucomm_open(const char* port, int baud);
bool ucomm_close(intptr_t fd);
bool ucomm_timeout(intptr_t fd, unsigned ms);
bool ucomm_drain(intptr_t fd);
bool ucomm_purge(intptr_t fd);
bool ucomm_dtr(intptr_t fd, bool pulldown);
bool ucomm_rts(intptr_t fd, bool pulldown);
size_t ucomm_available(intptr_t fd);
int ucomm_getc(intptr_t fd);
size_t ucomm_read(intptr_t fd, void* buffer, size_t length);
bool ucomm_putc(intptr_t fd, int ch);
size_t ucomm_write(intptr_t fd, const void* buffer, size_t length);
```
