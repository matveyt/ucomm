This is a bare minimum cross-platform serial port library. It is intended to work with
USB-TTL programmers only. Blocking read only, almost no features and no documentation.

Complete function list:
```
intptr_t ucomm_open(const char* port, unsigned baud, unsigned config);
int ucomm_close(intptr_t fd);
int ucomm_timeout(intptr_t fd, unsigned ms);
int ucomm_purge(intptr_t fd);
int ucomm_dtr(intptr_t fd, int pulldown);
int ucomm_rts(intptr_t fd, int pulldown);
ssize_t ucomm_available(intptr_t fd);
int ucomm_getc(intptr_t fd);
int ucomm_putc(intptr_t fd, int ch);
ssize_t ucomm_read(intptr_t fd, void* buffer, size_t length);
ssize_t ucomm_write(intptr_t fd, const void* buffer, size_t length);
char** ucomm_ports(void);
```
