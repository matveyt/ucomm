//
// uComm
// A very minimal cross-platform serial port library
//
// https://github.com/matveyt/ucomm
//

#include "ucomm.h"
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#elif defined(__unix__)
#include <dirent.h>
#include <unistd.h>
#endif

static int strntest(const char* str, const char* test, size_t n);

char** ucomm_ports(void)
{
    char** result = NULL;
    size_t n = 0, sz = 0, offset = 0;

#if defined(_WIN32)
    HKEY hKey = NULL;
    do {
        if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0,
                KEY_QUERY_VALUE, &hKey) != ERROR_SUCCESS)
            break;

        // get number of values (devices)
        DWORD cValues, cbMaxValueLen;
        if (RegQueryInfoKeyA(hKey, NULL, NULL, NULL, NULL, NULL, NULL, &cValues, NULL,
                &cbMaxValueLen, NULL, NULL) != ERROR_SUCCESS || cValues == 0)
            break;

        // alloc memory for (cValues + 1) pointers and cValues strings (Cf. argv[])
        offset = (cValues + 1) * sizeof(char*);
        sz = offset + cValues * (cbMaxValueLen + 1);
        result = malloc(sz);
        if (result == NULL)
            break;

        do {
            // read port value
            char device[MAX_PATH];
            DWORD dwType;
            char* value = (char*)result + offset;
            DWORD cb = cbMaxValueLen;
            if (RegEnumValueA(hKey, n, device, &(DWORD){MAX_PATH}, NULL, &dwType,
                    (LPBYTE)value, &cb) != ERROR_SUCCESS || dwType != REG_SZ)
                break;

            // append value pointer
            result[n] = value;
            if (value[cb - 1] != '\0') {
                value[cb] = '\0';
                ++offset;
            }
            offset += cb;
        } while (++n < cValues);
    } while(0);
    if (hKey != NULL)
        RegCloseKey(hKey);

#elif defined(__unix__)
    // try sysfs, fallback to scanning /dev
    DIR* dir = opendir("/sys/class/tty/");
    int sysfs = (dir != NULL);
    if (!sysfs)
        dir = opendir("/dev/");

    if (dir != NULL) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != NULL) {
            int match = 0;
            size_t entry_len = strlen(entry->d_name);

            if (sysfs) {
                // test if /sys/class/tty/d_name/device exists
                char device[sizeof("/sys/class/tty/") - 1 + sizeof(entry->d_name)
                    + sizeof("/device") - 1];
                strcpy(device, "/sys/class/tty/");
                strcat(device, entry->d_name);
                strcat(device, "/device");
                match = (access(device, F_OK) == 0);
            } else if (entry_len >= sizeof("ttyS0") - 1
                /*&& entry->d_type == DT_CHR*/) {
                // /dev/tty[A-Z][0-9A-Z]+
                if (strncmp(entry->d_name, "tty", 3) == 0)
                    match = (strntest(&entry->d_name[3], "AZ", 1) == 0
                        && strntest(&entry->d_name[4], "09AZ", 0) == 0);
                // /dev/cua[Udu][0-9]+ (FreeBSD)
                else if (strncmp(entry->d_name, "cua", 3) == 0)
                    match = (strchr("Udu", entry->d_name[3]) != NULL
                        && strntest(&entry->d_name[4], "09", 0) == 0);
            }

            if (match) {
                char* value = (char*)result;
                // grow buffer if needed
                if (offset + sizeof("/dev/") + entry_len > sz) {
                    value = realloc(value, sz + 1024);
                    if (value == NULL)
                        break;
                    result = (char**)value;
                    sz += 1024;
                }
                // append port
                strcpy(value + offset, "/dev/");
                strcat(value + offset, entry->d_name);
                offset += sizeof("/dev/") + entry_len;
                ++n;
            }
        }
        closedir(dir);
    }
#endif

    // no ports found
    if (n == 0) {
        free(result);
        return NULL;
    }

#if defined(__unix__)
    // prepend array of (n + 1) pointers
    // (already done for _WIN32)
    char* value = (char*)result;
    size_t extra = (n + 1) * sizeof(char*);
    // grow buffer if needed
    if (extra + offset > sz) {
        value = realloc(value, extra + offset);
        if (value == NULL) {
            free(result);
            return NULL;
        }
        result = (char**)value;
        sz = extra + offset;
    }
    // make room for (n + 1) pointers
    memmove(value + extra, value, offset);
    offset = extra;
    // store n pointers
    for (size_t i = 0; i < n; ++i) {
        result[i] = value + offset;
        offset += strlen(value + offset) + 1;
    }
#endif

    // append NULL pointer
    result[n] = NULL;
    // shrink memory block
    return (offset < sz) ? realloc(result, offset) : result;
}

#if defined(__unix__)
// strntest("Hello!", "AZaz", 5) => 0
// strntest("Hello!", "AZaz", 6) => -1
static int strntest(const char* str, const char* test, size_t n)
{
    if (n == 0)
        n = strlen(str);
    for (size_t i = 0; i < n; ++i) {
        for (const char* p = test; ; p += 2) {
            if (p[0] == '\0' || str[i] < p[0])
                return -1;
            if (p[1] == '\0' || str[i] <= p[1])
                break;
        }
    }
    return 0;
}
#endif
