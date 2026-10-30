/* SPDX-License-Identifier: Apache-2.0
 * Recover silent CP2108 RX depletion without treating console idle as failure.
 * Only vendor status reads are sent through usbfs. No interface is detached,
 * claimed, reset, or written by this monitor.
 */
#define _GNU_SOURCE 1
#include "rx_stall.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/usbdevice_fs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <time.h>
#include <unistd.h>

#define PROBE_INTERVAL_MS 1000U
#define DISCOVERY_RETRY_MS 5000U
#define RX_IDLE_MS 2000U
#define STALL_CONFIRM_MS 2000U
#define RX_HOLD_DSR (1U << 6)

static uint64_t monotonic_ms(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) < 0)
        return 0;
    return (uint64_t)ts.tv_sec * 1000U + (uint64_t)ts.tv_nsec / 1000000U;
}

static int read_number(const char *directory, const char *attribute,
                        unsigned int base, unsigned int *value)
{
    char path[PATH_MAX], text[32], *end;
    unsigned long number;
    FILE *file;
    int length = snprintf(path, sizeof(path), "%s/%s", directory, attribute);
    if (length < 0 || (size_t)length >= sizeof(path))
        return -1;
    file = fopen(path, "r");
    if (!file)
        return -1;
    if (!fgets(text, sizeof(text), file)) {
        fclose(file);
        return -1;
    }
    fclose(file);
    errno = 0;
    number = strtoul(text, &end, (int)base);
    if (errno || end == text || (*end && *end != '\n') || number > UINT_MAX)
        return -1;
    *value = (unsigned int)number;
    return 0;
}

static int vendor_read(int fd, unsigned int interface, unsigned int request,
                       unsigned int value, void *data, unsigned int length)
{
    struct usbdevfs_ctrltransfer control = {
        .bRequestType = request == 0xff ? 0xc0 : 0xc1,
        .bRequest = request, .wValue = value, .wIndex = interface,
        .wLength = length, .timeout = 100, .data = data,
    };
    return ioctl(fd, USBDEVFS_CONTROL, &control);
}

void rx_stall_close(struct rx_stall *watch)
{
    if (watch->usb_fd >= 0)
        close(watch->usb_fd);
    watch->usb_fd = -1;
}

void rx_stall_refresh(struct rx_stall *watch, int tty_fd)
{
    char link[128], resolved[PATH_MAX], usb_path[64], *slash;
    unsigned int interface = UINT_MAX, vendor, product, bus, device;
    unsigned char part = 0;
    struct stat st;
    int fd;

    rx_stall_close(watch);
    watch->discovery_retry = false;
    watch->suspect_since_ms = 0;
    watch->next_probe_ms = monotonic_ms() + PROBE_INTERVAL_MS;
    if (fstat(tty_fd, &st) < 0 || !S_ISCHR(st.st_mode))
        return;
    snprintf(link, sizeof(link), "/sys/dev/char/%u:%u/device",
             major(st.st_rdev), minor(st.st_rdev));
    if (!realpath(link, resolved))
        return;
    /* Derive the interface and USB address from sysfs, never ttyUSB numbering
     * or a cached bus/device address. Re-discover after every tty replacement.
     */
    for (;;) {
        if (interface == UINT_MAX)
            (void)read_number(resolved, "bInterfaceNumber", 16, &interface);
        if (!read_number(resolved, "idVendor", 16, &vendor) &&
            !read_number(resolved, "idProduct", 16, &product))
            break;
        slash = strrchr(resolved, '/');
        if (!slash || slash == resolved)
            return;
        *slash = '\0';
    }
    if (vendor != 0x10c4 || product != 0xea71 || interface > 3 ||
        read_number(resolved, "busnum", 10, &bus) ||
        read_number(resolved, "devnum", 10, &device) || bus > 999 || device > 999)
        return;
    snprintf(usb_path, sizeof(usb_path), "/dev/bus/usb/%03u/%03u", bus, device);
    fd = open(usb_path, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        int error = errno;
        watch->discovery_retry = true;
        watch->next_probe_ms = monotonic_ms() + DISCOVERY_RETRY_MS;
        fprintf(stderr, "WARNING: CP2108 RX monitor unavailable: %s: %s\n",
                usb_path, strerror(error));
        return;
    }
    /* ID alone is not proof of the chip model. CP2108 part number is 0x08. */
    if (vendor_read(fd, 0, 0xff, 0x370b, &part, 1) != 1 || part != 0x08) {
        close(fd);
        return;
    }
    watch->usb_fd = fd;
    watch->interface = interface;
    watch->probe_failed = false;
    fprintf(stderr, "INFO: CP2108 RX monitor enabled: %s interface=%u\n",
            usb_path, interface);
}

void rx_stall_init(struct rx_stall *watch, int tty_fd)
{
    memset(watch, 0, sizeof(*watch));
    watch->usb_fd = -1;
    watch->last_rx_ms = monotonic_ms();
    rx_stall_refresh(watch, tty_fd);
}

void rx_stall_progress(struct rx_stall *watch)
{
    if (watch->awaiting_rx)
        fprintf(stderr, "INFO: physical RX resumed after %u stall recovery attempt(s)\n",
                watch->attempts);
    watch->last_rx_ms = monotonic_ms();
    watch->suspect_since_ms = 0;
    watch->retry_after_ms = 0;
    watch->attempts = 0;
    watch->awaiting_rx = false;
}

static bool rx_stall_observe(struct rx_stall *watch, uint64_t now_ms,
                      int input_bytes, int tty_bytes, unsigned int hold)
{
    unsigned int delay;
    if (now_ms < watch->last_rx_ms || now_ms - watch->last_rx_ms < RX_IDLE_MS ||
        input_bytes <= 0 || tty_bytes != 0 || (hold & RX_HOLD_DSR)) {
        watch->suspect_since_ms = 0;
        return false;
    }
    if (!watch->suspect_since_ms) {
        watch->suspect_since_ms = now_ms;
        return false;
    }
    if (now_ms < watch->suspect_since_ms ||
        now_ms - watch->suspect_since_ms < STALL_CONFIRM_MS ||
        now_ms < watch->retry_after_ms)
        return false;
    if (watch->attempts < UINT_MAX)
        watch->attempts++;
    delay = 5000U << (watch->attempts > 5 ? 4 : watch->attempts - 1);
    if (delay > 60000U)
        delay = 60000U;
    watch->retry_after_ms = now_ms + delay;
    watch->suspect_since_ms = 0;
    watch->awaiting_rx = true;
    return true;
}

static unsigned int little_u32(const unsigned char *data)
{
    return (unsigned int)data[0] | (unsigned int)data[1] << 8 |
           (unsigned int)data[2] << 16 | (unsigned int)data[3] << 24;
}

bool rx_stall_check(struct rx_stall *watch, int tty_fd)
{
    unsigned char status[19];
    unsigned int pending, hold;
    int tty_bytes, result;
    uint64_t now;
    if (watch->usb_fd < 0 && !watch->discovery_retry)
        return false;
    now = monotonic_ms();
    if (now < watch->next_probe_ms)
        return false;
    if (watch->usb_fd < 0) {
        /* Retry discovery even while RX is active; this is not a stall. */
        rx_stall_refresh(watch, tty_fd);
        return false;
    }
    if (now < watch->last_rx_ms || now - watch->last_rx_ms < RX_IDLE_MS)
        return false;
    watch->next_probe_ms = now + PROBE_INTERVAL_MS;
    result = vendor_read(watch->usb_fd, watch->interface, 0x10, 0,
                         status, sizeof(status));
    if (result != sizeof(status) || ioctl(tty_fd, TIOCINQ, &tty_bytes) < 0) {
        watch->suspect_since_ms = 0;
        if (!watch->probe_failed)
            fprintf(stderr, "WARNING: CP2108 RX status unavailable; no stall recovery inferred\n");
        watch->probe_failed = true;
        return false;
    }
    watch->probe_failed = false;
    pending = little_u32(status + 8);
    hold = little_u32(status + 4);
    if (pending > INT_MAX)
        return rx_stall_observe(watch, now, -1, tty_bytes, hold);
    if (!rx_stall_observe(watch, now, (int)pending, tty_bytes, hold))
        return false;
    fprintf(stderr, "WARNING: CP2108 RX stalled: interface=%u device_pending=%u "
            "tty_pending=%d idle_ms=%llu attempt=%u; reopening physical tty; "
            "buffered data may be lost\n", watch->interface, pending, tty_bytes,
            (unsigned long long)(now - watch->last_rx_ms), watch->attempts);
    return true;
}
