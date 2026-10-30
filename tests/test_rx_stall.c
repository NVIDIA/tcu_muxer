/* SPDX-License-Identifier: Apache-2.0 */
#define _GNU_SOURCE 1
#include <assert.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

static int test_ioctl(int fd, unsigned long request, ...);
static int test_clock_gettime(clockid_t id, struct timespec *ts);
static int test_open(const char *path, int flags, ...);
static int test_fstat(int fd, struct stat *st);
static char *test_realpath(const char *path, char *resolved);
static FILE *test_fopen(const char *path, const char *mode);
#define ioctl test_ioctl
#define clock_gettime test_clock_gettime
#define open test_open
#define fstat test_fstat
#define realpath test_realpath
#define fopen test_fopen
#include "../rx_stall.c"
#undef ioctl
#undef clock_gettime
#undef open
#undef fstat
#undef realpath
#undef fopen

static uint64_t test_now;
static unsigned int hardware_bytes;
static unsigned int hold_bits;
static int tty_queue;
static int query_length = 19;
static int query_calls;
static bool discovery_fixture;
static unsigned int open_failures;
static unsigned int open_calls;
static unsigned int part_calls;
static unsigned char chip_part = 0x08;

static int test_open(const char *path, int flags, ...)
{
    assert(discovery_fixture);
    assert(!strcmp(path, "/dev/bus/usb/001/002"));
    assert(flags == (O_RDWR | O_CLOEXEC));
    open_calls++;
    if (open_failures) {
        open_failures--;
        errno = ENOENT;
        return -1;
    }
    return open("/dev/null", flags);
}

static int test_fstat(int fd, struct stat *st)
{
    if (!discovery_fixture)
        return fstat(fd, st);
    memset(st, 0, sizeof(*st));
    st->st_mode = S_IFCHR;
    st->st_rdev = makedev(188, 2);
    return 0;
}

static char *test_realpath(const char *path, char *resolved)
{
    if (!discovery_fixture)
        return realpath(path, resolved);
    assert(!strcmp(path, "/sys/dev/char/188:2/device"));
    strcpy(resolved, "/fixture/cp2108");
    return resolved;
}

static FILE *test_fopen(const char *path, const char *mode)
{
    const char *attribute, *value;
    if (!discovery_fixture)
        return fopen(path, mode);
    attribute = strrchr(path, '/') + 1;
    if (!strcmp(attribute, "bInterfaceNumber"))
        value = "02\n";
    else if (!strcmp(attribute, "idVendor"))
        value = "10c4\n";
    else if (!strcmp(attribute, "idProduct"))
        value = "ea71\n";
    else if (!strcmp(attribute, "busnum"))
        value = "1\n";
    else {
        assert(!strcmp(attribute, "devnum"));
        value = "2\n";
    }
    return fmemopen((void *)value, strlen(value), mode);
}

static int test_clock_gettime(clockid_t id, struct timespec *ts)
{
    (void)id;
    ts->tv_sec = (time_t)(test_now / 1000);
    ts->tv_nsec = (long)(test_now % 1000) * 1000000L;
    return 0;
}

static int test_ioctl(int fd, unsigned long request, ...)
{
    va_list args;
    (void)fd;
    va_start(args, request);
    if (request == USBDEVFS_CONTROL) {
        struct usbdevfs_ctrltransfer *c = va_arg(args, void *);
        unsigned char *data = c->data;
        if (c->bRequest == 0xff) {
            assert(discovery_fixture && c->bRequestType == 0xc0);
            assert(c->wValue == 0x370b && c->wLength == 1);
            *data = chip_part;
            part_calls++;
            va_end(args);
            return 1;
        }
        assert(c->bRequestType == 0xc1 && c->bRequest == 0x10);
        assert(c->wIndex == 2 && c->wLength == 19 && c->timeout == 100);
        memset(data, 0, 19);
        for (unsigned int i = 0; i < 4; i++) {
            data[8 + i] = (unsigned char)(hardware_bytes >> (i * 8));
            data[4 + i] = (unsigned char)(hold_bits >> (i * 8));
        }
        query_calls++;
        va_end(args);
        return query_length;
    }
    assert(request == TIOCINQ);
    *va_arg(args, int *) = tty_queue;
    va_end(args);
    return 0;
}

static struct rx_stall fresh(void)
{
    struct rx_stall w = { .usb_fd = 100, .interface = 2, .last_rx_ms = 1000 };
    return w;
}

int main(void)
{
    struct rx_stall w = fresh();
    /* Long idle, normal tty backlog, RX hold, missing status, and a brief
     * burst of hardware backlog must never be treated as silent RX depletion.
     */
    for (unsigned int t = 3000; t < 100000; t += 1000)
        assert(!rx_stall_observe(&w, t, 0, 0, 0));
    assert(!rx_stall_observe(&w, 100000, 3071, 0, 0));
    assert(!rx_stall_observe(&w, 101000, 3071, 1, 0));
    assert(!rx_stall_observe(&w, 102000, 3071, 0, 0));
    assert(!rx_stall_observe(&w, 104000, 3071, 0, 1U << 6));
    assert(!rx_stall_observe(&w, 105000, 3071, 0, 0));
    assert(!rx_stall_observe(&w, 107000, -1, 0, 0));
    assert(!rx_stall_observe(&w, 108000, 3071, 0, 0));
    assert(!rx_stall_observe(&w, 109000, 0, 0, 0));
    assert(w.attempts == 0);
    puts("PASS idle, tty backlog, receive hold, missing telemetry and transient backlog");

    w = fresh();
    assert(!rx_stall_observe(&w, 2000, 3071, 0, 0));
    assert(!rx_stall_observe(&w, 3000, 3071, 0, 0));
    assert(!rx_stall_observe(&w, 4000, 3071, 0, 0));
    assert(rx_stall_observe(&w, 5000, 3071, 0, 0));
    assert(w.attempts == 1 && w.awaiting_rx && w.retry_after_ms == 10000);
    assert(!rx_stall_observe(&w, 6000, 3071, 0, 0));
    assert(!rx_stall_observe(&w, 9000, 3071, 0, 0));
    assert(rx_stall_observe(&w, 10000, 3071, 0, 0));
    assert(w.attempts == 2 && w.retry_after_ms == 20000);
    for (unsigned int attempt = 3; attempt <= 8; attempt++) {
        uint64_t due = w.retry_after_ms;
        assert(!rx_stall_observe(&w, due - 2000, 3071, 0, 0));
        assert(!rx_stall_observe(&w, due - 1, 3071, 0, 0));
        assert(rx_stall_observe(&w, due, 3071, 0, 0));
        assert(w.retry_after_ms - due <= 60000);
        if (attempt >= 5)
            assert(w.retry_after_ms - due == 60000);
    }
    test_now = w.retry_after_ms;
    rx_stall_progress(&w);
    assert(w.attempts == 0 && !w.awaiting_rx && w.retry_after_ms == 0);
    assert(!rx_stall_observe(&w, test_now + 1000, 3071, 0, 0));
    puts("PASS sustained silent RX stalls, bounded retries and reset only on received bytes");

    /* Exercise real USB decoding and the rate limiter with injected ioctl
     * replies, including a truncated successful response and all-ones input.
     */
    w = fresh();
    hardware_bytes = 3071;
    test_now = 3000;
    assert(!rx_stall_check(&w, 200));
    int before = query_calls;
    test_now = 3100;
    assert(!rx_stall_check(&w, 200) && query_calls == before);
    test_now = 4000;
    query_length = 18;
    assert(!rx_stall_check(&w, 200) && w.suspect_since_ms == 0);
    query_length = 19;
    test_now = 5000;
    assert(!rx_stall_check(&w, 200));
    test_now = 7000;
    assert(rx_stall_check(&w, 200));
    assert(w.attempts == 1 && w.awaiting_rx);
    hardware_bytes = UINT_MAX;
    test_now = 9000;
    assert(!rx_stall_check(&w, 200) && w.suspect_since_ms == 0);
    hold_bits = 1U << 6;
    hardware_bytes = 3071;
    test_now = 10000;
    assert(!rx_stall_check(&w, 200));
    puts("PASS USB status decoding, probe rate, truncated status and receive hold");

    /* A disappearing usbfs node must not disable monitoring permanently.
     * Exercise repeated failures and retry deadlines during active RX.
     */
    discovery_fixture = true;
    open_failures = 2;
    test_now = 20000;
    rx_stall_init(&w, 200);
    assert(w.usb_fd == -1 && w.discovery_retry && open_calls == 1);
    assert(w.next_probe_ms == 25000);
    for (test_now = 20001; test_now < 25000; test_now++) {
        rx_stall_progress(&w);
        assert(!rx_stall_check(&w, 200) && open_calls == 1);
    }
    assert(!rx_stall_check(&w, 200) && open_calls == 2);
    assert(w.usb_fd == -1 && w.discovery_retry && w.next_probe_ms == 30000);
    for (test_now = 25001; test_now < 30000; test_now++) {
        rx_stall_progress(&w);
        assert(!rx_stall_check(&w, 200) && open_calls == 2);
    }
    assert(!rx_stall_check(&w, 200));
    assert(w.usb_fd >= 0 && !w.discovery_retry && open_calls == 3);
    assert(part_calls == 1 && w.interface == 2 && w.attempts == 0);
    test_now = 32000;
    hold_bits = 0;
    before = query_calls;
    assert(!rx_stall_check(&w, 200) && query_calls == before + 1);
    test_now = 34000;
    assert(rx_stall_check(&w, 200));
    rx_stall_close(&w);
    puts("PASS transient USB open failures retry at bounded intervals during active RX, then restore stall detection");

    chip_part = 0x04;
    rx_stall_init(&w, 200);
    before = open_calls;
    test_now += 60000;
    assert(w.usb_fd == -1 && !w.discovery_retry);
    assert(!rx_stall_check(&w, 200) && open_calls == (unsigned int)before);
    discovery_fixture = false;
    puts("PASS unsupported chip is not repeatedly rediscovered");

    int null_fd = open("/dev/null", O_RDWR);
    assert(null_fd >= 0);
    rx_stall_init(&w, null_fd);
    assert(w.usb_fd == -1 && !rx_stall_check(&w, null_fd));
    rx_stall_close(&w);
    close(null_fd);
    puts("PASS unsupported tty/device leaves watchdog disabled");
    return 0;
}
