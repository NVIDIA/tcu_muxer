/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * Exercise the production workers with real PTYs and injected poll failures.
 */
#define _GNU_SOURCE 1
#include <poll.h>
#include <unistd.h>
#include <errno.h>

static int injected_events;
static int injected_error;
static int interrupted_once;
static int test_poll(struct pollfd *fds, nfds_t count, int timeout);
static ssize_t test_write(int fd, const void *buffer, size_t length);
int muxer_main(int argc, char *argv[]);
#undef _XOPEN_SOURCE
#define main muxer_main
#define poll test_poll
#define write test_write
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "../tcu_com.c"
#undef main
#undef poll
#undef write

#define INJECTED_FD (-123)

static int test_poll(struct pollfd *fds, nfds_t count, int timeout)
{
    if (count == 1 && fds[0].fd == INJECTED_FD) {
        if (interrupted_once) {
            interrupted_once = 0;
            errno = EINTR;
            return -1;
        }
        if (injected_error) {
            errno = injected_error;
            return -1;
        }
        fds[0].revents = injected_events;
        if (!injected_events)
            return poll(NULL, 0, timeout);
        return 1;
    }
    return poll(fds, count, timeout);
}

static ssize_t test_write(int fd, const void *buffer, size_t length)
{
    if (fd == INJECTED_FD) {
        errno = EAGAIN;
        return -1;
    }
    return write(fd, buffer, length);
}

static double seconds(clockid_t clock_id)
{
    struct timespec now;
    assert(clock_gettime(clock_id, &now) == 0);
    return now.tv_sec + now.tv_nsec / 1e9;
}

static void start_muxer(void)
{
    __atomic_store_n(&tcu_muxer_started, true, __ATOMIC_RELEASE);
}

static void stop_worker(pthread_t worker)
{
    double start = seconds(CLOCK_MONOTONIC);
    request_muxer_stop();
    assert(pthread_join(worker, NULL) == 0);
    assert(seconds(CLOCK_MONOTONIC) - start < 1.0);
}

static void test_poll_events(int events, int error, bool fatal)
{
    struct thread_data sink = { .fd = INJECTED_FD, .log.fd = -1 };
    pthread_t worker;

    injected_events = events;
    injected_error = error;
    interrupted_once = 1;
    start_muxer();
    assert(init_pty_output_queue(&sink, 64) == 0);
    assert(enqueue_pty_output(&sink, (const unsigned char *)"ordered", 7) == 0);
    assert(pthread_create(&worker, NULL, pty_output_handler, &sink) == 0);
    usleep(350000);
    stop_worker(worker);
    assert(sink.output_eagain >= 2 && sink.output_eagain <= 6);
    assert(sink.output_unhealthy);
    if (fatal) {
        assert(sink.output_queue_length == 7);
        assert(sink.output_shutdown_dropped == 0);
    } else {
        assert(sink.output_queue_length == 0);
        assert(sink.output_shutdown_dropped == 7);
    }
    destroy_pty_output_queue(&sink);
    printf("PASS poll events=%x error=%d: retries=%llu, bounded stop\n",
           events, error, sink.output_eagain);
}

static void test_pty_reconnect(void)
{
    struct thread_data sink = { .log.fd = -1 };
    struct termios options;
    pthread_t worker;
    unsigned char payload[128 * 1024];
    unsigned char received[sizeof(payload)];
    int slave;
    size_t offset = 0;
    double deadline;
    double cpu_start;
    double cpu_used;
    size_t index;

    for (index = 0; index < sizeof(payload); index++)
        payload[index] = (unsigned char)index;
    sink.fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(sink.fd >= 0);
    assert(grantpt(sink.fd) == 0 && unlockpt(sink.fd) == 0);
    slave = open(ptsname(sink.fd), O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(slave >= 0 && tcgetattr(slave, &options) == 0);
    cfmakeraw(&options);
    assert(tcsetattr(slave, TCSANOW, &options) == 0);
    close(slave);
    start_muxer();
    assert(init_pty_output_queue(&sink, PTY_OUTPUT_QUEUE_SIZE) == 0);
    assert(enqueue_pty_output(&sink, payload, sizeof(payload)) == 0);
    cpu_start = seconds(CLOCK_PROCESS_CPUTIME_ID);
    assert(pthread_create(&worker, NULL, pty_output_handler, &sink) == 0);
    usleep(350000);
    cpu_used = seconds(CLOCK_PROCESS_CPUTIME_ID) - cpu_start;
    assert(cpu_used < 0.1);
    pthread_mutex_lock(&sink.output_lock);
    assert(sink.output_queue_length > 0 && !sink.output_unhealthy);
    assert(sink.output_eagain > 0 && sink.output_eagain <= 6);
    pthread_mutex_unlock(&sink.output_lock);

    slave = open(ptsname(sink.fd), O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(slave >= 0);
    deadline = seconds(CLOCK_MONOTONIC) + 5;
    while (offset < sizeof(received) && seconds(CLOCK_MONOTONIC) < deadline) {
        struct pollfd pfd = { .fd = slave, .events = POLLIN };
        if (poll(&pfd, 1, 100) > 0) {
            ssize_t n = read(slave, received + offset, sizeof(received) - offset);
            if (n > 0)
                offset += (size_t)n;
            else
                assert(n < 0 && (errno == EAGAIN || errno == EINTR));
        }
    }
    assert(offset == sizeof(payload));
    assert(memcmp(payload, received, sizeof(payload)) == 0);
    stop_worker(worker);
    assert(sink.output_bytes == sizeof(payload));
    assert(sink.output_shutdown_dropped == 0 && !sink.output_unhealthy);
    printf("PASS real disconnected PTY: %.6fs CPU / 0.35s, %zu ordered bytes after reconnect\n",
           cpu_used, offset);
    destroy_pty_output_queue(&sink);
    close(slave);
    close(sink.fd);
}

int main(void)
{
    test_poll_events(POLLHUP, 0, false);
    test_poll_events(POLLHUP | POLLOUT, 0, false);
    test_poll_events(POLLERR | POLLOUT, 0, false);
    test_poll_events(0, 0, false);
    test_poll_events(POLLNVAL | POLLOUT, 0, true);
    test_poll_events(0, EBADF, true);
    test_pty_reconnect();
    return 0;
}
