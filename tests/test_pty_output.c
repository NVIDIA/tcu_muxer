/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * Exercise the production workers with real PTYs and injected poll failures.
 */
#define _GNU_SOURCE 1
#include <poll.h>
#include <sys/socket.h>
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

static void read_pty_exact(int fd, unsigned char *buffer, size_t length)
{
    size_t offset = 0;
    double deadline = seconds(CLOCK_MONOTONIC) + 5;

    while (offset < length && seconds(CLOCK_MONOTONIC) < deadline) {
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        if (poll(&pfd, 1, 100) > 0) {
            ssize_t n = read(fd, buffer + offset, length - offset);
            if (n > 0)
                offset += (size_t)n;
            else
                assert(n < 0 && (errno == EAGAIN || errno == EINTR));
        }
    }
    assert(offset == length);
}

static void wait_output_idle(struct thread_data *sink)
{
    double deadline = seconds(CLOCK_MONOTONIC) + 3;

    while (seconds(CLOCK_MONOTONIC) < deadline) {
        bool idle;
        pthread_mutex_lock(&sink->output_lock);
        idle = sink->output_queue_length == 0 && !sink->output_overflow_paused;
        pthread_mutex_unlock(&sink->output_lock);
        if (idle)
            return;
        usleep(1000);
    }
    assert(!"output failed to drain and resume");
}

static void test_pty_reconnect(bool overflow)
{
    struct thread_data sink = { .log.fd = -1 };
    struct termios options;
    pthread_t worker;
    unsigned char payload[128 * 1024];
    unsigned char received[sizeof(payload)];
    const unsigned char sentinel[] = "AFTER_GAP";
    unsigned char *rejected = malloc(PTY_OUTPUT_QUEUE_SIZE);
    int slave;
    size_t index;
    unsigned int cycle;
    unsigned int cycles = overflow ? 2 : 1;

    assert(rejected != NULL);
    memset(rejected, 'X', PTY_OUTPUT_QUEUE_SIZE);
    for (index = 0; index < sizeof(payload); index++)
        payload[index] = (unsigned char)index;
    sink.fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(sink.fd >= 0);
    assert(grantpt(sink.fd) == 0 && unlockpt(sink.fd) == 0);
    slave = open(ptsname(sink.fd), O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(slave >= 0 && tcgetattr(slave, &options) == 0);
    cfmakeraw(&options);
    assert(tcsetattr(slave, TCSANOW, &options) == 0);
    start_muxer();
    assert(init_pty_output_queue(&sink, PTY_OUTPUT_QUEUE_SIZE) == 0);
    assert(pthread_create(&worker, NULL, pty_output_handler, &sink) == 0);

    for (cycle = 0; cycle < cycles; cycle++) {
        double cpu_start;
        double cpu_used;
        unsigned long long retries;
        size_t queued;

        close(slave);
        pthread_mutex_lock(&sink.output_lock);
        retries = sink.output_eagain;
        pthread_mutex_unlock(&sink.output_lock);
        assert(enqueue_pty_output(&sink, payload, sizeof(payload)) == 0);
        cpu_start = seconds(CLOCK_PROCESS_CPUTIME_ID);
        usleep(350000);
        cpu_used = seconds(CLOCK_PROCESS_CPUTIME_ID) - cpu_start;
        assert(cpu_used < 0.1);
        pthread_mutex_lock(&sink.output_lock);
        queued = sink.output_queue_length;
        assert(queued > 0 && !sink.output_unhealthy);
        assert(sink.output_eagain > retries && sink.output_eagain - retries <= 6);
        pthread_mutex_unlock(&sink.output_lock);

        if (overflow) {
            assert(enqueue_pty_output(&sink, rejected, PTY_OUTPUT_QUEUE_SIZE) == -ENOSPC);
            assert(enqueue_pty_output(&sink, sentinel, sizeof(sentinel)) == -ENOSPC);
            pthread_mutex_lock(&sink.output_lock);
            assert(sink.output_overflow_paused && !sink.output_unhealthy);
            assert(sink.output_queue_length == queued);
            pthread_mutex_unlock(&sink.output_lock);
        }

        slave = open(ptsname(sink.fd), O_RDWR | O_NOCTTY | O_NONBLOCK);
        assert(slave >= 0);
        read_pty_exact(slave, received, sizeof(payload));
        assert(memcmp(payload, received, sizeof(payload)) == 0);
        wait_output_idle(&sink);
        assert(is_tcu_muxer_started());
        if (overflow) {
            assert(enqueue_pty_output(&sink, sentinel, sizeof(sentinel)) == 0);
            read_pty_exact(slave, received, sizeof(sentinel));
            assert(memcmp(sentinel, received, sizeof(sentinel)) == 0);
            wait_output_idle(&sink);
        }
        printf("PASS real disconnected PTY: overflow=%u cycle=%u %.6fs CPU / 0.35s, "
               "%zu ordered bytes after reconnect\n",
               overflow, cycle + 1, cpu_used, sizeof(payload));
    }
    stop_worker(worker);
    assert(sink.output_bytes == cycles * (sizeof(payload) +
                                         (overflow ? sizeof(sentinel) : 0)));
    assert(sink.output_shutdown_dropped == 0 && !sink.output_unhealthy);
    assert(sink.output_overflows == (overflow ? cycles : 0));
    assert(sink.output_overflow_recoveries == (overflow ? cycles : 0));
    assert(sink.output_gap_rejected_bytes == 0);
    assert(sink.output_rejected_bytes == (overflow ?
           cycles * (PTY_OUTPUT_QUEUE_SIZE + sizeof(sentinel)) : 0));
    if (overflow)
        puts("PASS overflow recovery twice on the same PTY/worker without restart; gap accounting exact");
    destroy_pty_output_queue(&sink);
    free(rejected);
    close(slave);
    close(sink.fd);
}

static void test_oversized_batch(void)
{
    struct thread_data sink = { .log.fd = -1 };
    unsigned char data[128] = { 0 };

    assert(init_pty_output_queue(&sink, 64) == 0);
    assert(enqueue_pty_output(&sink, data, sizeof(data)) == -ENOSPC);
    assert(!sink.output_unhealthy && !sink.output_overflow_paused);
    assert(sink.output_rejected_bytes == sizeof(data));
    assert(sink.output_overflow_recoveries == 1);
    assert(enqueue_pty_output(&sink, data, 1) == 0);
    destroy_pty_output_queue(&sink);
    puts("PASS oversized admission does not permanently pause an empty queue");
}

static void wait_for_queue(struct thread_data *sink, size_t length)
{
    double deadline = seconds(CLOCK_MONOTONIC) + 3;
    while (seconds(CLOCK_MONOTONIC) < deadline) {
        bool ready;
        pthread_mutex_lock(&sink->output_lock);
        ready = sink->output_queue_length == length;
        pthread_mutex_unlock(&sink->output_lock);
        if (ready)
            return;
        usleep(1000);
    }
    assert(!"RX routing failed to deliver healthy channel");
}

static void test_overflow_isolation(void)
{
    int pair[2];
    pthread_t reader;
    unsigned char seed[63];
    const unsigned char first[] = "\xff\xe1" "PSC-full" "\xff\xe9" "healthy";
    const unsigned char second[] = "\xff\xe1" "still-full" "\xff\xe9" "-again";
    int index;

    memset(seed, 'Q', sizeof(seed));
    start_muxer();
    tags = chip_tags;
    num_proc = sizeof(chip_tags) / sizeof(chip_tags[0]);
    pty_max_count = num_proc + 1;
    raw_pty_idx = num_proc;
    pty_data = calloc((size_t)pty_max_count, sizeof(*pty_data));
    assert(pty_data != NULL);
    for (index = 0; index < pty_max_count; index++) {
        pty_data[index].id = (unsigned int)index;
        assert(init_pty_output_queue(&pty_data[index], 64) == 0);
    }
    assert(enqueue_pty_output(&pty_data[0], seed, sizeof(seed)) == 0);
    assert(enqueue_pty_output(&pty_data[raw_pty_idx], seed, sizeof(seed)) == 0);
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, pair) == 0);
    tty_data.fd = pair[1];
    tty_data.log.fd = -1;
    assert(init_physical_tty_lifecycle() == 0);
    assert(pthread_create(&reader, NULL, tty_input_handler, NULL) == 0);
    assert(write(pair[0], first, sizeof(first) - 1) == sizeof(first) - 1);
    wait_for_queue(&pty_data[14], 7);
    assert(is_tcu_muxer_started());
    assert(write(pair[0], second, sizeof(second) - 1) == sizeof(second) - 1);
    wait_for_queue(&pty_data[14], 13);
    assert(is_tcu_muxer_started());
    stop_worker(reader);
    assert(memcmp(pty_data[14].output_queue, "healthy-again", 13) == 0);
    assert(!pty_data[14].output_unhealthy);
    assert(pty_data[0].output_overflows == 1 && !pty_data[0].output_unhealthy);
    assert(pty_data[0].output_overflow_paused);
    assert(pty_data[0].output_rejected_bytes == 18);
    assert(pty_data[raw_pty_idx].output_overflows == 1);
    assert(pty_data[raw_pty_idx].output_overflow_paused);
    assert(!pty_data[raw_pty_idx].output_unhealthy);
    assert(pty_data[raw_pty_idx].output_rejected_bytes ==
           sizeof(first) + sizeof(second) - 2);
    assert(pty_data[0].output_queue_length == sizeof(seed));
    assert(pty_data[raw_pty_idx].output_queue_length == sizeof(seed));
    for (index = 0; index < pty_max_count; index++)
        destroy_pty_output_queue(&pty_data[index]);
    free(pty_data);
    pty_data = NULL;
    close(pair[0]);
    close(pair[1]);
    destroy_physical_tty_lifecycle();
    puts("PASS PSC + RAW overflow and subsequent rejection preserve healthy CCPLEX routing");
}

int main(void)
{
    test_poll_events(POLLHUP, 0, false);
    test_poll_events(POLLHUP | POLLOUT, 0, false);
    test_poll_events(POLLERR | POLLOUT, 0, false);
    test_poll_events(0, 0, false);
    test_poll_events(POLLNVAL | POLLOUT, 0, true);
    test_poll_events(0, EBADF, true);
    test_pty_reconnect(false);
    test_pty_reconnect(true);
    test_oversized_batch();
    test_overflow_isolation();
    return 0;
}
