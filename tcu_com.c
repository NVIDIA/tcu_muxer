/*
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: Copyright (c) 2013-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define _XOPEN_SOURCE 600
#define _BSD_SOURCE 1
#define _DEFAULT_SOURCE 1
#define _GNU_SOURCE 1

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <pthread.h>
#include <termios.h>
#include <stdbool.h>
#include <ctype.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <dirent.h>
#include <signal.h>
#include <stdint.h>
#include <limits.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include "raw_capture.h"
#include "uart-proto.h"

struct reply_ctx {
    unsigned char data[2048];
    unsigned int length;
};

#ifdef UNIT_TEST
#include "test/test_tcu_com_events.h"

#define ut_static
#define exit(code)  ut_exit(code, __FILE__, __LINE__, __func__)
#define abort() ut_abort(__FILE__, __LINE__, __func__)
#undef assert
#define assert(cond) ut_assert(cond, #cond, __FILE__, __LINE__, __func__)

int main(int argc, char *argv[]);
int write_data_to_uart(unsigned char pty_idx, const unsigned char *data, size_t len);
void uucp_unlock_tty_device(void);
int uucp_set_filelock_name(void);
bool uucp_check_is_locked(void);
int uucp_lock_tty_device(void);

#else
#define ut_static static
#endif

#define BUF_SIZE            64
#define MAX_PATH            256
#define DEFAULT_BAUDRATE 115200
/* A physical UART may legitimately deassert flow control while its peer
 * drains a burst.  The former 300 ms deadline was suitable for small console
 * commands but produced a false fatal error during the 1.3 MiB qualification
 * transfer after 17,339 bytes had already made forward progress.  This is a
 * per-progress stall deadline, not a total-transfer deadline: every accepted
 * write starts a fresh wait.  A genuinely wedged link still fails closed
 * after five seconds and leaves explicit counters/evidence. */
#define DEFAULT_POLL_OUTPUT_TIMEOUT 5000
#define DEFAULT_TTY_DEVICE "/dev/ttyUSB3"
#ifndef UUCP_DIR
#define UUCP_DIR "/var/lock"
#endif
#define RAW_PTY "RAW"
#define MAX_WRITE_CHUNK_SIZE 4096  // Maximum bytes to process at once (16KB encoded max)
#define THREAD_STACK_SIZE (128 * 1024)  // 128KB stack per thread (sufficient for this application)
#ifndef PHYSICAL_TX_QUEUE_SIZE
#define PHYSICAL_TX_QUEUE_SIZE (4U * 1024U * 1024U)
#endif
/* Use the complete bounded allocation as the smoothing window. Normal
 * commands and the 1.3 MiB qualification batch are admitted without forcing
 * the upstream obmc-console relay into transient PTY backpressure. Once this
 * finite queue is full, admission waits and the corrected relay propagates
 * backpressure to SSH without discarding the unwritten suffix. */
#define PHYSICAL_TX_BACKPRESSURE_LIMIT PHYSICAL_TX_QUEUE_SIZE
/* Keep individual submissions to the USB-serial driver at the same scale as
 * the original PTY reads. Driver acceptance is not proof of electrical
 * delivery; small submissions avoid hiding a large suffix behind one
 * successful 16 KiB write return. */
#define PHYSICAL_TX_WRITE_SIZE 1024U
/* A stalled console is isolated behind its own bounded queue. The physical
 * UART reader must never block on a PTY consumer or silently discard EAGAIN. */
#define PTY_OUTPUT_QUEUE_SIZE (1024U * 1024U)

ut_static bool tcu_muxer_started = true; // Needed to exit endless while loops during testing
ut_static volatile sig_atomic_t output_stats_requested;
ut_static const char* tty_device = DEFAULT_TTY_DEVICE;
ut_static int int_baudrate = DEFAULT_BAUDRATE;
ut_static int pty_max_count;
ut_static int raw_pty_idx;
ut_static bool uucp_locked = false;
ut_static char filelock[MAX_PATH];
static int poll_output_timeout = DEFAULT_POLL_OUTPUT_TIMEOUT;
static char path[MAX_PATH];
static bool enable_write_raw_pty = false;
static char *native_raw_capture_path = NULL;
static bool native_raw_capture_enabled = false;
static unsigned long long parser_input_bytes;
/* Physical-UART transmit counters distinguish successfully handled
 * backpressure from data loss. They are atomic because any VSER input worker
 * may become the writer, even though tty_data.write_lock serializes complete
 * encoded transactions. */
static unsigned long long tx_requested_bytes;
static unsigned long long tx_accepted_bytes;
static unsigned long long tx_write_calls;
static unsigned long long tx_short_writes;
static unsigned long long tx_eintr_retries;
static unsigned long long tx_eagain_retries;
static unsigned long long tx_poll_timeouts;
static unsigned long long tx_zero_writes;
static unsigned long long tx_fatal_errors;
static unsigned long long parser_invalid_tags;
static unsigned long long parser_quarantined_bytes;
static unsigned long long parser_resynchronizations;
static unsigned long long parser_last_invalid_offset;
static unsigned long long parser_last_invalid_realtime_ns;
static unsigned long long parser_last_invalid_monotonic_ns;

struct tag {
    char *name;
    unsigned char value;
};

const struct tag chip_tags[] = {
    {
        .name = "PSC",
        .value = 0xe1
    },
    {
        .name = "BPMP",
        .value = 0xe2
    },
    {
        .name = "OOBHUB",
        .value = 0xe3
    },
    {
        .name = "SatMC",
        .value = 0xe4
    },
    {
        .name = "RAS",
        .value = 0xe5
    },
    {
        .name = "CCPLEX-ROOT",
        .value = 0xe6
    },
    {
        .name = "TZ",
        .value = 0xe7
    },
    {
        .name = "CCPLEX-REALM",
        .value = 0xe8
    },
    {
        .name = "MSEQ",
        .value = 0xea
    },
    {
        .name = "PCORE",
        .value = 0xeb
    },
    {
        .name = "C2C",
        .value = 0xec
    },
    {
        .name = "DBG2",
        .value = 0xed
    },
    {
        .name = "RSVD15",
        .value = 0xef
    },
    {
        .name = "RSVD16",
        .value = 0xf0
    },
    {
        .name = "CCPLEX",
        .value = 0xe9
    }
};

const struct tag *tags = NULL;
int num_proc = 0;
int default_tag_idx = 0;

int disable_patch = 1; // patch line ending
bool log_timestamp_enabled = false;
ut_static char* save_output_path = NULL;

ut_static const size_t temporary_buffer_size = 1024;
#define PHYSICAL_RX_BUFFER_SIZE (16ul * 1024ul)
#define PHYSICAL_RX_EXPANSION_SIZE (2ul * PHYSICAL_RX_BUFFER_SIZE)

/* One physical read can expand by at most one inserted CR/LF byte for each
 * input byte.  Accumulating that bounded expansion per VSER lets the physical
 * reader enqueue once per destination instead of taking a mutex and waking a
 * worker for every character. */
struct rx_route_batch {
    unsigned char data[PHYSICAL_RX_EXPANSION_SIZE];
    size_t length;
};

#define TIMESTAMP_ESC_START 1
#define TIMESTAMP_ESC_CSI 2
#define MUXER_LOG_BUFFER_SIZE (16U * 1024U)

struct muxer_log {
    int fd;
    size_t size;
    size_t maxsize;
    char *filename;
    char *rotate_filename;
    unsigned char last_ch;
    bool timestamp_pending;
    int timestamp_escape_state;
    /* Timestamp expansion is accumulated here by the per-VSER worker. This
     * turns a burst of individual console characters into a small number of
     * filesystem writes without moving disk latency back into the physical
     * UART reader. */
    unsigned char write_buffer[MUXER_LOG_BUFFER_SIZE];
    size_t write_buffer_length;
};

static int muxer_log_flush(struct muxer_log *log);

// prototype
bool should_retry_io(ssize_t ret);
int putch_or_exit(int fd, const unsigned char ch);
int putbuf_or_exit(int fd, const unsigned char *buf, size_t len);
size_t get_timestamp(char *buf, const size_t len);
int flush_stream(int fd, int tag, unsigned char ch);
int patch2flush_stream(int fd, int tag, unsigned char ch, int *p_seen_n, int *p_seen_r);
void* tty_input_handler(void *arg);
void* pty_input_handler(void* arg);
void print_usage(char *argv[]);
int invalid_baudrate(int baudrate);
speed_t get_baudrate(int baudrate);
int create_thread_with_stack(pthread_t *thread, void *(*start_routine)(void *), void *arg);
ut_static int open_tty_device(void);

struct thread_data
{
    int fd;
    struct muxer_log log;
    unsigned int id;
    pthread_mutex_t write_lock;
    bool write_lock_initialized;
    char device_name[128];
    /* The UART reader is the sole producer and this VSER's worker is the sole
     * consumer. All queue state is protected by output_lock. */
    unsigned char *output_queue;
    size_t output_queue_capacity;
    size_t output_queue_head;
    size_t output_queue_length;
    size_t output_queue_high_water;
    unsigned long long output_bytes;
    unsigned long long output_eagain;
    unsigned long long output_overflows;
    unsigned long long output_log_errors;
    unsigned long long output_shutdown_dropped;
    unsigned long long input_bytes;
    unsigned long long input_reads;
    bool output_unhealthy;
    pthread_mutex_t output_lock;
    pthread_cond_t output_ready;
};

struct thread_data tty_data;
struct thread_data *pty_data;

/* All VSER input threads produce encoded TCU bytes into one queue because the
 * platform has one physical UART. A single worker preserves queue order and
 * removes bytes only after the BMC TTY driver accepts them. */
struct physical_tx_queue {
    unsigned char *data;
    size_t capacity;
    size_t head;
    size_t length;
    size_t high_water;
    unsigned long long overflows;
    unsigned long long backpressure_waits;
    unsigned long long admission_timeouts;
    unsigned long long shutdown_dropped;
    bool unhealthy;
    pthread_mutex_t lock;
    pthread_cond_t ready;
    pthread_cond_t space_available;
};

static struct physical_tx_queue physical_tx;

/* The physical UART descriptor is shared by the RX reader and the dedicated
 * TX worker.  Only the RX thread replaces it, but replacement must not close
 * the descriptor while TX is inside poll(2) or write(2).  This lock is kept
 * separate from tty_data.write_lock: producers may wait for TX queue space
 * while holding write_lock, so using it for reopen would deadlock the worker
 * that must drain that queue. */
struct physical_tty_lifecycle {
    pthread_mutex_t lock;
    pthread_cond_t changed;
    unsigned long long generation;
    unsigned long long reopen_requests;
    unsigned long long reopen_attempts;
    unsigned long long reopen_successes;
    unsigned long long reopen_failures;
    unsigned long long tx_retries;
    unsigned int active_tx;
    bool reopen_requested;
    bool reopen_in_progress;
    bool failed;
    bool initialized;
};

static struct physical_tty_lifecycle physical_tty;

#define for_each_tags(tag_idx, tag) \
    for (tag_idx = 0, tag = &tags[tag_idx]; tag_idx < num_proc; tag_idx++, tag++)

#define for_each_pty(i, pty) \
    for (i = 0, pty = &pty_data[i]; i < pty_max_count; i++, pty++)

ut_static bool is_tcu_muxer_started(void)
{
    return __atomic_load_n(&tcu_muxer_started, __ATOMIC_ACQUIRE);
}

static void request_muxer_stop(void)
{
    __atomic_store_n(&tcu_muxer_started, false, __ATOMIC_RELEASE);
}

static bool output_stats_request_pending(void)
{
    return __atomic_load_n(&output_stats_requested, __ATOMIC_ACQUIRE) != 0;
}

static bool consume_output_stats_request(void)
{
    return __atomic_exchange_n(&output_stats_requested, 0, __ATOMIC_ACQ_REL) != 0;
}

ut_static void handle_output_stats_request(int sig)
{
    (void)sig;
    /* The signal handler only records intent. A normal worker takes locks and
     * prints the snapshot, keeping this handler async-signal-safe. */
    __atomic_store_n(&output_stats_requested, 1, __ATOMIC_RELEASE);
}

static void muxer_log_init(struct muxer_log *log)
{
    log->fd = -1;
    log->size = 0;
    log->maxsize = 0;
    log->filename = NULL;
    log->rotate_filename = NULL;
    log->last_ch = '\0';
    log->timestamp_pending = false;
    log->timestamp_escape_state = 0;
    log->write_buffer_length = 0;
}

static void muxer_log_close(struct muxer_log *log)
{
    if (muxer_log_flush(log) < 0)
        fprintf(stderr, "ERROR: failed to flush %s while closing log.\n",
                log->filename ? log->filename : "unnamed muxer log");
    if (log->fd >= 0)
        close(log->fd);
    log->fd = -1;
    free(log->filename);
    free(log->rotate_filename);
    log->filename = NULL;
    log->rotate_filename = NULL;
}

static int parse_log_size(const char *size_str, size_t *size)
{
    size_t logsize;
    char *suffix;
    size_t shift = 0;

    if (!size_str)
        return -1;

    errno = 0;
    logsize = strtoul(size_str, &suffix, 0);
    if (errno || logsize == 0 || logsize >= UINT32_MAX ||
            suffix == size_str) {
        return -1;
    }

    while (*suffix && isspace((unsigned char)*suffix))
        suffix++;

    if (*suffix == 'k')
        shift = 10;
    else if (*suffix == 'M')
        shift = 20;
    else if (*suffix == 'G')
        shift = 30;

    if (shift) {
        if (logsize > (UINT32_MAX >> shift))
            return -1;
        logsize <<= shift;
        suffix++;
    }

    while (*suffix && (tolower((unsigned char)*suffix) == 'b' ||
                isspace((unsigned char)*suffix))) {
        suffix++;
    }

    if (*suffix)
        return -1;

    *size = logsize;
    return 0;
}

static int muxer_log_rotate(struct muxer_log *log)
{
    int rc, old_fd = log->fd;

    rc = rename(log->filename, log->rotate_filename);
    if (rc) {
        fprintf(stderr, "WARNING: failed to rename %s to %s: %s\n",
                log->filename, log->rotate_filename, strerror(errno));
        /* Re-open in append mode to avoid data loss */
        log->fd = open(log->filename, O_WRONLY | O_CREAT | O_APPEND,
                       S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR);
        if (log->fd < 0) {
            fprintf(stderr, "ERROR: log file open failed: %s: %s\n",
                    log->filename, strerror(errno));
            log->fd = old_fd;  /* Restore old fd */
            return -1;
        }
        if (old_fd >= 0)
            close(old_fd);
        /* Keep existing size since we're appending */
        return 0;
    }

    log->fd = open(log->filename, O_WRONLY | O_CREAT | O_TRUNC,
                   S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR);
    if (log->fd < 0) {
        fprintf(stderr, "ERROR: log file open failed: %s: %s\n",
                log->filename, strerror(errno));
        log->fd = old_fd;  /* Restore old fd */
        return -1;
    }
    if (old_fd >= 0)
        close(old_fd);

    log->size = 0;
    log->last_ch = '\0';
    log->timestamp_pending = false;
    log->timestamp_escape_state = 0;
    return 0;
}

static int muxer_log_open(struct muxer_log *log, const char *filename,
                          size_t maxsize)
{
    off_t pos;

    muxer_log_init(log);
    log->maxsize = maxsize;
    log->filename = strdup(filename);
    if (!log->filename)
        goto err;

    if (asprintf(&log->rotate_filename, "%s.1", filename) < 0) {
        log->rotate_filename = NULL;
        goto err;
    }

    log->fd = open(log->filename, O_CREAT | O_APPEND | O_WRONLY,
                   S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR);
    if (log->fd < 0) {
        fprintf(stderr, "ERROR: log file open failed: %s: %s\n",
                log->filename, strerror(errno));
        goto err;
    }

    if (!log->maxsize)
        return 0;

    pos = lseek(log->fd, 0, SEEK_END);
    if (pos < 0) {
        fprintf(stderr, "ERROR: failed to query log file size: %s: %s\n",
                log->filename, strerror(errno));
        goto err;
    }

    log->size = (size_t)pos;
    if (log->size >= log->maxsize) {
        if (muxer_log_rotate(log))
            goto err;
    }

    return 0;

err:
    muxer_log_close(log);
    return -1;
}

static int muxer_log_write(struct muxer_log *log, const void *buf, size_t len)
{
    const unsigned char *data = buf;
    size_t written = 0;
    ssize_t ret;

    if (log->fd < 0 || len == 0)
        return 0;

    if (log->maxsize) {
        if (len > log->maxsize) {
            data += len - log->maxsize;
            len = log->maxsize;
        }

        if (log->size > log->maxsize - len) {
            if (muxer_log_rotate(log))
                return -1;
        }
    }

    while (written < len) {
        ret = write(log->fd, data + written, len - written);
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (ret == 0)
            return -1;
        written += (size_t)ret;
    }

    log->size += len;
    return 0;
}

static int muxer_log_flush(struct muxer_log *log)
{
    size_t length = log->write_buffer_length;

    if (length == 0)
        return 0;
    if (muxer_log_write(log, log->write_buffer, length) < 0)
        return -1;
    log->write_buffer_length = 0;
    return 0;
}

static int muxer_log_buffer(struct muxer_log *log, const void *buf, size_t len)
{
    const unsigned char *data = buf;

    while (len != 0) {
        size_t available = sizeof(log->write_buffer) - log->write_buffer_length;
        size_t copy;

        if (available == 0) {
            if (muxer_log_flush(log) < 0)
                return -1;
            continue;
        }
        copy = len < available ? len : available;
        memcpy(log->write_buffer + log->write_buffer_length, data, copy);
        log->write_buffer_length += copy;
        data += copy;
        len -= copy;
    }
    return 0;
}

static int muxer_log_write_char(struct muxer_log *log, unsigned char ch)
{
    char timestamp[30];
    char logbuf[sizeof(timestamp) + 1];
    size_t len_ts = 0;
    size_t log_len = 0;

    if (!log_timestamp_enabled) {
        if (muxer_log_buffer(log, &ch, 1) < 0)
            return -1;
        log->last_ch = ch;
        return 0;
    }

    if (!log->timestamp_pending &&
            (log->last_ch == '\n' || log->last_ch == '\r' ||
             log->last_ch == '\0')) {
        log->timestamp_pending = true;
    }

    if (log->timestamp_pending) {
        if (log->timestamp_escape_state) {
            if (muxer_log_buffer(log, &ch, 1) < 0)
                return -1;
            if (log->timestamp_escape_state == TIMESTAMP_ESC_START)
                log->timestamp_escape_state =
                    (ch == '[') ? TIMESTAMP_ESC_CSI : 0;
            else if (ch >= 0x40 && ch <= 0x7e)
                log->timestamp_escape_state = 0;
            log->last_ch = ch;
            return 0;
        }

        if (ch == '\033')
            log->timestamp_escape_state = TIMESTAMP_ESC_START;

        if (ch == '\033' || ch == '\r' || ch == '\n') {
            if (muxer_log_buffer(log, &ch, 1) < 0)
                return -1;
            if (ch == '\n')
                log->timestamp_pending = false;
            log->last_ch = ch;
            return 0;
        }

        len_ts = get_timestamp(timestamp, sizeof timestamp);
        if (0 < len_ts && len_ts < sizeof timestamp) {
            memcpy(logbuf, timestamp, len_ts);
            log_len += len_ts;
        }
        log->timestamp_pending = false;
    }

    logbuf[log_len++] = ch;
    if (muxer_log_buffer(log, logbuf, log_len) < 0)
        return -1;

    log->last_ch = ch;

    return 0;
}

static void report_output_stats(const char *reason)
{
    int index;
    struct thread_data *pty;
    struct raw_capture_stats raw_stats;

    fprintf(stderr, "INFO: UART output statistics snapshot: reason=%s\n", reason);
    for_each_pty(index, pty) {
        pthread_mutex_lock(&pty->output_lock);
        fprintf(stderr,
                "INFO: VSER %u output stats: bytes=%llu queued=%zu high_water=%zu "
                "eagain=%llu overflows=%llu log_errors=%llu "
                "shutdown_dropped=%llu unhealthy=%u "
                "input_bytes=%llu input_reads=%llu\n",
                pty->id, pty->output_bytes, pty->output_queue_length,
                pty->output_queue_high_water, pty->output_eagain,
                pty->output_overflows, pty->output_log_errors,
                pty->output_shutdown_dropped,
                pty->output_unhealthy,
                __atomic_load_n(&pty->input_bytes, __ATOMIC_RELAXED),
                __atomic_load_n(&pty->input_reads, __ATOMIC_RELAXED));
        pthread_mutex_unlock(&pty->output_lock);
    }

    memset(&raw_stats, 0, sizeof(raw_stats));
    if (native_raw_capture_enabled)
        raw_capture_get_stats(&raw_stats);
    fprintf(stderr,
            "INFO: physical/raw capture stats: enabled=%u reads=%llu physical_bytes=%llu "
            "parser_bytes=%llu records=%llu record_bytes=%llu dropped_records=%llu "
            "dropped_bytes=%llu gaps=%llu queue_high_water=%llu rotations=%llu io_errors=%llu\n",
            native_raw_capture_enabled, (unsigned long long)raw_stats.physical_reads,
            (unsigned long long)raw_stats.physical_bytes,
            __atomic_load_n(&parser_input_bytes, __ATOMIC_RELAXED),
            (unsigned long long)raw_stats.records_written,
            (unsigned long long)raw_stats.bytes_written,
            (unsigned long long)raw_stats.dropped_records,
            (unsigned long long)raw_stats.dropped_bytes,
            (unsigned long long)raw_stats.gap_records,
            (unsigned long long)raw_stats.queue_high_water,
            (unsigned long long)raw_stats.rotations,
            (unsigned long long)raw_stats.io_errors);
    fprintf(stderr,
            "INFO: TCU parser integrity: invalid_tags=%llu quarantined_bytes=%llu "
            "resynchronizations=%llu last_invalid_offset=%llu "
            "last_invalid_realtime_ns=%llu last_invalid_monotonic_ns=%llu\n",
            __atomic_load_n(&parser_invalid_tags, __ATOMIC_RELAXED),
            __atomic_load_n(&parser_quarantined_bytes, __ATOMIC_RELAXED),
            __atomic_load_n(&parser_resynchronizations, __ATOMIC_RELAXED),
            __atomic_load_n(&parser_last_invalid_offset, __ATOMIC_RELAXED),
            __atomic_load_n(&parser_last_invalid_realtime_ns, __ATOMIC_RELAXED),
            __atomic_load_n(&parser_last_invalid_monotonic_ns, __ATOMIC_RELAXED));
    fprintf(stderr,
            "INFO: physical TX stats: requested_bytes=%llu accepted_bytes=%llu "
            "write_calls=%llu short_writes=%llu eintr_retries=%llu "
            "eagain_retries=%llu poll_timeouts=%llu zero_writes=%llu "
            "fatal_errors=%llu\n",
            __atomic_load_n(&tx_requested_bytes, __ATOMIC_RELAXED),
            __atomic_load_n(&tx_accepted_bytes, __ATOMIC_RELAXED),
            __atomic_load_n(&tx_write_calls, __ATOMIC_RELAXED),
            __atomic_load_n(&tx_short_writes, __ATOMIC_RELAXED),
            __atomic_load_n(&tx_eintr_retries, __ATOMIC_RELAXED),
            __atomic_load_n(&tx_eagain_retries, __ATOMIC_RELAXED),
            __atomic_load_n(&tx_poll_timeouts, __ATOMIC_RELAXED),
            __atomic_load_n(&tx_zero_writes, __ATOMIC_RELAXED),
            __atomic_load_n(&tx_fatal_errors, __ATOMIC_RELAXED));
    if (physical_tty.initialized) {
        pthread_mutex_lock(&physical_tty.lock);
        fprintf(stderr,
                "INFO: physical TTY lifecycle: generation=%llu fd_available=%u "
                "reopen_requested=%u reopen_in_progress=%u failed=%u "
                "active_tx=%u requests=%llu attempts=%llu successes=%llu "
                "failures=%llu tx_retries=%llu\n",
                physical_tty.generation, tty_data.fd >= 0,
                physical_tty.reopen_requested,
                physical_tty.reopen_in_progress, physical_tty.failed,
                physical_tty.active_tx,
                physical_tty.reopen_requests, physical_tty.reopen_attempts,
                physical_tty.reopen_successes, physical_tty.reopen_failures,
                physical_tty.tx_retries);
        pthread_mutex_unlock(&physical_tty.lock);
    }
    if (physical_tx.data) {
        pthread_mutex_lock(&physical_tx.lock);
        fprintf(stderr,
                "INFO: physical TX queue: queued=%zu high_water=%zu "
                "capacity=%zu overflows=%llu shutdown_dropped=%llu "
                "unhealthy=%u backpressure_limit=%u "
                "backpressure_waits=%llu admission_timeouts=%llu\n",
                physical_tx.length, physical_tx.high_water,
                physical_tx.capacity, physical_tx.overflows,
                physical_tx.shutdown_dropped, physical_tx.unhealthy,
                PHYSICAL_TX_BACKPRESSURE_LIMIT,
                physical_tx.backpressure_waits,
                physical_tx.admission_timeouts);
        pthread_mutex_unlock(&physical_tx.lock);
    }
}

ut_static int init_pty_output_queue(struct thread_data *pty, size_t capacity)
{
    int status;

    pty->output_queue = malloc(capacity);
    if (!pty->output_queue)
        return -ENOMEM;

    pty->output_queue_capacity = capacity;
    pty->output_queue_head = 0;
    pty->output_queue_length = 0;
    pty->output_queue_high_water = 0;
    pty->output_bytes = 0;
    pty->output_eagain = 0;
    pty->output_overflows = 0;
    pty->output_log_errors = 0;
    pty->output_shutdown_dropped = 0;
    pty->input_bytes = 0;
    pty->input_reads = 0;
    pty->output_unhealthy = false;
    status = pthread_mutex_init(&pty->output_lock, NULL);
    if (status != 0) {
        free(pty->output_queue);
        pty->output_queue = NULL;
        return -status;
    }
    status = pthread_cond_init(&pty->output_ready, NULL);
    if (status != 0) {
        pthread_mutex_destroy(&pty->output_lock);
        free(pty->output_queue);
        pty->output_queue = NULL;
        return -status;
    }
    return 0;
}

ut_static void destroy_pty_output_queue(struct thread_data *pty)
{
    if (!pty->output_queue)
        return;
    pthread_cond_destroy(&pty->output_ready);
    pthread_mutex_destroy(&pty->output_lock);
    free(pty->output_queue);
    pty->output_queue = NULL;
}

ut_static int enqueue_pty_output(struct thread_data *pty,
                                 const unsigned char *data, size_t len)
{
    size_t tail;
    size_t first;

    pthread_mutex_lock(&pty->output_lock);
    if (pty->output_unhealthy) {
        pthread_mutex_unlock(&pty->output_lock);
        return -EIO;
    }
    /* Never evict old bytes: eviction would create a plausible-looking but
     * corrupt stream. Isolate and report only the affected VSER. */
    if (len > pty->output_queue_capacity - pty->output_queue_length) {
        pty->output_overflows++;
        pty->output_unhealthy = true;
        fprintf(stderr,
                "ERROR: VSER %u output queue overflow: queued=%zu requested=%zu "
                "capacity=%zu; isolating this VSER.\n",
                pty->id, pty->output_queue_length, len,
                pty->output_queue_capacity);
        pthread_mutex_unlock(&pty->output_lock);
        return -ENOSPC;
    }

    tail = (pty->output_queue_head + pty->output_queue_length) %
           pty->output_queue_capacity;
    first = pty->output_queue_capacity - tail;
    if (first > len)
        first = len;
    memcpy(pty->output_queue + tail, data, first);
    memcpy(pty->output_queue, data + first, len - first);
    pty->output_queue_length += len;
    if (pty->output_queue_length > pty->output_queue_high_water)
        pty->output_queue_high_water = pty->output_queue_length;
    pthread_cond_signal(&pty->output_ready);
    pthread_mutex_unlock(&pty->output_lock);
    return 0;
}

ut_static void *pty_output_handler(void *arg)
{
    struct thread_data *pty = arg;
    unsigned char buffer[4096];

    for (;;) {
        size_t available;
        size_t first;
        ssize_t written;
        struct timespec wake;

        if (consume_output_stats_request())
            report_output_stats("SIGUSR1");

        pthread_mutex_lock(&pty->output_lock);
        while (pty->output_queue_length == 0 && is_tcu_muxer_started() &&
               !output_stats_request_pending()) {
            clock_gettime(CLOCK_REALTIME, &wake);
            wake.tv_nsec += 100000000;
            if (wake.tv_nsec >= 1000000000) {
                wake.tv_sec++;
                wake.tv_nsec -= 1000000000;
            }
            pthread_cond_timedwait(&pty->output_ready, &pty->output_lock, &wake);
        }
        if (output_stats_request_pending()) {
            pthread_mutex_unlock(&pty->output_lock);
            continue;
        }
        /* Service shutdown must be bounded even when a PTY consumer has
         * stopped reading. Capture orchestration waits for the queues to
         * become idle before requesting shutdown; anything still queued here
         * therefore represents an incomplete evidence boundary. Account for
         * it explicitly instead of making systemd wait forever while this
         * worker tries to drain a permanently full nonblocking PTY. */
        if (!is_tcu_muxer_started()) {
            size_t discarded = pty->output_queue_length;

            if (discarded != 0) {
                pty->output_shutdown_dropped += discarded;
                pty->output_queue_head = 0;
                pty->output_queue_length = 0;
                pty->output_unhealthy = true;
                fprintf(stderr,
                        "WARNING: VSER %u discarded %zu queued bytes during "
                        "bounded shutdown.\n",
                        pty->id, discarded);
            }
            pthread_mutex_unlock(&pty->output_lock);
            break;
        }

        available = pty->output_queue_length;
        if (available > sizeof(buffer))
            available = sizeof(buffer);
        first = pty->output_queue_capacity - pty->output_queue_head;
        if (first > available)
            first = available;
        memcpy(buffer, pty->output_queue + pty->output_queue_head, first);
        memcpy(buffer + first, pty->output_queue, available - first);
        pthread_mutex_unlock(&pty->output_lock);

        written = write(pty->fd, buffer, available);
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct pollfd pfd = { .fd = pty->fd, .events = POLLOUT, .revents = 0 };
            int poll_status;

            pthread_mutex_lock(&pty->output_lock);
            pty->output_eagain++;
            pthread_mutex_unlock(&pty->output_lock);
            do {
                poll_status = poll(&pfd, 1, poll_output_timeout);
            } while (poll_status < 0 && errno == EINTR &&
                     is_tcu_muxer_started());
            /* Re-enter the top of the loop so a concurrent stop request uses
             * the bounded-shutdown path and accounts for the complete queue. */
            continue;
        }
        if (written < 0 && errno == EINTR)
            continue;
        if (written <= 0) {
            pthread_mutex_lock(&pty->output_lock);
            pty->output_unhealthy = true;
            pthread_mutex_unlock(&pty->output_lock);
            fprintf(stderr, "ERROR: VSER %u output failed: %s; isolating this VSER.\n",
                    pty->id, written == 0 ? "zero progress" : strerror(errno));
            break;
        }

        /* Log exactly the suffix accepted by the PTY. Rotation and timestamp
         * state remain owned by muxer_log, but disk latency is now isolated
         * to this VSER worker instead of the physical UART reader. Character
         * processing preserves timestamp state in memory; one flush per PTY
         * batch avoids a filesystem write for every received character. */
        if (pty->log.fd >= 0) {
            size_t index;
            bool log_failed = false;

            for (index = 0; index < (size_t)written; index++) {
                if (muxer_log_write_char(&pty->log, buffer[index]) < 0) {
                    log_failed = true;
                    break;
                }
            }
            if (!log_failed && muxer_log_flush(&pty->log) < 0)
                log_failed = true;
            if (log_failed) {
                pthread_mutex_lock(&pty->output_lock);
                pty->output_log_errors++;
                pty->output_unhealthy = true;
                pthread_mutex_unlock(&pty->output_lock);
                fprintf(stderr, "ERROR: VSER %u log write failed.\n", pty->id);
            }
        }

        pthread_mutex_lock(&pty->output_lock);
        pty->output_queue_head = (pty->output_queue_head + (size_t)written) %
                                 pty->output_queue_capacity;
        pty->output_queue_length -= (size_t)written;
        pty->output_bytes += (unsigned long long)written;
        pthread_mutex_unlock(&pty->output_lock);
    }
    return NULL;
}

static int init_physical_tx_queue(size_t capacity)
{
    int status;

    memset(&physical_tx, 0, sizeof(physical_tx));
    physical_tx.data = malloc(capacity);
    if (!physical_tx.data)
        return -ENOMEM;
    physical_tx.capacity = capacity;
    status = pthread_mutex_init(&physical_tx.lock, NULL);
    if (status != 0) {
        free(physical_tx.data);
        physical_tx.data = NULL;
        return -status;
    }
    status = pthread_cond_init(&physical_tx.ready, NULL);
    if (status != 0) {
        pthread_mutex_destroy(&physical_tx.lock);
        free(physical_tx.data);
        physical_tx.data = NULL;
        return -status;
    }
    status = pthread_cond_init(&physical_tx.space_available, NULL);
    if (status != 0) {
        pthread_cond_destroy(&physical_tx.ready);
        pthread_mutex_destroy(&physical_tx.lock);
        free(physical_tx.data);
        physical_tx.data = NULL;
        return -status;
    }
    return 0;
}

static void destroy_physical_tx_queue(void)
{
    if (!physical_tx.data)
        return;
    pthread_cond_destroy(&physical_tx.space_available);
    pthread_cond_destroy(&physical_tx.ready);
    pthread_mutex_destroy(&physical_tx.lock);
    free(physical_tx.data);
    memset(&physical_tx, 0, sizeof(physical_tx));
}

static int init_physical_tty_lifecycle(void)
{
    int status;

    memset(&physical_tty, 0, sizeof(physical_tty));
    status = pthread_mutex_init(&physical_tty.lock, NULL);
    if (status != 0)
        return -status;
    status = pthread_cond_init(&physical_tty.changed, NULL);
    if (status != 0) {
        pthread_mutex_destroy(&physical_tty.lock);
        return -status;
    }
    physical_tty.initialized = true;
    return 0;
}

static void destroy_physical_tty_lifecycle(void)
{
    if (!physical_tty.initialized)
        return;
    pthread_cond_destroy(&physical_tty.changed);
    pthread_mutex_destroy(&physical_tty.lock);
    memset(&physical_tty, 0, sizeof(physical_tty));
}

/* The caller holds physical_tty.lock. Coalesce simultaneous RX and TX
 * observations of the same disconnect into one reopen request. */
static void request_physical_tty_reopen_locked(void)
{
    if (!physical_tty.reopen_requested && !physical_tty.reopen_in_progress) {
        physical_tty.reopen_requested = true;
        physical_tty.reopen_requests++;
    }
    pthread_cond_broadcast(&physical_tty.changed);
}

static bool physical_tty_reopen_pending(void)
{
    bool pending;

    pthread_mutex_lock(&physical_tty.lock);
    pending = physical_tty.reopen_requested;
    pthread_mutex_unlock(&physical_tty.lock);
    return pending;
}

/* RX owns descriptor replacement. New TX transactions are excluded once a
 * reopen is requested, and replacement waits for an in-flight transaction to
 * finish before closing the descriptor. Holding the lifecycle lock across
 * close, open, termios setup, and the reset sequence then prevents TX from
 * observing an unpublished descriptor or interleaving bytes with the reset. */
static int reopen_physical_tty(void)
{
    int old_fd;
    int new_fd;

    pthread_mutex_lock(&physical_tty.lock);
    request_physical_tty_reopen_locked();
    if (physical_tty.reopen_in_progress) {
        pthread_mutex_unlock(&physical_tty.lock);
        return 0;
    }

    physical_tty.reopen_requested = false;
    physical_tty.reopen_in_progress = true;
    physical_tty.reopen_attempts++;
    while (is_tcu_muxer_started() && physical_tty.active_tx != 0)
        pthread_cond_wait(&physical_tty.changed, &physical_tty.lock);
    if (!is_tcu_muxer_started()) {
        physical_tty.reopen_in_progress = false;
        pthread_cond_broadcast(&physical_tty.changed);
        pthread_mutex_unlock(&physical_tty.lock);
        return -ECANCELED;
    }
    old_fd = tty_data.fd;
    tty_data.fd = -1;
    if (old_fd >= 0)
        close(old_fd);

    new_fd = open_tty_device();
    if (new_fd < 0) {
        physical_tty.reopen_failures++;
        physical_tty.failed = true;
        physical_tty.reopen_in_progress = false;
        pthread_cond_broadcast(&physical_tty.changed);
        pthread_mutex_unlock(&physical_tty.lock);
        return new_fd;
    }

    tty_data.fd = new_fd;
    physical_tty.generation++;
    physical_tty.reopen_successes++;
    physical_tty.reopen_in_progress = false;
    pthread_cond_broadcast(&physical_tty.changed);
    pthread_mutex_unlock(&physical_tty.lock);
    return 0;
}

static bool physical_tty_disconnect_error(int status)
{
    return status == -ENODEV || status == -EBADF || status == -ENXIO;
}

/* Pin the descriptor generation while a complete poll/write transaction is
 * active. The lifecycle mutex protects only ownership metadata; it is not
 * held during normal physical I/O. This keeps RX responsive during sustained
 * full-duplex traffic while guaranteeing that reopen cannot close the file
 * descriptor until the transaction releases it. A disconnect leaves the TX
 * queue head untouched, asks RX to reopen, and retries the same snapshot only
 * after a newer generation is published. */
static int write_physical_tx_snapshot(const unsigned char *buffer, size_t len)
{
    for (;;) {
        unsigned long long generation;
        int fd;
        int status;

        pthread_mutex_lock(&physical_tty.lock);
        while (is_tcu_muxer_started() && !physical_tty.failed &&
               (physical_tty.reopen_requested ||
                physical_tty.reopen_in_progress || tty_data.fd < 0))
            pthread_cond_wait(&physical_tty.changed, &physical_tty.lock);
        if (!is_tcu_muxer_started()) {
            pthread_mutex_unlock(&physical_tty.lock);
            return -ECANCELED;
        }
        if (physical_tty.failed) {
            pthread_mutex_unlock(&physical_tty.lock);
            return -ENODEV;
        }

        generation = physical_tty.generation;
        fd = tty_data.fd;
        physical_tty.active_tx++;
        pthread_mutex_unlock(&physical_tty.lock);

        status = putbuf_or_exit(fd, buffer, len);

        pthread_mutex_lock(&physical_tty.lock);
        physical_tty.active_tx--;
        pthread_cond_broadcast(&physical_tty.changed);
        if (!physical_tty_disconnect_error(status)) {
            pthread_mutex_unlock(&physical_tty.lock);
            return status;
        }

        physical_tty.tx_retries++;
        request_physical_tty_reopen_locked();
        while (is_tcu_muxer_started() && !physical_tty.failed &&
               physical_tty.generation == generation)
            pthread_cond_wait(&physical_tty.changed, &physical_tty.lock);
        if (physical_tty.failed) {
            pthread_mutex_unlock(&physical_tty.lock);
            return -ENODEV;
        }
        pthread_mutex_unlock(&physical_tty.lock);
    }
}

static int enqueue_physical_tx(const unsigned char *data, size_t len)
{
    size_t tail;
    size_t first;
    size_t previous_length;
    struct timespec deadline;
    int wait_status = 0;

    pthread_mutex_lock(&physical_tx.lock);
    if (physical_tx.unhealthy) {
        pthread_mutex_unlock(&physical_tx.lock);
        return -EIO;
    }
    /* Never partially enqueue a TCU-encoded block. Partial admission would
     * turn a visible overload into a syntactically plausible truncated
     * command. The caller reports ENOSPC and the end-to-end target ledger
     * classifies the command-delivery failure. */
    if (len > physical_tx.capacity || len > PHYSICAL_TX_BACKPRESSURE_LIMIT) {
        physical_tx.overflows++;
        physical_tx.unhealthy = true;
        fprintf(stderr,
                "ERROR: physical TX queue overflow: queued=%zu requested=%zu "
                "capacity=%zu; rejecting the complete encoded block.\n",
                physical_tx.length, len, physical_tx.capacity);
        request_muxer_stop();
        pthread_mutex_unlock(&physical_tx.lock);
        return -ENOSPC;
    }

    /* Stop draining the VSER PTY once the smoothing window is full. Blocking
     * here is intentional: the PTY and obmc-console then propagate pressure
     * to the SSH channel, so Paramiko sendall() cannot outrun every receiver
     * and disappear into unrelated userspace buffers. A wake without queue
     * progress does not reset the deadline; each successful dequeue does. */
    previous_length = physical_tx.length;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += poll_output_timeout / 1000;
    deadline.tv_nsec += (long)(poll_output_timeout % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }
    while (physical_tx.length > PHYSICAL_TX_BACKPRESSURE_LIMIT - len &&
           !physical_tx.unhealthy && is_tcu_muxer_started()) {
        physical_tx.backpressure_waits++;
        wait_status = pthread_cond_timedwait(&physical_tx.space_available,
                                             &physical_tx.lock, &deadline);
        if (physical_tx.length < previous_length) {
            previous_length = physical_tx.length;
            clock_gettime(CLOCK_REALTIME, &deadline);
            deadline.tv_sec += poll_output_timeout / 1000;
            deadline.tv_nsec += (long)(poll_output_timeout % 1000) * 1000000L;
            if (deadline.tv_nsec >= 1000000000L) {
                deadline.tv_sec++;
                deadline.tv_nsec -= 1000000000L;
            }
        } else if (wait_status == ETIMEDOUT) {
            physical_tx.admission_timeouts++;
            physical_tx.unhealthy = true;
            fprintf(stderr,
                    "ERROR: physical TX admission made no progress for %d ms: "
                    "queued=%zu requested=%zu limit=%u; failing closed.\n",
                    poll_output_timeout, physical_tx.length, len,
                    PHYSICAL_TX_BACKPRESSURE_LIMIT);
            request_muxer_stop();
            pthread_mutex_unlock(&physical_tx.lock);
            return -ETIMEDOUT;
        }
    }
    if (!is_tcu_muxer_started() || physical_tx.unhealthy) {
        pthread_mutex_unlock(&physical_tx.lock);
        return -ECANCELED;
    }

    tail = (physical_tx.head + physical_tx.length) % physical_tx.capacity;
    first = physical_tx.capacity - tail;
    if (first > len)
        first = len;
    memcpy(physical_tx.data + tail, data, first);
    memcpy(physical_tx.data, data + first, len - first);
    physical_tx.length += len;
    if (physical_tx.length > physical_tx.high_water)
        physical_tx.high_water = physical_tx.length;
    pthread_cond_signal(&physical_tx.ready);
    pthread_mutex_unlock(&physical_tx.lock);
    return 0;
}

static void *physical_tx_handler(void *arg)
{
    unsigned char buffer[PHYSICAL_TX_WRITE_SIZE];

    (void)arg;
    for (;;) {
        size_t available;
        size_t first;
        int status;

        pthread_mutex_lock(&physical_tx.lock);
        while (physical_tx.length == 0 && is_tcu_muxer_started())
            pthread_cond_wait(&physical_tx.ready, &physical_tx.lock);
        if (!is_tcu_muxer_started()) {
            if (physical_tx.length != 0) {
                physical_tx.shutdown_dropped += physical_tx.length;
                physical_tx.head = 0;
                physical_tx.length = 0;
                physical_tx.unhealthy = true;
            }
            pthread_mutex_unlock(&physical_tx.lock);
            break;
        }

        available = physical_tx.length;
        if (available > sizeof(buffer))
            available = sizeof(buffer);
        first = physical_tx.capacity - physical_tx.head;
        if (first > available)
            first = available;
        memcpy(buffer, physical_tx.data + physical_tx.head, first);
        memcpy(buffer + first, physical_tx.data, available - first);
        pthread_mutex_unlock(&physical_tx.lock);

        /* The lifecycle wrapper prevents close/reopen from racing this
         * transaction. It also preserves the queue head and retries this
         * exact snapshot after a successful descriptor replacement. */
        status = write_physical_tx_snapshot(buffer, available);
        if (status < 0) {
            pthread_mutex_lock(&physical_tx.lock);
            physical_tx.unhealthy = true;
            pthread_mutex_unlock(&physical_tx.lock);
            request_muxer_stop();
            break;
        }

        pthread_mutex_lock(&physical_tx.lock);
        physical_tx.head = (physical_tx.head + available) % physical_tx.capacity;
        physical_tx.length -= available;
        pthread_cond_broadcast(&physical_tx.space_available);
        pthread_mutex_unlock(&physical_tx.lock);
    }
    return NULL;
}


bool should_retry_io(ssize_t ret)
{
    return (ret == -1) &&
        (
        errno == EINTR ||
        errno == EAGAIN ||
        errno == EWOULDBLOCK
        );
}

static int set_tx_progress_deadline(struct timespec *deadline)
{
    if (clock_gettime(CLOCK_MONOTONIC, deadline))
        return -errno;
    deadline->tv_sec += poll_output_timeout / 1000;
    deadline->tv_nsec += (long)(poll_output_timeout % 1000) * 1000000L;
    if (deadline->tv_nsec >= 1000000000L) {
        deadline->tv_sec++;
        deadline->tv_nsec -= 1000000000L;
    }
    return 0;
}

static int tx_progress_time_left_ms(const struct timespec *deadline)
{
    struct timespec now;
    time_t seconds;
    long nanoseconds;
    long long milliseconds;

    if (clock_gettime(CLOCK_MONOTONIC, &now))
        return -errno;
    seconds = deadline->tv_sec - now.tv_sec;
    nanoseconds = deadline->tv_nsec - now.tv_nsec;
    if (nanoseconds < 0) {
        seconds--;
        nanoseconds += 1000000000L;
    }
    if (seconds < 0 || (seconds == 0 && nanoseconds == 0))
        return 0;
    milliseconds = (long long)seconds * 1000 +
        (nanoseconds + 999999L) / 1000000L;
    return milliseconds > INT_MAX ? INT_MAX : (int)milliseconds;
}

int putbuf_or_exit(int fd, const unsigned char *buf, size_t len)
{
    struct timespec progress_deadline;
    size_t offset = 0;
    int status;

    /* Count each physical submission attempt. If a disconnect accepts only a
     * prefix before the full snapshot is retried, requested/accepted totals
     * retain that fact instead of hiding bytes already handed to the driver. */
    __atomic_fetch_add(&tx_requested_bytes, len, __ATOMIC_RELAXED);
    status = set_tx_progress_deadline(&progress_deadline);
    if (status) {
        __atomic_fetch_add(&tx_fatal_errors, 1, __ATOMIC_RELAXED);
        fprintf(stderr, "ERROR: failed to read monotonic clock: %s\n",
                strerror(-status));
        return status;
    }

    while (offset < len) {
        struct pollfd pfd = { .fd = fd, .events = POLLOUT, .revents = 0 };
        ssize_t ret = 0;
        int timeout_ms;

        if (!is_tcu_muxer_started())
            return -ECANCELED;

        do {
            timeout_ms = tx_progress_time_left_ms(&progress_deadline);
            if (timeout_ms <= 0)
                break;
            ret = poll(&pfd, 1, timeout_ms);
            if (ret < 0 && errno == EINTR)
                __atomic_fetch_add(&tx_eintr_retries, 1, __ATOMIC_RELAXED);
        } while (ret < 0 && errno == EINTR && is_tcu_muxer_started());
        if (!is_tcu_muxer_started())
            return -ECANCELED;
        if (timeout_ms < 0) {
            __atomic_fetch_add(&tx_fatal_errors, 1, __ATOMIC_RELAXED);
            fprintf(stderr, "ERROR: failed to read monotonic clock: %s\n",
                    strerror(-timeout_ms));
            return timeout_ms;
        }
        if (timeout_ms == 0) {
            __atomic_fetch_add(&tx_poll_timeouts, 1, __ATOMIC_RELAXED);
            fprintf(stderr, "ERROR: TTY output made no progress for %d ms.\n",
                    poll_output_timeout);
            return -ETIMEDOUT;
        }
        if (ret == 0) {
            __atomic_fetch_add(&tx_poll_timeouts, 1, __ATOMIC_RELAXED);
            fprintf(stderr, "ERROR: poll() on TTY device timed out.\n");
            return -ETIMEDOUT;
        }
        if (ret < 0) {
            __atomic_fetch_add(&tx_fatal_errors, 1, __ATOMIC_RELAXED);
            fprintf(stderr, "ERROR: poll() on TTY device failed: %s\n",
                    strerror(errno));
            return -errno;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            fprintf(stderr, "WARNING: TTY output requires reopen: revents=0x%x\n",
                    pfd.revents);
            return -ENODEV;
        }

        ret = write(fd, buf + offset, len - offset);
        __atomic_fetch_add(&tx_write_calls, 1, __ATOMIC_RELAXED);
        if (ret > 0) {
            if ((size_t)ret < len - offset)
                __atomic_fetch_add(&tx_short_writes, 1, __ATOMIC_RELAXED);
            __atomic_fetch_add(&tx_accepted_bytes, (unsigned long long)ret,
                               __ATOMIC_RELAXED);
            offset += (size_t)ret;
            status = set_tx_progress_deadline(&progress_deadline);
            if (status) {
                __atomic_fetch_add(&tx_fatal_errors, 1, __ATOMIC_RELAXED);
                fprintf(stderr,
                        "ERROR: failed to read monotonic clock: %s\n",
                        strerror(-status));
                return status;
            }
            continue;
        }
        if (ret < 0 && errno == EINTR && is_tcu_muxer_started()) {
            __atomic_fetch_add(&tx_eintr_retries, 1, __ATOMIC_RELAXED);
            continue;
        }
        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) &&
            is_tcu_muxer_started()) {
            __atomic_fetch_add(&tx_eagain_retries, 1, __ATOMIC_RELAXED);
            continue;
        }
        if (ret == 0)
            __atomic_fetch_add(&tx_zero_writes, 1, __ATOMIC_RELAXED);
        else if (errno != EBADF && errno != ENODEV && errno != ENXIO &&
                 errno != EIO)
            __atomic_fetch_add(&tx_fatal_errors, 1, __ATOMIC_RELAXED);
        if (ret < 0 && (errno == EBADF || errno == ENODEV || errno == ENXIO ||
                       errno == EIO)) {
            fprintf(stderr, "WARNING: physical TTY write requires reopen: %s\n",
                    strerror(errno));
            return -ENODEV;
        }
        fprintf(stderr, "ERROR: failed to make progress writing TTY: %s\n",
                ret == 0 ? "zero-byte write" : strerror(errno));
        return ret == 0 ? -EIO : -errno;
    }

    return 0;
}
int putch_or_exit(int fd, const unsigned char ch)
{
    ssize_t ret;
    struct pollfd pfd;

    pfd.fd = fd;
    pfd.events = POLLOUT;
    pfd.revents = 0;

    do {
        ret = poll(&pfd, 1, poll_output_timeout /* ms */);
    } while (should_retry_io(ret));

    if (ret == 0) {
        fprintf(stderr, "ERROR: poll() on TTY device timed out.\n");
        return -1;
    }
    if (ret < 0) {
        fprintf(stderr, "ERROR: poll() on TTY device failed: %s\n",
                strerror(errno));
        return -1;
    }
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        fprintf(stderr, "ERROR: TTY unavailable while writing reset: revents=0x%x\n",
                pfd.revents);
        return -1;
    }

    do {
        ret = write(fd, &ch, 1);
    } while (should_retry_io(ret));

    if (ret <= 0) {
        fprintf(stderr, "ERROR: failed to write - %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

size_t get_timestamp(char *buf, const size_t maxsize)
{
    struct timeval tv;
    struct tm tm;
    int len = 0;

    if (buf && maxsize && !gettimeofday(&tv, NULL) &&
            localtime_r(&tv.tv_sec, &tm)) {
        len = snprintf(buf, maxsize, "[%04d-%02d-%02d %02d:%02d:%02d.%06ld] ",
                tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec, (long)tv.tv_usec);
        if (0 < len && len < (int)maxsize ) {
            /* len should less than maxsize since at least one byte */
            /* must be reserved for the terminating null character */
            return (size_t)len;
        }
    }
    return 0;
}

int flush_stream(int fd, int pty_idx, unsigned char ch)
{
    (void)fd;
    if (pty_idx < 0 || pty_idx >= pty_max_count)
        return -EINVAL;
    return enqueue_pty_output(&pty_data[pty_idx], &ch, 1);
}

int patch2flush_stream(int fd, int pty_idx, unsigned char ch, int *p_seen_n, int *p_seen_r)
{
    unsigned char patch_ch = 0;
    int ret_val = 0;

    if (disable_patch) {
        return flush_stream(fd, pty_idx, ch);
    }

    if (ch == '\n') {
        *p_seen_n = 1;
    } else if (ch == '\r') {
        *p_seen_r = 1;
    } else {
        if (*p_seen_r) {
            // write a \n
            patch_ch = '\n';
            flush_stream(fd, pty_idx, patch_ch);
            *p_seen_r = 0;
            *p_seen_n = 0;
        }
        if (*p_seen_n) {
            // write a \r
            patch_ch = '\r';
            flush_stream(fd, pty_idx, patch_ch);
            *p_seen_r = 0;
            *p_seen_n = 0;
        }
    }
    if ((ret_val = flush_stream(fd, pty_idx, ch)) < 0)
        return ret_val;
    if (*p_seen_r && *p_seen_n) {
        *p_seen_r = 0;
        *p_seen_n = 0;
    }

    return 0;
}

static int append_rx_byte(struct rx_route_batch *batch, unsigned char ch)
{
    if (batch->length >= sizeof(batch->data))
        return -ENOSPC;
    batch->data[batch->length++] = ch;
    return 0;
}

/* Preserve the legacy CR/LF completion rule while writing into a local batch.
 * The output is semantically identical to patch2flush_stream(); only the
 * queue-lock and worker-wakeup frequency changes. */
static int patch2batch_stream(struct rx_route_batch *batch, unsigned char ch,
                              int *p_seen_n, int *p_seen_r)
{
    int status;

    if (disable_patch)
        return append_rx_byte(batch, ch);

    if (ch == '\n') {
        *p_seen_n = 1;
    } else if (ch == '\r') {
        *p_seen_r = 1;
    } else {
        if (*p_seen_r) {
            status = append_rx_byte(batch, '\n');
            if (status < 0)
                return status;
            *p_seen_r = 0;
            *p_seen_n = 0;
        }
        if (*p_seen_n) {
            status = append_rx_byte(batch, '\r');
            if (status < 0)
                return status;
            *p_seen_r = 0;
            *p_seen_n = 0;
        }
    }
    status = append_rx_byte(batch, ch);
    if (status < 0)
        return status;
    if (*p_seen_r && *p_seen_n) {
        *p_seen_r = 0;
        *p_seen_n = 0;
    }
    return 0;
}

ut_static void uucp_unlock_tty_device(void)
{
    if (filelock[0] && uucp_locked) {
        unlink(filelock);
        uucp_locked = false;
    }
}

static void handle_stop_request(int sig)
{
    (void)sig;
    /* Storing one atomic flag is async-signal-safe. Worker wakeups, joins,
     * capture draining and file cleanup remain in normal thread context. */
    request_muxer_stop();
}

ut_static int uucp_set_filelock_name(void)
{
    char *ptr;
    char buf[MAX_PATH];
    int ret, len, i;

    ptr = strchr(tty_device + 1, '/');
    ptr = ptr ? ptr + 1 : (char*)tty_device;
    strncpy(buf, ptr, sizeof(buf) - 1);
    ptr = buf;
    len = strlen(ptr);
    for (i = 0; i < len; i++) {
        if (ptr[i] == '/')
            ptr[i] = '_';
    }

    ret = snprintf(filelock, sizeof(filelock), "%s/LCK..%s", UUCP_DIR, ptr);
    if (ret < 0 || (size_t)ret >= sizeof(filelock))
        return -1;

    return 0;
}

ut_static bool uucp_check_is_locked(void)
{
    char buf[MAX_PATH];
    int fd, n, pid;

    fd = open(filelock, O_RDONLY);
    if (fd >= 0) {
        memset(buf, 0, sizeof(buf));
        n = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        pid = (n == 4) ? *(int *)buf : strtol(buf, NULL, 10);
        if (pid > 0 && kill((pid_t)pid, 0) < 0 && errno == ESRCH) {
            /* stale lock file */
            fprintf(stdout, "Removing stale lock: %s\n", filelock);
            sleep(1);
            unlink(filelock);
        } else {
            return true;
        }
    }

    return false;
}

ut_static int uucp_lock_tty_device(void)
{
    char buf[MAX_PATH];
    int ret, fd;

    fd = open(filelock, O_WRONLY|O_CREAT|O_EXCL, 0666);
    if ( fd < 0 )
        return -1;
    ret = snprintf(buf, sizeof(buf), "%10d\n", getpid());
    if (ret < 0 || (size_t)ret >= sizeof(buf)) {
        close(fd);
        return -1;
    }
    if (write(fd, buf, strlen(buf)) < 0){
        close(fd);
        return -2;
    }
    close(fd);

    uucp_locked = true;
    return 0;
}

ut_static int open_tty_device(void)
{
    int fd = -1;
    speed_t baudrate;
    struct flock tty_flock;
    struct termios options;
    struct stat sbuf;
    unsigned char chr;
    int retries = 40;

    // use uucp locking if lock file directory is present
    // since minicom still uses it
    if (stat(UUCP_DIR, &sbuf) == 0 && !uucp_locked) {
        if (uucp_set_filelock_name())  {
            fprintf(stderr, "ERROR: unable to set the filelock name\n");
            goto err;
        }
        if (uucp_check_is_locked() || uucp_lock_tty_device()) {
            fprintf(stderr, "ERROR: unable to obtain file lock\n");
            goto err;
        }
    }

    // bounds for -d arg
    while ((fd = open(tty_device, O_RDWR|O_NOCTTY)) < 0) {
        if (--retries < 0)
            break;
        usleep(50000);
    }
    if (fd < 0) {
        fprintf(stderr, "ERROR: failed to open %s\n", tty_device);
        goto err;
    }

    // bounds for -r arg
    if (invalid_baudrate(int_baudrate)) {
        fprintf(stderr, "ERROR: invalid baudrate\n");
        goto err;
    }
    baudrate = get_baudrate(int_baudrate);

    // try to lock tty device
    tty_flock.l_type = F_WRLCK;
    tty_flock.l_whence = SEEK_SET;
    tty_flock.l_start = 0;
    tty_flock.l_len = 0;
    if (fcntl(fd, F_SETLK, &tty_flock)) {
        fprintf(stderr, "ERROR: TTY %s possibly locked by other uart_muxer instance!\n", tty_device);
        goto err;
    }

    tcgetattr(fd, &options);

    // set baudrate for in/out
    if (cfsetispeed(&options, baudrate)) {
        fprintf(stderr, "ERROR: failed to set in_baudrate %d\n", int_baudrate);
        goto err;
    }
    if (cfsetospeed(&options, baudrate)) {
        fprintf(stderr, "ERROR: failed to set out_baudrate %d\n", int_baudrate);
        goto err;
    }

    cfmakeraw (&options);

    /* MIN == 0; TIME == 0: If data is available, read(2) returns
     * immediately, with the lesser of the number of bytes available,
     * or the number of bytes requested. If no data is available, read(2)
     * returns 0 */
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    options.c_iflag &= ~(INLCR | ICRNL | IGNPAR);
    options.c_iflag |= IGNBRK;

    options.c_oflag &= ~(ONLCR | OCRNL | ONOCR);
    options.c_lflag &= ~(IEXTEN | ECHO | ECHOK | ECHOE | ECHOKE | ECHOCTL | ECHONL);
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        fprintf(stderr, "ERROR: on tcsetattr - %s\n", strerror(errno));
        goto err;
    }

    //TODO: Confirm that all attributes were set correctly

    chr = '\0';
    if (putch_or_exit(fd, chr) < 0)
        goto err;
    chr = UART_PROTO_ESC_START;
    if (putch_or_exit(fd, chr) < 0)
        goto err;
    chr = UART_PROTO_ESC_RESET;
    if (putch_or_exit(fd, chr) < 0)
        goto err;

    return fd;

err:
    fprintf(stderr, "%s: failed\n", __func__);
    if (fd >= 0)
        close(fd);
    return -1;
}

void* tty_input_handler(void *arg)
{
    (void)arg;
    unsigned char *buf = NULL;
    struct rx_route_batch *batches = NULL;
    struct pollfd pfd;

    unsigned char ch;
    bool tag_match = false;
    bool in_escape = false;
    bool routing_synchronized = true;
    unsigned long long physical_offset = 0;
    int cur_rx_guest = 0;
    int tag_idx;
    const struct tag *tag;

    int index;
    ssize_t len;
    int ret_val;
    int seen_n[pty_max_count]; // flag set for seeing '\n'
    int seen_r[pty_max_count]; // flag set for seeing '\r'

    memset(seen_n, 0, sizeof(seen_n));
    memset(seen_r, 0, sizeof(seen_r));

    /* Drain the physical TTY in larger batches than an individual PTY input.
     * The USB-serial driver commonly exposes several KiB at once during a
     * firmware burst; limiting each read to 1 KiB needlessly multiplies
     * syscall and parser scheduling gaps at the only pre-demux boundary. */
    buf = malloc(PHYSICAL_RX_BUFFER_SIZE);
    if (!buf) {
        fprintf(stderr,
            "ERROR: Failed to allocate temporary buffer with size %zu bytes\n",
            (size_t)PHYSICAL_RX_BUFFER_SIZE);
        goto out;
    }
    batches = calloc(pty_max_count, sizeof(*batches));
    if (!batches) {
        fprintf(stderr, "ERROR: failed to allocate RX routing batches\n");
        goto out;
    }

    while (is_tcu_muxer_started()) {
        /* TX may observe descriptor loss before RX does. Service its request
         * before polling again so the TX worker is not left waiting for a
         * generation change until another RX event happens. */
        if (physical_tty_reopen_pending()) {
            if (reopen_physical_tty() < 0) {
                fprintf(stderr, "ERROR: tty_input_handler: reopen failed\n");
                request_muxer_stop();
                goto out;
            }
            continue;
        }
        pfd.fd = tty_data.fd;
        pfd.events = POLLIN;
        pfd.revents = 0;

        /* A bounded wait lets SIGINT/SIGTERM stop the service without doing
         * unsafe cleanup in the signal handler. It does not poll the UART
         * continuously; the kernel still sleeps until data or timeout. */
        ret_val = poll(&pfd, 1, 100);

        if (should_retry_io(ret_val))
            continue;
        if (ret_val == 0)
            continue;
        assert(ret_val == 1);

        if (pfd.revents & (POLLHUP|POLLERR|POLLNVAL)) {
#ifdef UNIT_TEST
            // This additional check is needed to allow the thread to exit gracefully during unit testing.
            // Poll() returns POLLUP when the mock device closes at the end of a test, so we check here if the test
            // is over to avoid failing the test.
            if(!is_tcu_muxer_started())
                goto out;
#endif
            if (reopen_physical_tty() == 0) {
                continue;
            } else {
                fprintf(stderr, "ERROR: tty_input_handler: hangup\n");
                request_muxer_stop();
                goto out;
            }
        }

        len = read(tty_data.fd, buf, PHYSICAL_RX_BUFFER_SIZE);
        if (len < 0) {
            if (errno == EBADF || errno == ENODEV || errno == ENXIO ||
                errno == EIO) {
                if (reopen_physical_tty() == 0)
                    continue;
            }
            fprintf(stderr, "ERROR: failed to read\n");
            request_muxer_stop();
            goto out;
        }
        if (len > 0) {
            /* Capture the exact successful physical read before raw logging,
             * framing, routing, timestamping, or PTY backpressure. */
            raw_capture_record(buf, (size_t)len);
            __atomic_add_fetch(&parser_input_bytes, (unsigned long long)len,
                               __ATOMIC_RELAXED);
        }
        if (tty_data.log.fd >= 0) {
            if (muxer_log_write(&tty_data.log, buf, (size_t)len) < 0) {
                fprintf(stderr, "ERROR: failed to write\n");
                request_muxer_stop();
                goto out;
            }
        }

        for (index = 0; index < pty_max_count; index++)
            batches[index].length = 0;

        for (index = 0; index < len; index++) {
            unsigned long long current_offset = physical_offset++;
            ch = buf[index];

            ret_val = patch2batch_stream(&batches[raw_pty_idx], ch,
                                         &seen_n[raw_pty_idx],
                                         &seen_r[raw_pty_idx]);
            if (ret_val < 0)
                goto route_failed;

            if (in_escape) {
                in_escape = false;
                // Handle UTC control characters
                switch (ch) {
                    case UART_PROTO_ESC_ESC:
                        ch = UART_PROTO_ESC_START;
                        goto not_escape_ch;

                    case UART_PROTO_ESC_RESET:
                        cur_rx_guest = default_tag_idx;
                        if (!routing_synchronized)
                            __atomic_fetch_add(&parser_resynchronizations, 1,
                                               __ATOMIC_RELAXED);
                        routing_synchronized = true;
                        break;

                    default:
                        // Check for tag matches
                        tag_match = false;
                        for_each_tags(tag_idx, tag) {
                            if (ch == tag->value) {
                                cur_rx_guest = tag_idx;
                                tag_match = true;
                                if (!routing_synchronized)
                                    __atomic_fetch_add(&parser_resynchronizations, 1,
                                                       __ATOMIC_RELAXED);
                                routing_synchronized = true;
                                break;
                            }
                        }
                        if (!tag_match) {
                            struct timespec realtime;
                            struct timespec monotonic;
                            unsigned long long realtime_ns = 0;
                            unsigned long long monotonic_ns = 0;

                            if (clock_gettime(CLOCK_REALTIME, &realtime) == 0)
                                realtime_ns = (unsigned long long)realtime.tv_sec *
                                              1000000000ULL + realtime.tv_nsec;
                            if (clock_gettime(CLOCK_MONOTONIC, &monotonic) == 0)
                                monotonic_ns = (unsigned long long)monotonic.tv_sec *
                                               1000000000ULL + monotonic.tv_nsec;
                            __atomic_fetch_add(&parser_invalid_tags, 1,
                                               __ATOMIC_RELAXED);
                            __atomic_store_n(&parser_last_invalid_offset,
                                             current_offset, __ATOMIC_RELAXED);
                            __atomic_store_n(&parser_last_invalid_realtime_ns,
                                             realtime_ns, __ATOMIC_RELAXED);
                            __atomic_store_n(&parser_last_invalid_monotonic_ns,
                                             monotonic_ns, __ATOMIC_RELAXED);
                            fprintf(stderr,
                                    "ERROR: invalid TCU control sequence FF %02X at "
                                    "physical_offset=%llu realtime_ns=%llu "
                                    "monotonic_ns=%llu previous_vser=%d; "
                                    "quarantining until a valid tag or reset.\n",
                                    ch, current_offset, realtime_ns, monotonic_ns,
                                    cur_rx_guest);
                            routing_synchronized = false;
                        }
                        break;
                }
            } else {
                if (ch == UART_PROTO_ESC_START) {
                    in_escape = true;
                } else {
not_escape_ch:
                        if (!routing_synchronized) {
                            __atomic_fetch_add(&parser_quarantined_bytes, 1,
                                               __ATOMIC_RELAXED);
                            continue;
                        }
                        assert(cur_rx_guest < (int) (pty_max_count));
                        ret_val = patch2batch_stream(&batches[cur_rx_guest], ch,
                                                    &seen_n[cur_rx_guest],
                                                    &seen_r[cur_rx_guest]);
                        if (ret_val < 0)
                            goto route_failed;
                }
            }
        }

        /* A physical read is now handed to each touched VSER with one queue
         * transaction.  Per-VSER byte order and parser state are unchanged,
         * while the physical reader avoids thousands of mutex/signal pairs. */
        for (index = 0; index < pty_max_count; index++) {
            if (batches[index].length == 0)
                continue;
            ret_val = enqueue_pty_output(&pty_data[index], batches[index].data,
                                         batches[index].length);
            if (ret_val < 0)
                goto route_failed;
        }
        continue;

route_failed:
        fprintf(stderr, "ERROR: physical RX routing failed: %s\n",
                strerror(-ret_val));
        request_muxer_stop();
        goto out;
    }
out:
    if (batches)
        free(batches);
    if (buf)
        free(buf);
    pthread_exit(NULL);
}

ut_static int write_data_to_uart(unsigned char pty_idx, const unsigned char *data, size_t len)
{
    const unsigned char esc = UART_PROTO_ESC_START;
    const unsigned char esc_esc = UART_PROTO_ESC_ESC;
    size_t data_index;
    ssize_t r;
    size_t encoded_buf_index = 0;
    size_t processed = 0;
    size_t chunk_size;
    static bool write_raw_pty_warning_shown = false;

    if (pty_idx >= pty_max_count) {
        fprintf(stderr, "ERROR: Invalid pty\n");
        return -EINVAL;
    }

    if (pty_idx == raw_pty_idx && !enable_write_raw_pty) {
        if (!write_raw_pty_warning_shown) {
            fprintf(stderr, "WARNING: Writing to RAW client is disabled. Use -w to enable.\n");
            write_raw_pty_warning_shown = true;
        }
        return 0;
    }

    /*
     * Process data in chunks to limit memory allocation.
     * Maximum encoded buffer size: MAX_WRITE_CHUNK_SIZE * 4 + 16
     * This prevents excessive memory usage for large writes.
     */
    const size_t max_encoded_buf_len = MAX_WRITE_CHUNK_SIZE * 4 + 16;
    unsigned char *encoded_buf = malloc(max_encoded_buf_len);
    if (!encoded_buf) {
        fprintf(stderr, "ERROR: Failed to allocate %zu bytes.\n", max_encoded_buf_len);
        return -ENOMEM;
    }

#define WRITE_BYTE_TO_ENCODED_BUF(x)                                     \
    do {                                                                 \
        if (encoded_buf_index >= max_encoded_buf_len)                    \
        {                                                                \
            fprintf(stderr,                                              \
                "ERROR: Buffer not large enough. Buf size: %zu. Index: %zu\n",  \
                max_encoded_buf_len, encoded_buf_index);                 \
            free(encoded_buf);                                           \
            return -ENOMEM;                                              \
        }                                                                \
        encoded_buf[encoded_buf_index++] = (x);                          \
    } while (0)

    pthread_mutex_lock(&tty_data.write_lock);

    /* A thread may have waited behind another VSER writer while systemd sent
     * SIGTERM. Do not begin a new physical-UART transaction after shutdown
     * has been requested. */
    if (!is_tcu_muxer_started()) {
        r = -ECANCELED;
        goto fail;
    }

    while (processed < len) {
        encoded_buf_index = 0;
        chunk_size = (len - processed > MAX_WRITE_CHUNK_SIZE) ?
                     MAX_WRITE_CHUNK_SIZE : (len - processed);

        /* Add tag header only for the first chunk of non-raw PTY */
        if (processed == 0 && pty_idx != raw_pty_idx) {
            WRITE_BYTE_TO_ENCODED_BUF(esc);
            WRITE_BYTE_TO_ENCODED_BUF(tags[pty_idx].value);
        }

        for (data_index = 0; data_index < chunk_size; data_index++) {
            // send data
            WRITE_BYTE_TO_ENCODED_BUF(data[processed + data_index]);
            if (data[processed + data_index] == esc && pty_idx != raw_pty_idx) {
                WRITE_BYTE_TO_ENCODED_BUF(esc_esc);
            }
        }

        r = enqueue_physical_tx(encoded_buf, encoded_buf_index);
        if (r < 0) {
            fprintf(stderr, "ERROR: Failed to queue physical UART buffer. Error: %zd\n", r);
            goto fail;
        }

        processed += chunk_size;
    }

    pthread_mutex_unlock(&tty_data.write_lock);
    free(encoded_buf);

#undef WRITE_BYTE_TO_ENCODED_BUF

    return 0;

fail:
    pthread_mutex_unlock(&tty_data.write_lock);
    if (encoded_buf)
        free(encoded_buf);

    return r;
}

void* pty_input_handler(void* arg)
{
    struct thread_data *t = arg;
    unsigned char pty_idx = t->id;

    unsigned char *buf = NULL;
    struct pollfd pfd;

    ssize_t len;
    int ret;

    buf = malloc(temporary_buffer_size);
    if (!buf) {
        fprintf(stderr,
            "ERROR: Failed to allocate temporary buffer with size %zu bytes\n",
            temporary_buffer_size);
        pthread_exit(NULL);
    }

    while (is_tcu_muxer_started()) {
        pfd.fd = t->fd;
        pfd.events = POLLIN;
        pfd.revents = 0;

        ret = poll(&pfd, 1, 100);
        if (should_retry_io(ret))
            continue;
        if (ret == 0)
            continue;
        assert(ret == 1);
        assert(!(pfd.revents & POLLNVAL));

        /* Guest pseudo-terminals continuously report
         * POLLIN when disconnected, so we spin
         * slowly to wait for valid data. */
        if (!(pfd.revents & POLLIN)) {
            sleep(1);
            continue;
        }

        /* Read the data from pts */
        len = read(t->fd, buf, temporary_buffer_size);
        if (len > 0) {
            /* This is the first muxer-owned TX boundary. Comparing this
             * per-VSER count with the controller source ledger distinguishes
             * SSH/obmc-console/PTY loss from physical-UART loss. */
            __atomic_fetch_add(&t->input_bytes, (unsigned long long)len,
                               __ATOMIC_RELAXED);
            __atomic_fetch_add(&t->input_reads, 1, __ATOMIC_RELAXED);
            /* Send data */
            ret = write_data_to_uart(pty_idx, buf, (size_t)len);
            if (ret < 0)
                fprintf(stderr, "Failed to write to uart\n");
        }
    }

    if (buf)
        free(buf);

    return 0;
}

speed_t get_baudrate(int baudrate)
{
    speed_t speed = -1;
    switch(baudrate) {
        case 0:
            speed = B0;
            break;
        case 50:
            speed = B50;
            break;
        case 75:
            speed = B75;
            break;
        case 110:
            speed = B110;
            break;
        case 134:
            speed = B134;
            break;
        case 150:
            speed = B150;
            break;
        case 200:
            speed = B200;
            break;
        case 300:
            speed = B300;
            break;
        case 600:
            speed = B600;
            break;
        case 1200:
            speed = B1200;
            break;
        case 1800:
            speed = B1800;
            break;
        case 2400:
            speed = B2400;
            break;
        case 4800:
            speed = B4800;
            break;
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        case 230400:
            speed = B230400;
            break;
        default:
            break;
    }
    return speed;
}

int invalid_baudrate(int baudrate)
{
    if((int)get_baudrate(baudrate) == -1)
        return 1;
    return 0;
}

int create_thread_with_stack(pthread_t *thread, void *(*start_routine)(void *), void *arg)
{
    pthread_attr_t attr;
    int ret;

    ret = pthread_attr_init(&attr);
    if (ret != 0) {
        fprintf(stderr, "ERROR: pthread_attr_init failed: %s\n", strerror(ret));
        return ret;
    }

    ret = pthread_attr_setstacksize(&attr, THREAD_STACK_SIZE);
    if (ret != 0) {
        fprintf(stderr, "ERROR: pthread_attr_setstacksize failed: %s\n", strerror(ret));
        pthread_attr_destroy(&attr);
        return ret;
    }

    ret = pthread_create(thread, &attr, start_routine, arg);
    if (ret != 0) {
        fprintf(stderr, "ERROR: pthread_create failed: %s\n", strerror(ret));
        pthread_attr_destroy(&attr);
        return ret;
    }

    pthread_attr_destroy(&attr);
    return 0;
}

void print_usage(char *argv[])
{
    fprintf(stderr, "Usage: %s [OPTION]...\n", argv[0]);
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "\t -h       : "
            "Print this help screen\n");
    fprintf(stderr, "\t -i       : "
            "Enable the patch for line ending\n");
    fprintf(stderr, "\t -u       : "
            "Use separate uart_muxer tool for Guest console\n");
    fprintf(stderr, "\t -d <dev> : "
            "Select UART device <dev> to communicate with. Default: %s\n", DEFAULT_TTY_DEVICE);
    fprintf(stderr, "\t -r <rate>: "
            "Select UART i/o speed <rate>. Default: %d\n", DEFAULT_BAUDRATE);
    fprintf(stderr, "\t -s <path>: "
            "Save the output to directory <path>\n");
    fprintf(stderr, "\t -t       : "
            "Prefix the output with a timestamp\n");
    fprintf(stderr, "\t -p <int> : "
            "Set the timeout for output polling ready status. Default: %d\n", DEFAULT_POLL_OUTPUT_TIMEOUT);
    fprintf(stderr, "\t -l <path>: "
            "Save the raw output with tags to log file <path>\n");
    fprintf(stderr, "\t -R <path>: "
            "Capture versioned physical UART read records asynchronously\n");
    fprintf(stderr, "\t -z <size>: "
            "Set max log file size before rotation. Supports k, M, G suffixes\n");
    fprintf(stderr, "\t -w       : "
            "Enable writing to RAW client\n");
    fprintf(stderr, "\nSend SIGUSR1 to print a read-only per-VSER output statistics snapshot.\n");
}

int main(int argc, char *argv[])
{
    char *raw_log_file_path = NULL;
    size_t max_log_size = 0;
    pthread_t tty_thread;
    pthread_t *pty_input_threads = NULL;
    pthread_t *pty_output_threads = NULL;
    struct termios options;
    struct sigaction stop_action;
    struct sigaction stats_action;
    size_t pty_input_threads_started = 0;
    size_t pty_output_threads_started = 0;
    bool tty_thread_started = false;
    pthread_t physical_tx_thread;
    bool physical_tx_thread_started = false;
    bool physical_tx_queue_initialized = false;
    bool physical_tty_lifecycle_initialized = false;
    bool tty_write_lock_initialized = false;
    int status = -1;
    int result;
    int opt;
    int i;
    size_t len;
    struct thread_data *pty;

    __atomic_store_n(&tcu_muxer_started, true, __ATOMIC_RELEASE);
    __atomic_store_n(&output_stats_requested, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_input_bytes, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tx_requested_bytes, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tx_accepted_bytes, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tx_write_calls, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tx_short_writes, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tx_eintr_retries, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tx_eagain_retries, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tx_poll_timeouts, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tx_zero_writes, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&tx_fatal_errors, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_invalid_tags, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_quarantined_bytes, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_resynchronizations, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_last_invalid_offset, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_last_invalid_realtime_ns, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&parser_last_invalid_monotonic_ns, 0, __ATOMIC_RELEASE);
    native_raw_capture_path = NULL;
    native_raw_capture_enabled = false;

    while ((opt = getopt(argc, argv, ":d:r:s:l:R:z:p:hitw")) != -1) {
        switch (opt)
        {
            case 'd':
                tty_device = optarg;
                break;
            case 'r':
                int_baudrate = atoi(optarg);
                break;
            case 's':
                len = strlen(optarg) + 1;
                if (len > sizeof(path)) {
                    fprintf(stderr, "-s argument length exceeds buffer size\n");
                    return -1;
                }
                strncpy(path, optarg, sizeof(path) - 1);
                save_output_path = path;
                if (save_output_path[strlen(save_output_path) - 1] == '/')
                    save_output_path[strlen(save_output_path) - 1] = '\0';
                break;
            case 'l':
                raw_log_file_path = optarg;
                break;
            case 'R':
                native_raw_capture_path = optarg;
                break;
            case 'z':
                if (parse_log_size(optarg, &max_log_size)) {
                    fprintf(stderr, "ERROR: invalid log size: %s\n", optarg);
                    return -1;
                }
                break;
            case 'h':
                print_usage(argv);
                return 0;
            case 'i':
                disable_patch = 0;
                break;
            case 't':
                log_timestamp_enabled = true;
                break;
            case 'w':
                enable_write_raw_pty = true;
                break;
            case 'p':
                poll_output_timeout = atoi(optarg);
                break;
            case ':':
                fprintf(stderr, "Option `-%c` requires an argument.\n", optopt);
                return -1;
            case '?':
                if (isprint(optopt))
                    fprintf(stderr, "Unknown option `-%c`.\n", optopt);
                else
                    fprintf(stderr, "Unknown option ``\\x%x`.\n", optopt);
                return -1;
            default:
                abort();
        }
    }

    tags = chip_tags;
    num_proc = sizeof(chip_tags) / sizeof(chip_tags[0]);
    default_tag_idx = num_proc - 1; // last tag is the default tag

    pty_max_count = num_proc + 1;
    raw_pty_idx = pty_max_count - 1; // last pty is the raw pty

    muxer_log_init(&tty_data.log);
    tty_data.fd = -1;

    pty_input_threads = calloc((size_t)pty_max_count, sizeof(*pty_input_threads));
    pty_output_threads = calloc((size_t)pty_max_count, sizeof(*pty_output_threads));
    pty_data = calloc((size_t)pty_max_count, sizeof(*pty_data));
    if (!pty_input_threads || !pty_output_threads || !pty_data) {
        fprintf(stderr, "ERROR: failed to allocate PTY state.\n");
        goto cleanup;
    }
    for_each_pty(i, pty) {
        pty->fd = -1;
        muxer_log_init(&pty->log);
    }

    result = init_physical_tty_lifecycle();
    if (result != 0) {
        fprintf(stderr, "ERROR: failed to initialize physical TTY lifecycle: %s\n",
                strerror(-result));
        goto cleanup;
    }
    physical_tty_lifecycle_initialized = true;
    result = open_tty_device();
    if (result < 0) {
        fprintf(stderr, "ERROR: failed to open in read mode %s\n", tty_device);
        goto cleanup;
    }
    pthread_mutex_lock(&physical_tty.lock);
    tty_data.fd = result;
    physical_tty.generation = 1;
    pthread_mutex_unlock(&physical_tty.lock);

    // bounds for -s arg
    if (save_output_path) {
        // check if save directory path exists
        DIR* dir = opendir(save_output_path);
        if (!dir) {
            fprintf(stderr, "ERROR: failed to open directory %s\n", save_output_path);
            goto cleanup;
        }
        closedir(dir);
    }

    if (raw_log_file_path) {
        if (muxer_log_open(&tty_data.log, raw_log_file_path, max_log_size)) {
            fprintf(stderr, "ERROR: raw log file open failed!\n");
            goto cleanup;
        }
    }
#ifdef UNIT_TEST
    // Exit main early if only testing argument parsing.
    if(ut_main_args_done()) {
        status = 0;
        goto cleanup;
    }
#endif

    tty_data.id = -1;
    result = pthread_mutex_init(&tty_data.write_lock, NULL);
    if (result != 0) {
        fprintf(stderr, "ERROR: failed to initialize UART write lock: %s\n",
                strerror(result));
        goto cleanup;
    }
    tty_write_lock_initialized = true;
    result = init_physical_tx_queue(PHYSICAL_TX_QUEUE_SIZE);
    if (result != 0) {
        fprintf(stderr, "ERROR: failed to initialize physical TX queue: %s\n",
                strerror(-result));
        goto cleanup;
    }
    physical_tx_queue_initialized = true;

    if (native_raw_capture_path) {
        result = raw_capture_start(native_raw_capture_path,
                                   RAW_CAPTURE_DEFAULT_FILE_SIZE,
                                   RAW_CAPTURE_DEFAULT_QUEUE_RECORDS);
        if (result != 0) {
            fprintf(stderr, "ERROR: failed to start native raw capture %s: %s\n",
                    native_raw_capture_path, strerror(-result));
            goto cleanup;
        }
        native_raw_capture_enabled = true;
    }

    // create pseudo-terminals and thread locks
    for_each_pty(i, pty) {
        char *name = NULL;
        int ret;
        int fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd < 0) {
            fprintf(stderr, "ERROR: posix_openpt failed for guest %d\n", i);
            goto cleanup;
        }
        pty->fd = fd;

        if (grantpt(fd)) {
            fprintf(stderr, "ERROR: grantpt failed for guest %d\n", i);
            goto cleanup;
        }

        if (unlockpt(fd)) {
            fprintf(stderr, "ERROR: unlockpt failed for guest %d\n", i);
            goto cleanup;
        }

        if (i != raw_pty_idx) {
            name = tags[i].name;
        } else if (i == raw_pty_idx) {
            name = RAW_PTY;
        }
        assert(name != NULL);

        if (save_output_path) {
            char savefile[1024];
            // generate save file name and path
            ret = snprintf(savefile, sizeof(savefile), "%s/%s.txt",
                           save_output_path, name);
            if (ret < 0 || (size_t)ret >= sizeof(savefile)) {
                fprintf(stderr, "ERROR: logfile path too long for %s\n", name);
                goto cleanup;
            }
            if (muxer_log_open(&pty->log, savefile, max_log_size))
                fprintf(stderr, "ERROR: logfile: %s open failed!\n", savefile);
        }
        ptsname_r(fd, pty->device_name,
                  sizeof(pty->device_name));
        pty->id = i;
        result = pthread_mutex_init(&pty->write_lock, NULL);
        if (result != 0) {
            fprintf(stderr, "ERROR: failed to initialize PTY %d write lock: %s\n",
                    i, strerror(result));
            goto cleanup;
        }
        pty->write_lock_initialized = true;
        result = init_pty_output_queue(pty, PTY_OUTPUT_QUEUE_SIZE);
        if (result != 0) {
            fprintf(stderr, "ERROR: failed to initialize PTY %d output queue: %s\n",
                    i, strerror(-result));
            goto cleanup;
        }
        {
            int slave = open(pty->device_name, O_RDWR);
            if (slave < 0) {
                fprintf(stderr, "ERROR: failed to open %s\n", pty->device_name);
                goto cleanup;
            }
            tcgetattr(slave, &options);
            cfmakeraw (&options);
            tcsetattr (slave, TCSANOW, &options);
            close(slave);
        }
        fprintf(stdout, "%s\t%s\n", pty->device_name, name);
    }

    fflush(stdout);

    memset(&stop_action, 0, sizeof(stop_action));
    stop_action.sa_handler = handle_stop_request;
    sigemptyset(&stop_action.sa_mask);
    if (sigaction(SIGINT, &stop_action, NULL) != 0 ||
        sigaction(SIGTERM, &stop_action, NULL) != 0) {
        fprintf(stderr, "ERROR: failed to install stop signal handlers: %s\n",
                strerror(errno));
        goto cleanup;
    }
    memset(&stats_action, 0, sizeof(stats_action));
    stats_action.sa_handler = handle_output_stats_request;
    sigemptyset(&stats_action.sa_mask);
    if (sigaction(SIGUSR1, &stats_action, NULL) != 0) {
        fprintf(stderr, "ERROR: failed to install statistics signal handler: %s\n",
                strerror(errno));
        goto cleanup;
    }

    /* Output consumers exist before the physical reader can enqueue bytes. */
    for_each_pty(i, pty) {
        if (create_thread_with_stack(&pty_output_threads[i], pty_output_handler, pty)) {
            fprintf(stderr, "ERROR: failed to spawn PTY output thread %d\n", i);
            goto shutdown;
        }
        pty_output_threads_started++;
    }
    if (create_thread_with_stack(&physical_tx_thread, physical_tx_handler, NULL)) {
        fprintf(stderr, "ERROR: failed to spawn physical TX worker\n");
        goto shutdown;
    }
    physical_tx_thread_started = true;
    if (create_thread_with_stack(&tty_thread, tty_input_handler, &tty_data)) {
        fprintf(stderr, "ERROR: failed to spawn physical UART thread\n");
        goto shutdown;
    }
    tty_thread_started = true;
    for_each_pty(i, pty) {
        if (create_thread_with_stack(&pty_input_threads[i], pty_input_handler, pty)) {
            fprintf(stderr, "ERROR: failed to spawn PTY input thread %d\n", i);
            goto shutdown;
        }
        pty_input_threads_started++;
    }

#ifdef UNIT_TEST
    //inform test program that tcu_muxer is now ready to do i/o
    ut_main_threads_started();
#endif

    pthread_join(tty_thread, NULL);
    tty_thread_started = false;
    status = 0;

shutdown:
    request_muxer_stop();
    for_each_pty(i, pty) {
        if (pty->output_queue)
            pthread_cond_broadcast(&pty->output_ready);
    }
    if (physical_tx_queue_initialized)
        pthread_cond_broadcast(&physical_tx.ready);
    if (physical_tx_queue_initialized)
        pthread_cond_broadcast(&physical_tx.space_available);
    if (physical_tty_lifecycle_initialized) {
        pthread_mutex_lock(&physical_tty.lock);
        pthread_cond_broadcast(&physical_tty.changed);
        pthread_mutex_unlock(&physical_tty.lock);
    }
    if (tty_thread_started)
        pthread_join(tty_thread, NULL);
    for (i = 0; i < (int)pty_input_threads_started; i++)
        pthread_join(pty_input_threads[i], NULL);
    if (physical_tx_thread_started)
        pthread_join(physical_tx_thread, NULL);
    for (i = 0; i < (int)pty_output_threads_started; i++)
        pthread_join(pty_output_threads[i], NULL);
    if (native_raw_capture_enabled) {
        raw_capture_stop();
    }
    report_output_stats(status == 0 ? "shutdown" : "error");
    native_raw_capture_enabled = false;

cleanup:
    if (native_raw_capture_enabled) {
        raw_capture_stop();
        native_raw_capture_enabled = false;
    }
    if (physical_tty_lifecycle_initialized) {
        pthread_mutex_lock(&physical_tty.lock);
        if (tty_data.fd >= 0) {
            close(tty_data.fd);
            tty_data.fd = -1;
        }
        pthread_mutex_unlock(&physical_tty.lock);
    } else if (tty_data.fd >= 0) {
        close(tty_data.fd);
        tty_data.fd = -1;
    }
    muxer_log_close(&tty_data.log);
    if (pty_data) {
        for_each_pty(i, pty) {
            if (pty->fd >= 0)
                close(pty->fd);
            muxer_log_close(&pty->log);
            destroy_pty_output_queue(pty);
            if (pty->write_lock_initialized)
                pthread_mutex_destroy(&pty->write_lock);
        }
    }
    if (tty_write_lock_initialized)
        pthread_mutex_destroy(&tty_data.write_lock);
    if (physical_tx_queue_initialized)
        destroy_physical_tx_queue();
    if (physical_tty_lifecycle_initialized)
        destroy_physical_tty_lifecycle();
    uucp_unlock_tty_device();
    free(pty_input_threads);
    free(pty_output_threads);
    free(pty_data);
    pty_data = NULL;
    return status;
}
