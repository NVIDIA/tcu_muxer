/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#define _GNU_SOURCE 1

#include "raw_capture.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

struct queued_read {
    uint64_t realtime_ns;
    uint64_t monotonic_ns;
    uint64_t sequence;
    uint32_t length;
    unsigned char payload[RAW_CAPTURE_MAX_PAYLOAD];
};

struct raw_capture_context {
    char path[PATH_MAX];
    int fd;
    size_t file_size_limit;
    size_t current_file_size;
    struct queued_read *queue;
    size_t queue_capacity;
    size_t queue_head;
    size_t queue_length;
    pthread_mutex_t lock;
    pthread_cond_t ready;
    pthread_t worker;
    bool initialized;
    bool running;
    struct raw_capture_stats stats;
    uint64_t next_sequence;
    uint64_t pending_gap_records;
    uint64_t reported_gap_records;
    uint64_t reported_gap_bytes;
#ifdef UNIT_TEST
    bool writer_paused;
#endif
};

static struct raw_capture_context capture = { .fd = -1 };
static struct raw_capture_stats last_stats;

static uint64_t nanoseconds(clockid_t clock)
{
    struct timespec value;

    if (clock_gettime(clock, &value) != 0)
        return 0;
    return (uint64_t)value.tv_sec * 1000000000ULL + (uint64_t)value.tv_nsec;
}

static uint32_t crc32_bytes(const unsigned char *data, size_t length)
{
    uint32_t crc = 0xffffffffU;
    size_t index;
    int bit;

    for (index = 0; index < length; index++) {
        crc ^= data[index];
        for (bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xedb88320U & (uint32_t)-(int)(crc & 1U));
    }
    return ~crc;
}

static int write_all(int fd, const void *data, size_t length)
{
    const unsigned char *cursor = data;
    size_t written = 0;

    while (written < length) {
        ssize_t result = write(fd, cursor + written, length - written);
        if (result > 0) {
            written += (size_t)result;
            continue;
        }
        if (result < 0 && errno == EINTR)
            continue;
        return result == 0 ? -EIO : -errno;
    }
    return 0;
}

static int rotate_if_needed(size_t record_size)
{
    char rotated[PATH_MAX];

    if (capture.current_file_size + record_size <= capture.file_size_limit)
        return 0;
    if (snprintf(rotated, sizeof(rotated), "%s.1", capture.path) >= (int)sizeof(rotated))
        return -ENAMETOOLONG;
    if (close(capture.fd) != 0)
        return -errno;
    capture.fd = -1;
    if (unlink(rotated) != 0 && errno != ENOENT)
        return -errno;
    if (rename(capture.path, rotated) != 0 && errno != ENOENT)
        return -errno;
    capture.fd = open(capture.path, O_CREAT | O_TRUNC | O_WRONLY, 0600);
    if (capture.fd < 0)
        return -errno;
    capture.current_file_size = 0;
    __atomic_add_fetch(&capture.stats.rotations, 1, __ATOMIC_RELAXED);
    return 0;
}

static int write_record(uint16_t type, const struct queued_read *read,
                        uint64_t dropped_records, uint64_t dropped_bytes)
{
    struct raw_capture_record_header header;
    size_t record_size;
    int status;

    memset(&header, 0, sizeof(header));
    header.magic = RAW_CAPTURE_RECORD_MAGIC;
    header.version = RAW_CAPTURE_RECORD_VERSION;
    header.header_size = sizeof(header);
    header.type = type;
    header.realtime_ns = read ? read->realtime_ns : nanoseconds(CLOCK_REALTIME);
    header.monotonic_ns = read ? read->monotonic_ns : nanoseconds(CLOCK_MONOTONIC);
    header.read_sequence = read ? read->sequence : 0;
    header.dropped_records = dropped_records;
    header.dropped_bytes = dropped_bytes;
    header.payload_length = read ? read->length : 0;
    header.payload_crc32 = read ? crc32_bytes(read->payload, read->length) : 0;
    record_size = sizeof(header) + header.payload_length;

    status = rotate_if_needed(record_size);
    if (status == 0)
        status = write_all(capture.fd, &header, sizeof(header));
    if (status == 0 && read)
        status = write_all(capture.fd, read->payload, read->length);
    if (status != 0) {
        __atomic_add_fetch(&capture.stats.io_errors, 1, __ATOMIC_RELAXED);
        return status;
    }
    capture.current_file_size += record_size;
    __atomic_add_fetch(&capture.stats.records_written, 1, __ATOMIC_RELAXED);
    __atomic_add_fetch(&capture.stats.bytes_written, record_size, __ATOMIC_RELAXED);
    if (type == RAW_CAPTURE_RECORD_GAP)
        __atomic_add_fetch(&capture.stats.gap_records, 1, __ATOMIC_RELAXED);
    return 0;
}

static void *capture_worker(void *unused)
{
    (void)unused;
    for (;;) {
        struct queued_read read;
        uint64_t gap_records;
        uint64_t gap_bytes;
        uint64_t gap_sequence;
        bool have_read = false;

        pthread_mutex_lock(&capture.lock);
        while (capture.running && capture.queue_length == 0 &&
               __atomic_load_n(&capture.pending_gap_records, __ATOMIC_ACQUIRE) == 0
#ifdef UNIT_TEST
               && !capture.writer_paused
#endif
        )
            pthread_cond_wait(&capture.ready, &capture.lock);
#ifdef UNIT_TEST
        while (capture.running && capture.writer_paused)
            pthread_cond_wait(&capture.ready, &capture.lock);
#endif
        (void)__atomic_exchange_n(&capture.pending_gap_records, 0,
                                  __ATOMIC_ACQ_REL);
        /* Dropped-record statistics are cumulative. The worker alone owns
         * the reported checkpoints, so a producer racing this snapshot is
         * accounted either in this GAP or the next one—never split across
         * unrelated first-sequence/record/byte atomics. */
        gap_records = __atomic_load_n(&capture.stats.dropped_records,
                                      __ATOMIC_ACQUIRE) -
                      capture.reported_gap_records;
        gap_bytes = __atomic_load_n(&capture.stats.dropped_bytes,
                                    __ATOMIC_ACQUIRE) -
                    capture.reported_gap_bytes;
        gap_sequence = __atomic_load_n(&capture.next_sequence, __ATOMIC_ACQUIRE);
        capture.reported_gap_records += gap_records;
        capture.reported_gap_bytes += gap_bytes;
        if (capture.queue_length != 0) {
            read = capture.queue[capture.queue_head];
            capture.queue_head = (capture.queue_head + 1) % capture.queue_capacity;
            capture.queue_length--;
            have_read = true;
        }
        if (!capture.running && !have_read && gap_records == 0) {
            pthread_mutex_unlock(&capture.lock);
            break;
        }
        pthread_mutex_unlock(&capture.lock);

        if (gap_records != 0) {
            read.realtime_ns = nanoseconds(CLOCK_REALTIME);
            read.monotonic_ns = nanoseconds(CLOCK_MONOTONIC);
            read.sequence = gap_sequence;
            read.length = 0;
            (void)write_record(RAW_CAPTURE_RECORD_GAP, &read, gap_records, gap_bytes);
        }
        if (have_read && write_record(RAW_CAPTURE_RECORD_DATA, &read, 0, 0) != 0)
            fprintf(stderr, "ERROR: asynchronous raw capture write failed.\n");
    }
    return NULL;
}

int raw_capture_start(const char *path, size_t file_size, size_t queue_records)
{
    char rotated[PATH_MAX];
    int error_code;

    if (!path || !path[0] || file_size < sizeof(struct raw_capture_record_header) ||
        queue_records == 0 || strlen(path) >= sizeof(capture.path))
        return -EINVAL;
    if (capture.initialized)
        return -EBUSY;
    memset(&capture, 0, sizeof(capture));
    memset(&last_stats, 0, sizeof(last_stats));
    capture.fd = -1;
    strcpy(capture.path, path);
    capture.file_size_limit = file_size;
    capture.queue_capacity = queue_records;
    capture.queue = calloc(queue_records, sizeof(*capture.queue));
    if (!capture.queue)
        return -ENOMEM;
    error_code = pthread_mutex_init(&capture.lock, NULL);
    if (error_code != 0)
        goto fail;
    error_code = pthread_cond_init(&capture.ready, NULL);
    if (error_code != 0) {
        pthread_mutex_destroy(&capture.lock);
        goto fail;
    }
    if (snprintf(rotated, sizeof(rotated), "%s.1", path) >= (int)sizeof(rotated)) {
        error_code = ENAMETOOLONG;
        goto fail_sync;
    }
    /* A service restart begins a new capture epoch. Preserve the previous
     * complete file as the single rotation, then start the new epoch empty. */
    if (unlink(rotated) != 0 && errno != ENOENT) {
        error_code = errno;
        goto fail_sync;
    }
    if (rename(path, rotated) != 0 && errno != ENOENT) {
        error_code = errno;
        goto fail_sync;
    }
    capture.fd = open(path, O_CREAT | O_TRUNC | O_WRONLY, 0600);
    if (capture.fd < 0) {
        error_code = errno;
        goto fail_sync;
    }
    capture.current_file_size = 0;
    capture.running = true;
    capture.initialized = true;
    error_code = pthread_create(&capture.worker, NULL, capture_worker, NULL);
    if (error_code != 0)
        goto fail_fd;
    return 0;

fail_fd:
    if (capture.fd >= 0)
        close(capture.fd);
fail_sync:
    pthread_cond_destroy(&capture.ready);
    pthread_mutex_destroy(&capture.lock);
fail:
    free(capture.queue);
    memset(&capture, 0, sizeof(capture));
    capture.fd = -1;
    return -error_code;
}

void raw_capture_record(const unsigned char *data, size_t length)
{
    struct queued_read *entry;
    uint64_t sequence;

    if (!capture.initialized || !data || length == 0)
        return;
    __atomic_add_fetch(&capture.stats.physical_reads, 1, __ATOMIC_RELAXED);
    __atomic_add_fetch(&capture.stats.physical_bytes, length, __ATOMIC_RELAXED);
    sequence = __atomic_add_fetch(&capture.next_sequence, 1, __ATOMIC_RELAXED);
    if (length > RAW_CAPTURE_MAX_PAYLOAD) {
        __atomic_add_fetch(&capture.stats.dropped_records, 1, __ATOMIC_RELAXED);
        __atomic_add_fetch(&capture.stats.dropped_bytes, length, __ATOMIC_RELAXED);
        __atomic_add_fetch(&capture.pending_gap_records, 1, __ATOMIC_RELAXED);
        return;
    }
    /* The worker releases this lock before every file write, rotation, fsync,
     * or error report. The physical reader can therefore wait only for a few
     * in-memory ring-accounting operations, never for storage. trylock was
     * intentionally removed: it converted harmless producer/consumer lock
     * overlap into false capture gaps even when 1,023 queue slots were free. */
    pthread_mutex_lock(&capture.lock);
    if (capture.queue_length == capture.queue_capacity) {
        __atomic_add_fetch(&capture.stats.dropped_records, 1, __ATOMIC_RELAXED);
        __atomic_add_fetch(&capture.stats.dropped_bytes, length, __ATOMIC_RELAXED);
        __atomic_add_fetch(&capture.pending_gap_records, 1, __ATOMIC_RELAXED);
        pthread_mutex_unlock(&capture.lock);
        return;
    }
    entry = &capture.queue[(capture.queue_head + capture.queue_length) % capture.queue_capacity];
    entry->realtime_ns = nanoseconds(CLOCK_REALTIME);
    entry->monotonic_ns = nanoseconds(CLOCK_MONOTONIC);
    entry->sequence = sequence;
    entry->length = (uint32_t)length;
    memcpy(entry->payload, data, length);
    capture.queue_length++;
    if (capture.queue_length >
        __atomic_load_n(&capture.stats.queue_high_water, __ATOMIC_RELAXED))
        __atomic_store_n(&capture.stats.queue_high_water, capture.queue_length,
                         __ATOMIC_RELAXED);
    pthread_cond_signal(&capture.ready);
    pthread_mutex_unlock(&capture.lock);
}

void raw_capture_get_stats(struct raw_capture_stats *stats)
{
    if (!stats)
        return;
    if (!capture.initialized) {
        memcpy(stats, &last_stats, sizeof(*stats));
        return;
    }
#define COPY_ATOMIC(field) \
    stats->field = __atomic_load_n(&capture.stats.field, __ATOMIC_RELAXED)
    COPY_ATOMIC(physical_reads);
    COPY_ATOMIC(physical_bytes);
    COPY_ATOMIC(records_written);
    COPY_ATOMIC(bytes_written);
    COPY_ATOMIC(dropped_records);
    COPY_ATOMIC(dropped_bytes);
    COPY_ATOMIC(gap_records);
    COPY_ATOMIC(queue_high_water);
    COPY_ATOMIC(rotations);
    COPY_ATOMIC(io_errors);
#undef COPY_ATOMIC
}

void raw_capture_stop(void)
{
    if (!capture.initialized)
        return;
    pthread_mutex_lock(&capture.lock);
    capture.running = false;
#ifdef UNIT_TEST
    capture.writer_paused = false;
#endif
    pthread_cond_broadcast(&capture.ready);
    pthread_mutex_unlock(&capture.lock);
    pthread_join(capture.worker, NULL);
    if (fsync(capture.fd) != 0) {
        __atomic_add_fetch(&capture.stats.io_errors, 1, __ATOMIC_RELAXED);
        fprintf(stderr, "ERROR: asynchronous raw capture fsync failed: %s\n",
                strerror(errno));
    }
    if (close(capture.fd) != 0) {
        __atomic_add_fetch(&capture.stats.io_errors, 1, __ATOMIC_RELAXED);
        fprintf(stderr, "ERROR: asynchronous raw capture close failed: %s\n",
                strerror(errno));
    }
    memcpy(&last_stats, &capture.stats, sizeof(last_stats));
    pthread_cond_destroy(&capture.ready);
    pthread_mutex_destroy(&capture.lock);
    free(capture.queue);
    memset(&capture, 0, sizeof(capture));
    capture.fd = -1;
}

#ifdef UNIT_TEST
void raw_capture_pause_writer_for_test(int paused)
{
    if (!capture.initialized)
        return;
    pthread_mutex_lock(&capture.lock);
    capture.writer_paused = paused != 0;
    pthread_cond_broadcast(&capture.ready);
    pthread_mutex_unlock(&capture.lock);
}
#endif
