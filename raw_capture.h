/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TCU_MUXER_RAW_CAPTURE_H
#define TCU_MUXER_RAW_CAPTURE_H

#include <stddef.h>
#include <stdint.h>

#define RAW_CAPTURE_DEFAULT_FILE_SIZE (16U * 1024U * 1024U)
#define RAW_CAPTURE_DEFAULT_QUEUE_RECORDS 1024U
#define RAW_CAPTURE_MAX_PAYLOAD 1024U
#define RAW_CAPTURE_RECORD_MAGIC 0x31524354U /* "TCR1" in little endian. */
#define RAW_CAPTURE_RECORD_VERSION 1U
#define RAW_CAPTURE_RECORD_DATA 1U
#define RAW_CAPTURE_RECORD_GAP 2U

/* The on-disk format is intentionally fixed-width and versioned so Jenkins
 * analyzers can reject incompatible records instead of guessing. */
struct __attribute__((packed)) raw_capture_record_header {
    uint32_t magic;
    uint16_t version;
    uint16_t header_size;
    uint16_t type;
    uint16_t reserved;
    uint64_t realtime_ns;
    uint64_t monotonic_ns;
    uint64_t read_sequence;
    uint64_t dropped_records;
    uint64_t dropped_bytes;
    uint32_t payload_length;
    uint32_t payload_crc32;
};

struct raw_capture_stats {
    uint64_t physical_reads;
    uint64_t physical_bytes;
    uint64_t records_written;
    uint64_t bytes_written;
    uint64_t dropped_records;
    uint64_t dropped_bytes;
    uint64_t gap_records;
    uint64_t queue_high_water;
    uint64_t rotations;
    uint64_t io_errors;
};

int raw_capture_start(const char *path, size_t file_size, size_t queue_records);
void raw_capture_record(const unsigned char *data, size_t length);
void raw_capture_get_stats(struct raw_capture_stats *stats);
void raw_capture_stop(void);

#ifdef UNIT_TEST
void raw_capture_pause_writer_for_test(int paused);
#endif
#endif
