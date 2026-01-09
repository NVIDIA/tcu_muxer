/*
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: Copyright (c) 2013-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <sys/ioctl.h>
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
int open_tty_device(void);
int reopen_tty_device(int old_fd);
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
#define DEFAULT_POLL_OUTPUT_TIMEOUT 300
#define DEFAULT_TTY_DEVICE "/dev/ttyUSB3"
#define UUCP_DIR "/var/lock"
#define RAW_PTY "RAW"
#define MAX_WRITE_CHUNK_SIZE 4096  // Maximum bytes to process at once (16KB encoded max)
#define THREAD_STACK_SIZE (128 * 1024)  // 128KB stack per thread (sufficient for this application)

ut_static bool tcu_muxer_started = true; // Needed to exit endless while loops during testing
ut_static const char* tty_device = DEFAULT_TTY_DEVICE;
ut_static int int_baudrate = DEFAULT_BAUDRATE;
ut_static int pty_max_count;
ut_static int raw_pty_idx;
ut_static bool uucp_locked = false;
ut_static char filelock[MAX_PATH];
static int poll_output_timeout = DEFAULT_POLL_OUTPUT_TIMEOUT;
static char path[MAX_PATH];

struct tag {
    char *name;
    unsigned char value;
};

const struct tag chip_tags[] = {
    {
        .name = "PSCFW",
        .value = 0xe1
    },
    {
        .name = "BPMP",
        .value = 0xe2
    },
    {
        .name = "OOBHUBFW",
        .value = 0xe3
    },
    {
        .name = "SatMCFW",
        .value = 0xe4
    },
    {
        .name = "RASFW",
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
        .name = "MSEQFW",
        .value = 0xea
    },
    {
        .name = "PCOREFW",
        .value = 0xeb
    },
    {
        .name = "C2CFW",
        .value = 0xec
    },
    {
        .name = "DBG2",
        .value = 0xed
    },
    {
        .name = "NCORE",
        .value = 0xef
    },
    {
        .name = "NPXIR",
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

struct thread_data
{
    int fd;
    int log_fd;
    unsigned int id;
    pthread_mutex_t write_lock;
    char device_name[128];
    char last_ch;
};

struct thread_data tty_data;
struct thread_data *pty_data;

#define for_each_tags(tag_idx, tag) \
    for (tag_idx = 0, tag = &tags[tag_idx]; tag_idx < num_proc; tag_idx++, tag++)

#define for_each_pty(i, pty) \
    for (i = 0, pty = &pty_data[i]; i < pty_max_count; i++, pty++)


bool should_retry_io(ssize_t ret)
{
    return (ret == -1) &&
        (
        errno == EINTR ||
        errno == EAGAIN ||
        errno == EWOULDBLOCK
        );
}

int putbuf_or_exit(int fd, const unsigned char *buf, size_t len)
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
    assert(ret == 1);
    assert(!(pfd.revents & POLLNVAL));

    do {
        ret = write(fd, buf, len);
    } while (should_retry_io(ret));

    if (ret < 0) {
        fprintf(stderr, "ERROR: failed to write - %s\n", strerror(errno));
        return ret;
    }

    if ((size_t)ret != len) {
        fprintf(stderr, "ERROR: failed to write bytes. Only %zd written but %zu requested\n",
            ret, len);
        return ret;
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
    assert(ret == 1);
    assert(!(pfd.revents & POLLNVAL));

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
    /* 'tsfmt' stores 28 chars: "[YYYY-mm-dd HH:MM:SS.%06u] \0" */
    /* resize 'fmt' accordingly when changing the timestamp format 'tsfmt' */
    const char tsfmt[] = "[%Y-%m-%d %H:%M:%S.%%06u] ";
    char fmt[28];

    struct timeval tv;
    struct tm *tm = NULL;
    int len = 0;

    if (buf && maxsize && !gettimeofday(&tv, NULL) && (tm = localtime(&tv.tv_sec))) {
        if (strftime(fmt, sizeof fmt, tsfmt, tm)) {
            /* fill in microseconds */
            len = snprintf(buf, maxsize, fmt, tv.tv_usec);
            if (0 < len && len < (int)maxsize ) {
                /* len should less than maxsize since at least one byte */
                /* must be reserved for the terminating null character */
                return (size_t)len;
            }
        }
    }
    return 0;
}

int flush_stream(int fd, int pty_idx, unsigned char ch)
{
    char timestamp[30];
    size_t len_ts = 0;
    /* 'timestamp' stores 30 chars: "[YYYY-mm-dd HH:MM:SS.ssssss] \0" */
    /* resize it accordingly when changing the timestamp format */

    if ((pty_idx < pty_max_count) && pty_data[pty_idx].log_fd >= 0) {
        if (log_timestamp_enabled) {
            if ('\n' == pty_data[pty_idx].last_ch || '\0' == pty_data[pty_idx].last_ch) {
                len_ts = get_timestamp(timestamp, sizeof timestamp);
                if (0 < len_ts && len_ts < sizeof timestamp)
                    if( write(pty_data[pty_idx].log_fd, timestamp, len_ts) < 0){
                        return -2;
                    }
            }
            pty_data[pty_idx].last_ch = ch;
        }

        if (write(pty_data[pty_idx].log_fd, &ch, 1) < 0){
            return -3;
        }
    }

    // write to pty path
    if (write(fd, &ch, 1) < 0)
        return -1;

    return 0;
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

ut_static void uucp_unlock_tty_device(void)
{
    if (filelock[0] && uucp_locked) {
        unlink(filelock);
        uucp_locked = false;
    }
}

static void handle_sigint(int sig) {
    (void)sig;
    uucp_unlock_tty_device();
    exit(0);
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

    signal(SIGINT, handle_sigint);
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
    putch_or_exit(fd, chr);
    chr = UART_PROTO_ESC_START;
    putch_or_exit(fd, chr);
    chr = UART_PROTO_ESC_RESET;
    putch_or_exit(fd, chr);

    return fd;

err:
    fprintf(stderr, "%s: failed\n", __func__);
    if (fd >= 0)
        close(fd);
    return -1;
}

ut_static int reopen_tty_device(int old_fd)
{
    if (old_fd >= 0)
        close(old_fd);
    return open_tty_device();
}

void* tty_input_handler(void *arg)
{
    (void)arg;
    unsigned char *buf = NULL;
    struct pollfd pfd;

    unsigned char ch;
    bool tag_match = false;
    bool in_escape = false;
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

    buf = malloc(temporary_buffer_size);
    if (!buf) {
        fprintf(stderr,
            "ERROR: Failed to allocate temporary buffer with size %zu bytes\n",
            temporary_buffer_size);
        goto out;
    }

    while (tcu_muxer_started) {
        pfd.fd = tty_data.fd;
        pfd.events = POLLIN;
        pfd.revents = 0;

        ret_val = poll(&pfd, 1, -1);

        if (should_retry_io(ret_val))
            continue;
        assert(ret_val == 1);
        assert(!(pfd.revents & POLLNVAL));

        if (pfd.revents & (POLLHUP|POLLERR)) {
#ifdef UNIT_TEST
            // This additional check is needed to allow the thread to exit gracefully during unit testing.
            // Poll() returns POLLUP when the mock device closes at the end of a test, so we check here if the test
            // is over to avoid failing the test.
            if(!tcu_muxer_started)
                goto out;
#endif
            tty_data.fd = reopen_tty_device(tty_data.fd);
            if (tty_data.fd >= 0) {
                continue;
            } else {
                fprintf(stderr, "ERROR: tty_input_handler: hangup\n");
                exit(-1);
            }
        }

        len = read(tty_data.fd, buf, temporary_buffer_size);
        if (len < 0) {
            fprintf(stderr, "ERROR: failed to read\n");
            goto out;
        }
        if (tty_data.log_fd >= 0){
            if (write(tty_data.log_fd, buf, len) < 0){
                fprintf(stderr, "ERROR: failed to write\n");
                goto out;
            }
        }

        for (index = 0; index < len; index++) {
            ch = buf[index];

            patch2flush_stream(pty_data[raw_pty_idx].fd, raw_pty_idx,
                ch, &seen_n[raw_pty_idx], &seen_r[raw_pty_idx]);

            if (in_escape) {
                in_escape = false;
                // Handle UTC control characters
                switch (ch) {
                    case UART_PROTO_ESC_ESC:
                        ch = UART_PROTO_ESC_START;
                        goto not_escape_ch;

                    case UART_PROTO_ESC_RESET:
                        cur_rx_guest = default_tag_idx;
                        break;

                    default:
                        // Check for tag matches
                        tag_match = false;
                        for_each_tags(tag_idx, tag) {
                            if (ch == tag->value) {
                                cur_rx_guest = tag_idx;
                                tag_match = true;
                                break;
                            }
                        }
                        if (!tag_match) {
                            // this shouldn't happen.
                            // Commenting out the log for now...
                            // fprintf(stderr, "Invalid control character 0x%x, ignoring...\n", ch);
                        }
                        break;
                }
            } else {
                if (ch == UART_PROTO_ESC_START) {
                    in_escape = true;
                } else {
not_escape_ch:
                        assert(cur_rx_guest < (int) (pty_max_count));
                        ret_val = patch2flush_stream(pty_data[cur_rx_guest].fd, cur_rx_guest,
                                ch, &seen_n[cur_rx_guest], &seen_r[cur_rx_guest]);
                }
            }
        }
    }
out:
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

    if (pty_idx >= pty_max_count) {
        fprintf(stderr, "ERROR: Invalid pty\n");
        return -EINVAL;
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

        r = putbuf_or_exit(tty_data.fd, encoded_buf, encoded_buf_index);
        if (r < 0) {
            fprintf(stderr, "ERROR: Failed to write buffer. Error: %zd\n", r);
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

    while (tcu_muxer_started) {
        pfd.fd = t->fd;
        pfd.events = POLLIN;
        pfd.revents = 0;

        ret = poll(&pfd, 1, -1);
        if (should_retry_io(ret))
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
}

int main(int argc, char *argv[])
{
    char *raw_log_file_path = NULL;

    pthread_t tty_thread;
    pthread_t *pty_thread;

    struct termios options;
    int opt;
    int i;
    size_t len;
    struct thread_data *pty;

    while ((opt = getopt(argc, argv, ":d:r:s:l:p:hit")) != -1) {
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
            case 'h':
                print_usage(argv);
                return 0;
            case 'i':
                disable_patch = 0;
                break;
            case 't':
                log_timestamp_enabled = true;
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

    pty_thread = malloc(sizeof(pthread_t) * pty_max_count);
    pty_data = malloc(sizeof(struct thread_data) * pty_max_count);

    tty_data.fd = open_tty_device();
    if (tty_data.fd < 0) {
        fprintf(stderr, "ERROR: failed to open in read mode %s\n", tty_device);
        goto err;
    }

    // bounds for -s arg
    if (save_output_path) {
        // check if save directory path exists
        DIR* dir = opendir(save_output_path);
        if (!dir) {
            fprintf(stderr, "ERROR: failed to open directory %s\n", save_output_path);
            goto err;
        }
        closedir(dir);
    }

    if (raw_log_file_path) {
        tty_data.log_fd = open(raw_log_file_path, O_CREAT|O_APPEND|O_WRONLY,
                               S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR);
        if (tty_data.log_fd < 0)
            fprintf(stderr, "ERROR: raw log file open failed!\n");
    } else {
        tty_data.log_fd = -1;
    }

#ifdef UNIT_TEST
    // Exit main early if only testing argument parsing.
    if(ut_main_args_done()) {
        uucp_unlock_tty_device();
        return 0;
    }
#endif

    tty_data.id = -1;
    pthread_mutex_init(&tty_data.write_lock, NULL);

    // create pseudo-terminals and thread locks
    for_each_pty(i, pty) {
        char *name = NULL;
        int fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (fd < 0) {
            fprintf(stderr, "ERROR: posix_openpt failed for guest %d\n", i);
            goto err;
        }

        if (grantpt(fd)) {
            fprintf(stderr, "ERROR: grantpt failed for guest %d\n", i);
            goto err;
        }

        if (unlockpt(fd)) {
            fprintf(stderr, "ERROR: unlockpt failed for guest %d\n", i);
            goto err;
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
            snprintf(savefile, sizeof(savefile), "%s/%s", save_output_path, name);
            strncat(savefile, ".txt", sizeof(savefile) - strlen(savefile) - 1);
            pty->log_fd = open(savefile, O_APPEND|O_CREAT|O_WRONLY,
                                      S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR);
            if (pty->log_fd < 0)
                fprintf(stderr, "ERROR: logfile: %s open failed!\n", savefile);
        } else {
            pty->log_fd = -1;
        }
        ptsname_r(fd, pty->device_name,
                  sizeof(pty->device_name));
        pty->fd = fd;
        pty->id = i;
        pthread_mutex_init(&pty->write_lock, NULL);
        {
            int slave = open(pty->device_name, O_RDWR);
            if (slave < 0) {
                fprintf(stderr, "ERROR: failed to open %s\n", pty->device_name);
                exit(-1);
            }
            tcgetattr(slave, &options);
            cfmakeraw (&options);
            tcsetattr (slave, TCSANOW, &options);
            close(slave);
        }
        fprintf(stdout, "%s\t%s\n", pty->device_name, name);
    }

    fflush(stdout);

    if (create_thread_with_stack(&tty_thread, tty_input_handler, &tty_data)) {
        fprintf(stderr, "ERROR: failed to spawn tty thread\n");
        goto err;
    }

    for_each_pty(i, pty) {
        if (create_thread_with_stack(&pty_thread[i], pty_input_handler, pty)) {
            fprintf(stderr, "ERROR: failed to spawn pty thread %d\n", i);
            goto err;
        }
    }

#ifdef UNIT_TEST
    //inform test program that tcu_muxer is now ready to do i/o
    ut_main_threads_started();
#endif

    pthread_join(tty_thread, NULL);
    for_each_pty(i, pty) {
        pthread_join(pty_thread[i], NULL);
    }

    // flock will get released automatically when we close tty fd
    close(tty_data.fd);
    if (raw_log_file_path && tty_data.log_fd >= 0)
        close(tty_data.log_fd);
    for_each_pty(i, pty) {
        close(pty->fd);
        if (save_output_path && pty->log_fd >= 0)
            close(pty->log_fd);
    }
    uucp_unlock_tty_device();
    return 0;
err:
    uucp_unlock_tty_device();
    return -1;
}
