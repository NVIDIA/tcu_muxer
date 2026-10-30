/* SPDX-License-Identifier: Apache-2.0 */
#ifndef TCU_RX_STALL_H
#define TCU_RX_STALL_H

#include <stdbool.h>
#include <stdint.h>

/* Owned exclusively by the physical RX thread. */
struct rx_stall {
    int usb_fd;
    unsigned int interface;
    uint64_t last_rx_ms;
    uint64_t next_probe_ms;
    uint64_t suspect_since_ms;
    uint64_t retry_after_ms;
    unsigned int attempts;
    bool awaiting_rx;
    bool probe_failed;
    bool discovery_retry;
};

void rx_stall_init(struct rx_stall *watch, int tty_fd);
void rx_stall_refresh(struct rx_stall *watch, int tty_fd);
void rx_stall_close(struct rx_stall *watch);
void rx_stall_progress(struct rx_stall *watch);
bool rx_stall_check(struct rx_stall *watch, int tty_fd);

#endif
