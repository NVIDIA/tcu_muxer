/* SPDX-License-Identifier: Apache-2.0
 * Inject only the watchdog's decision; run the actual muxer RX/reopen/parser.
 * The physical PTY remains connected and never reports HUP.
 */
#include "../rx_stall.h"
#include <stdlib.h>
#include <unistd.h>

bool __wrap_rx_stall_check(struct rx_stall *watch, int tty_fd);
bool __real_rx_stall_check(struct rx_stall *watch, int tty_fd);

bool __wrap_rx_stall_check(struct rx_stall *watch, int tty_fd)
{
    const char *trigger = getenv("TCU_TEST_STALL_TRIGGER");
    if (trigger && unlink(trigger) == 0)
        return true;
    return __real_rx_stall_check(watch, tty_fd);
}
