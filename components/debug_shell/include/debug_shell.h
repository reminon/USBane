#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SHELL_COLS          39
#define SHELL_ROWS          12
#define SHELL_HISTORY_DEPTH  8
#define SHELL_MAX_CMD_LEN   80
#define SHELL_MAX_CMDS      24

typedef int (*shell_cmd_fn_t)(int argc, char **argv);

typedef struct {
    const char     *name;
    const char     *help;
    shell_cmd_fn_t  fn;
} shell_cmd_t;

/**
 * Initialise the debug shell.
 * Starts UART2 on EXT header (G15/G13 at 115200 8N1),
 * hooks esp_log to queue-based display renderer,
 * initialises TCA8418 keyboard, starts shell/display/UART tasks.
 *
 * Call display_adv_init() before this.
 */
esp_err_t debug_shell_init(void);

/**
 * Register an application-specific shell command.
 * Safe to call before debug_shell_init().
 */
esp_err_t debug_shell_register_cmd(const shell_cmd_t *cmd);

/** Write a line to display + UART2. Safe from any task. */
void debug_shell_println(const char *line);

/** printf-style output to display + UART2. Safe from any task. */
void debug_shell_printf(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

/** Stop shell and free all resources. */
void debug_shell_deinit(void);

#ifdef __cplusplus
}
#endif
void debug_shell_enable_log_hook(void);
