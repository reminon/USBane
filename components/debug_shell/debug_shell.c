/*
 * debug_shell.c
 *
 * Additive debug shell for USBane on Cardputer ADV.
 * Core USBane functionality (DWC2, chain engine, web interface) is
 * completely unmodified. This component only adds:
 *   - Live log viewer on the ST7789V2 display
 *   - Command shell via TCA8418 keyboard and/or UART2 (EXT header)
 *   - Shell commands that wrap existing USBane API calls
 *
 * Architecture:
 *   esp_log hook  → xQueueSend (non-blocking) → log queue
 *   display task  (Core 0, pri 2) → drains log queue → renders
 *   uart task     (Core 0, pri 3) → feeds UART2 bytes into key queue
 *   shell task    (Core 0, pri 4) → processes key queue, dispatches cmds
 *
 * USB worker runs on Core 1 and is never touched.
 */

#include "debug_shell.h"
#include "cardputer_adv_display.h"
#include "tca8418.h"
#include "usbane.h"
#include "dwc2_backend.h"
#include "chain_engine.h"
#include "dwc2_backend.h"
#include "chain_engine.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include <dirent.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "shell";

/* ── Display geometry ──────────────────────────────────────────── */
#define CHAR_W         6
#define CHAR_H         8
#define SCROLL_Y_START 22
#define INPUT_Y        (135 - CHAR_H - 2)
#define SCROLL_Y_END   (INPUT_Y - 3)
#define SCROLL_ROWS    ((SCROLL_Y_END - SCROLL_Y_START) / CHAR_H)

/* ── Colours RGB565 ────────────────────────────────────────────── */
#define COL_BLACK  0x0000u
#define COL_WHITE  0xFFFFu
#define COL_PROMPT 0x3EFFu
#define COL_ERROR  0xF9A0u
#define COL_WARN   0xFD20u
#define COL_INFO   0xFFFFu
#define COL_CURSOR 0x3EFFu
#define COL_GREY   0xC618u

/* ── UART2 ─────────────────────────────────────────────────────── */
#define SHELL_UART_NUM   UART_NUM_2
#define SHELL_UART_TX    15
#define SHELL_UART_RX    13
#define SHELL_UART_BAUD  115200
#define UART_BUF         512

/* ── Log queue ─────────────────────────────────────────────────── */
#define LOG_Q_DEPTH  32
#define LOG_LINE_MAX 80

typedef struct { char text[LOG_LINE_MAX]; uint16_t colour; } log_line_t;

/* ── Scroll buffer ─────────────────────────────────────────────── */
typedef struct { char text[SHELL_COLS+1]; uint16_t colour; } scroll_line_t;
static scroll_line_t s_scroll[SHELL_ROWS];
static int s_scroll_head = 0, s_scroll_count = 0;

/* ── Input ─────────────────────────────────────────────────────── */
/* Menu state */
typedef enum { MODE_SHELL=0, MODE_MENU, MODE_USB_INFO, MODE_CHAINS } shell_mode_t;
static shell_mode_t s_mode = MODE_SHELL;
static int s_menu_sel = 0;
static const char *s_menu_items[] = {"USB Info", "Browse Chains", "Shell"};
#define MENU_ITEMS 3
static bool s_usb_info_cached = false;
static bool s_menu_dirty = true;
static char s_usb_info_lines[5][40];

static void _load_chains(void);
static void _chain_exec_task(void *arg);
static void _usb_info_fetch_task(void *arg);

/* Chain browser state */
#define MAX_CHAINS 20
static char s_chain_names[MAX_CHAINS][40];
static int  s_chain_count = 0;
static int  s_chain_sel = 0;
static bool s_chains_loaded = false;
static char s_chain_exec_path[64];

static char s_input[SHELL_MAX_CMD_LEN+1];
static int  s_input_len = 0, s_cursor = 0;

/* ── History ───────────────────────────────────────────────────── */
static char s_hist[SHELL_HISTORY_DEPTH][SHELL_MAX_CMD_LEN+1];
static int  s_hist_count = 0, s_hist_idx = -1;

/* ── Command registry ──────────────────────────────────────────── */
static const shell_cmd_t *s_cmds[SHELL_MAX_CMDS];
static int s_cmd_count = 0;

/* ── State ─────────────────────────────────────────────────────── */
static QueueHandle_t     s_log_q    = NULL;
static QueueHandle_t     s_key_q    = NULL;
static SemaphoreHandle_t s_mutex    = NULL;
static TaskHandle_t      s_disp_t   = NULL;
static TaskHandle_t      s_shell_t  = NULL;
static TaskHandle_t      s_uart_t   = NULL;
static bool              s_running  = false;

/* ── Display internals from cardputer_adv_display.c ───────────── */
extern void _fill_rect(int x, int y, int w, int h, uint16_t colour);
extern void _draw_string(int x, int y, const char *s,
                         uint16_t fg, uint16_t bg, int scale);

/* ── Forward declarations ──────────────────────────────────────── */
static void _display_task(void *arg);
static void _shell_task_fn(void *arg);
static void _uart_task(void *arg);
static void _kbd_cb(const tca8418_key_event_t *e, void *arg);
static void _process_char(char c);
static void _execute(void);
static void _scroll_push(const char *t, uint16_t col);
static void _redraw_scroll(void);
static void _redraw_input(void);
static void _uart_write(const char *s);
static int  _log_hook(const char *fmt, va_list args);

/* ── Built-in commands ─────────────────────────────────────────── */
static int cmd_help(int argc, char **argv);
static int cmd_clear(int argc, char **argv);
static int cmd_reboot(int argc, char **argv);
static int cmd_status(int argc, char **argv);
static int cmd_info(int argc, char **argv);
static int cmd_reset(int argc, char **argv);
static int cmd_ctrl(int argc, char **argv);
static int cmd_ep_in(int argc, char **argv);
static int cmd_ep_out(int argc, char **argv);
static int cmd_backend(int argc, char **argv);
static int cmd_autorecov(int argc, char **argv);

static const shell_cmd_t s_builtins[] = {
    {"help",      "List commands",                    cmd_help     },
    {"clear",     "Clear screen",                     cmd_clear    },
    {"reboot",    "Reboot device",                    cmd_reboot   },
    {"status",    "USB + backend status",             cmd_status   },
    {"info",      "USB device info",                  cmd_info     },
    {"backend",   "backend [dwc2|soft]",              cmd_backend  },
    {"reset",     "Send USB bus reset",               cmd_reset    },
    {"ctrl",      "ctrl bmRT bReq wVal wIdx wLen [addr]", cmd_ctrl },
    {"ep_in",     "ep_in ep addr len [ms]",           cmd_ep_in   },
    {"ep_out",    "ep_out ep addr hexdata [ms]",      cmd_ep_out  },
    {"autorecov", "autorecov on|off",                 cmd_autorecov},
};
#define N_BUILTINS (sizeof(s_builtins)/sizeof(s_builtins[0]))

/* ═══════════════════════════════════════════════════════════════
 * Public API
 * ═══════════════════════════════════════════════════════════════ */

esp_err_t debug_shell_register_cmd(const shell_cmd_t *cmd)
{
    if (s_cmd_count >= SHELL_MAX_CMDS) return ESP_ERR_NO_MEM;
    s_cmds[s_cmd_count++] = cmd;
    return ESP_OK;
}

esp_err_t debug_shell_init(void)
{
    if (s_running) return ESP_OK;
    for (int i = 0; i < (int)N_BUILTINS; i++)
        debug_shell_register_cmd(&s_builtins[i]);

    s_mutex = xSemaphoreCreateMutex();
    s_log_q = xQueueCreate(LOG_Q_DEPTH, sizeof(log_line_t));
    s_key_q = xQueueCreate(16, sizeof(tca8418_key_event_t));

    uart_config_t uc = {
        .baud_rate = SHELL_UART_BAUD, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(SHELL_UART_NUM, UART_BUF, UART_BUF, 0, NULL, 0);
    uart_param_config(SHELL_UART_NUM, &uc);
    uart_set_pin(SHELL_UART_NUM, SHELL_UART_TX, SHELL_UART_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    tca8418_init(_kbd_cb, NULL);
    // esp_log_set_vprintf(_log_hook); // TEMP DISABLED

    xTaskCreatePinnedToCore(_display_task,  "sh_disp",  3072, NULL, 2, &s_disp_t,  0);
    xTaskCreatePinnedToCore(_uart_task,     "sh_uart",  2048, NULL, 3, &s_uart_t,  0);
    xTaskCreatePinnedToCore(_shell_task_fn, "sh_shell", 4096, NULL, 4, &s_shell_t, 0);

    s_running = true;
    _uart_write("\r\n=== USBane shell === Type 'help'\r\n> ");
    ESP_LOGI(TAG, "Shell ready  UART2 TX=G%d RX=G%d %dbaud",
             SHELL_UART_TX, SHELL_UART_RX, SHELL_UART_BAUD);
    return ESP_OK;
}

void debug_shell_println(const char *line)
{
    if (!s_running || !line) return;
    log_line_t ll;
    strncpy(ll.text, line, LOG_LINE_MAX-1);
    ll.text[LOG_LINE_MAX-1] = '\0';
    ll.colour = COL_GREY;
    xQueueSend(s_log_q, &ll, 0);
    _uart_write(line); _uart_write("\r\n");
}

void debug_shell_printf(const char *fmt, ...)
{
    char buf[LOG_LINE_MAX*2];
    va_list a; va_start(a, fmt);
    vsnprintf(buf, sizeof(buf), fmt, a);
    va_end(a);
    char *p = buf, *nl;
    while ((nl = strchr(p, '\n'))) { *nl='\0'; debug_shell_println(p); p=nl+1; }
    if (*p) debug_shell_println(p);
}

void debug_shell_deinit(void)
{
    s_running = false;
    esp_log_set_vprintf(vprintf);
    if (s_shell_t) { vTaskDelete(s_shell_t); s_shell_t = NULL; }
    if (s_uart_t)  { vTaskDelete(s_uart_t);  s_uart_t  = NULL; }
    if (s_disp_t)  { vTaskDelete(s_disp_t);  s_disp_t  = NULL; }
    if (s_log_q)   { vQueueDelete(s_log_q);  s_log_q   = NULL; }
    if (s_key_q)   { vQueueDelete(s_key_q);  s_key_q   = NULL; }
    if (s_mutex)   { vSemaphoreDelete(s_mutex); s_mutex = NULL; }
    tca8418_deinit();
    uart_driver_delete(SHELL_UART_NUM);
}

/* ═══════════════════════════════════════════════════════════════
 * Log hook — non-blocking, returns immediately
 * ═══════════════════════════════════════════════════════════════ */

static int _log_hook(const char *fmt, va_list args)
{
    char buf[256];
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    _uart_write(buf);                   /* mirror to UART2 */
    if (!s_log_q) return n;

    log_line_t ll;
    int ci = 0;
    char level = 'I';
    bool esc = false;
    for (int i = 0; i < n && ci < LOG_LINE_MAX-1; i++) {
        if (buf[i] == '\033') { esc = true; continue; }
        if (esc) {
            if (buf[i] == 'm') esc = false;
            if (buf[i]=='I'||buf[i]=='W'||buf[i]=='E'||
                buf[i]=='D'||buf[i]=='V') level = buf[i];
            continue;
        }
        if (buf[i]=='\r'||buf[i]=='\n') continue;
        ll.text[ci++] = buf[i];
    }
    ll.text[ci] = '\0';
    /* Strip "(timestamp) TAG: " prefix */
    char *colon = strstr(ll.text, ": ");
    if (colon) {
        memmove(ll.text, colon + 2, strlen(colon + 2) + 1);
    }
    switch (level) {
        case 'E': ll.colour = COL_ERROR; break;
        case 'W': ll.colour = COL_WARN;  break;
        default:  ll.colour = COL_INFO;  break;
    }
    xQueueSend(s_log_q, &ll, 0);       /* drop if full — never block */
    return n;
}

/* ═══════════════════════════════════════════════════════════════
 * Display task — Core 0, pri 2
 * ═══════════════════════════════════════════════════════════════ */

static void _display_task(void *arg)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    _redraw_scroll(); _redraw_input();
    xSemaphoreGive(s_mutex);

    log_line_t ll;
    while (s_running) {
        /* G0 button polling */
        static bool g0_last = true;
        bool g0_now = (bool)gpio_get_level(0);
        if (g0_last && !g0_now) {
            if (s_mode == MODE_SHELL) { s_mode = MODE_MENU; s_menu_sel = 0; }
            else if (s_mode == MODE_MENU) { s_mode = MODE_SHELL; }
            else { s_mode = MODE_MENU; }
            s_menu_dirty = true;
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            _redraw_scroll(); _redraw_input();
            xSemaphoreGive(s_mutex);
        }
        g0_last = g0_now;

        if (xQueueReceive(s_log_q, &ll, pdMS_TO_TICKS(50)) == pdTRUE) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            _scroll_push(ll.text, ll.colour);
            _redraw_scroll();
            _redraw_input();
            xSemaphoreGive(s_mutex);
        } else if (s_menu_dirty && s_mode != MODE_SHELL) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            _redraw_scroll();
            _redraw_input();
            xSemaphoreGive(s_mutex);
        }
    }
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════
 * UART task — Core 0, pri 3
 * ═══════════════════════════════════════════════════════════════ */

static void _uart_task(void *arg)
{
    uint8_t b, esc = 0;
    while (s_running) {
        if (uart_read_bytes(SHELL_UART_NUM, &b, 1, pdMS_TO_TICKS(20)) != 1) continue;
        if (esc==0 && b==0x1B) { esc=1; continue; }
        if (esc==1 && b=='[')  { esc=2; continue; }
        if (esc==2) {
            esc=0;
            tca8418_key_event_t ke = {.pressed=true,.keycode=0};
            if      (b=='A') ke.ascii=0x01;
            else if (b=='B') ke.ascii=0x02;
            else if (b=='C') ke.ascii=0x06;
            else if (b=='D') ke.ascii=0x05;
            if (ke.ascii) xQueueSend(s_key_q, &ke, 0);
            continue;
        }
        esc=0;
        tca8418_key_event_t ke = {.ascii=(char)b,.keycode=0,.pressed=true};
        xQueueSend(s_key_q, &ke, 0);
    }
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════
 * Shell task — Core 0, pri 4
 * ═══════════════════════════════════════════════════════════════ */

static void _kbd_cb(const tca8418_key_event_t *e, void *arg)
{
    /* Modifier keys — track state but don't insert chars */
    if (e->ascii >= 1 && e->ascii <= 5) return;
    if (!e->pressed || !e->ascii) return;

    /* Menu navigation - intercept all input when not in shell mode */
    if (s_mode == MODE_MENU) {
        if (e->ascii == ';') { /* up */ if (s_menu_sel > 0) { s_menu_sel--; s_menu_dirty = true; } }
        else if (e->ascii == '.') { /* down */ if (s_menu_sel < MENU_ITEMS-1) { s_menu_sel++; s_menu_dirty = true; } }
        else if (e->ascii == ',') { /* left - back */ s_mode = MODE_SHELL; }
        else if (e->ascii == '/') { /* right - select */ /* same as enter */ }
        else if (e->ascii == '\r') {
            if (s_menu_sel == 2) s_mode = MODE_SHELL;
            else if (s_menu_sel == 0) {
                s_mode = MODE_USB_INFO;
                s_menu_dirty = true;
                s_usb_info_cached = false;
                strncpy(s_usb_info_lines[0], "Fetching...", 39);
                s_usb_info_lines[1][0] = s_usb_info_lines[2][0] = s_usb_info_lines[3][0] = 0;
                xTaskCreate(_usb_info_fetch_task, "usb_info", 4096, NULL, 3, NULL);
            }
            else if (s_menu_sel == 1) { s_mode = MODE_CHAINS; s_chain_sel = 0; _load_chains(); s_menu_dirty = true; }
        }
        if (s_mutex) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            _redraw_scroll();
            _redraw_input();
            xSemaphoreGive(s_mutex);
        }
        return;
    }
    if (s_mode == MODE_CHAINS) {
        if (e->ascii == ';') { if (s_chain_sel > 0) { s_chain_sel--; s_menu_dirty = true; } }
        else if (e->ascii == '.') { if (s_chain_sel < s_chain_count-1) { s_chain_sel++; s_menu_dirty = true; } }
        else if (e->ascii == '\r' && s_chain_count > 0) {
            /* Execute selected chain in background task */
            snprintf(s_chain_exec_path, sizeof(s_chain_exec_path), "/spiffs/chains/%s", s_chain_names[s_chain_sel]);
            xTaskCreate(_chain_exec_task, "chain_exec", 8192, NULL, 3, NULL);
            s_mode = MODE_SHELL;
            s_menu_dirty = true;
        }
        if (s_mutex) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            _redraw_scroll(); _redraw_input();
            xSemaphoreGive(s_mutex);
        }
        return;
    }
    if (s_mode != MODE_SHELL) return;
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        if (e->ascii == '\r' || e->ascii == '\n') {
            xSemaphoreGive(s_mutex);
            _execute();
            return;
        } else if (e->ascii == '\b' || e->ascii == 0x7F) {
            if (s_cursor > 0) {
                memmove(&s_input[s_cursor-1], &s_input[s_cursor], s_input_len-s_cursor+1);
                s_cursor--; s_input_len--;
            }
        } else if ((e->ascii >= 0x20 || e->ascii == '\t') && e->ascii < 0x7F && s_input_len < SHELL_MAX_CMD_LEN) {
            memmove(&s_input[s_cursor+1], &s_input[s_cursor], s_input_len-s_cursor+1);
            s_input[s_cursor++] = e->ascii; s_input_len++;
        }
        _redraw_input();
        xSemaphoreGive(s_mutex);
    }
    if (s_key_q && e->pressed) xQueueSend(s_key_q, e, 0);
}

static void _shell_task_fn(void *arg)
{
    ESP_LOGI("SHELL", "shell task started");
    tca8418_key_event_t ke;
    while (s_running) {
        if (xQueueReceive(s_key_q, &ke, pdMS_TO_TICKS(100)) != pdTRUE) continue;
        if (!ke.pressed || !ke.ascii) continue;

        if (ke.ascii == 0x01) {   /* up */
            if (s_hist_idx < s_hist_count-1) {
                s_hist_idx++;
                strncpy(s_input, s_hist[s_hist_idx], SHELL_MAX_CMD_LEN);
                s_input_len = strlen(s_input); s_cursor = s_input_len;
                xSemaphoreTake(s_mutex, portMAX_DELAY); _redraw_input(); xSemaphoreGive(s_mutex);
            }
            continue;
        }
        if (ke.ascii == 0x02) {   /* down */
            if (s_hist_idx > 0) { s_hist_idx--; strncpy(s_input, s_hist[s_hist_idx], SHELL_MAX_CMD_LEN); }
            else { s_hist_idx=-1; memset(s_input,0,sizeof(s_input)); }
            s_input_len = strlen(s_input); s_cursor = s_input_len;
            xSemaphoreTake(s_mutex, portMAX_DELAY); _redraw_input(); xSemaphoreGive(s_mutex);
            continue;
        }
        if (ke.ascii == 0x05) { if (s_cursor>0) s_cursor--; xSemaphoreTake(s_mutex,portMAX_DELAY); _redraw_input(); xSemaphoreGive(s_mutex); continue; }
        if (ke.ascii == 0x06) { if (s_cursor<s_input_len) s_cursor++; xSemaphoreTake(s_mutex,portMAX_DELAY); _redraw_input(); xSemaphoreGive(s_mutex); continue; }
        ESP_LOGI("SHELL", "proc:%c", ke.ascii);
        _process_char(ke.ascii);
    }
    vTaskDelete(NULL);
}

static void _process_char(char c)
{
    if (c=='\r'||c=='\n') { _execute(); return; }
    if (c=='\b'||c==0x7F) {
        if (s_cursor > 0) {
            memmove(&s_input[s_cursor-1], &s_input[s_cursor], s_input_len-s_cursor+1);
            s_cursor--; s_input_len--;
            xSemaphoreTake(s_mutex, portMAX_DELAY); _redraw_input(); xSemaphoreGive(s_mutex);
        }
        return;
    }
    if (c>=0x20 && c<0x7F && s_input_len<SHELL_MAX_CMD_LEN) {
        memmove(&s_input[s_cursor+1], &s_input[s_cursor], s_input_len-s_cursor+1);
        s_input[s_cursor++] = c; s_input_len++;
        char echo[2]={c,0}; _uart_write(echo);
        xSemaphoreTake(s_mutex, portMAX_DELAY); _redraw_input(); xSemaphoreGive(s_mutex);
    }
}

static void _execute(void)
{
    _uart_write("\r\n");
    if (s_input_len == 0) { _uart_write("> "); return; }

    char pline[SHELL_COLS+4];
    snprintf(pline, sizeof(pline), "> %.*s", (int)(sizeof(pline) - 3), s_input);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    _scroll_push(pline, COL_PROMPT); _redraw_scroll();
    xSemaphoreGive(s_mutex);

    memmove(&s_hist[1], &s_hist[0], sizeof(s_hist[0])*(SHELL_HISTORY_DEPTH-1));
    strncpy(s_hist[0], s_input, SHELL_MAX_CMD_LEN);
    if (s_hist_count < SHELL_HISTORY_DEPTH) s_hist_count++;
    s_hist_idx = -1;

    static char ab[SHELL_MAX_CMD_LEN+1];
    static char *av[16];
    strncpy(ab, s_input, SHELL_MAX_CMD_LEN);
    int ac = 0;
    char *tok = strtok(ab, " \t");
    while (tok && ac<15) { av[ac++]=tok; tok=strtok(NULL," \t"); }
    av[ac]=NULL;

    memset(s_input, 0, sizeof(s_input)); s_input_len=0; s_cursor=0;

    bool found = false;
    for (int i=0; i<s_cmd_count; i++) {
        if (strcmp(s_cmds[i]->name, av[0])==0) {
            s_cmds[i]->fn(ac, av); found=true; break;
        }
    }
    if (!found) debug_shell_printf("Unknown: %s  (try 'help')", av[0]);
    _uart_write("> ");
    xSemaphoreTake(s_mutex, portMAX_DELAY); _redraw_input(); xSemaphoreGive(s_mutex);
}

/* ── Scroll / render (call with mutex held) ────────────────────── */

static void _scroll_push(const char *t, uint16_t col)
{
    int idx;
    if (s_scroll_count < SHELL_ROWS) { idx=(s_scroll_head+s_scroll_count)%SHELL_ROWS; s_scroll_count++; }
    else { idx=s_scroll_head; s_scroll_head=(s_scroll_head+1)%SHELL_ROWS; }
    strncpy(s_scroll[idx].text, t, SHELL_COLS); s_scroll[idx].text[SHELL_COLS]='\0';
    s_scroll[idx].colour = col;
}

static void _chain_exec_task(void *arg)
{
    FILE *f = fopen(s_chain_exec_path, "r");
    if (f) {
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        char *csv = malloc(sz+1);
        if (csv) {
            fread(csv, 1, sz, f);
            csv[sz] = 0;
            ESP_LOGI("CHAIN", "Executing: %s", s_chain_exec_path);
            chain_execute_csv(csv, sz, NULL, NULL, NULL);
            free(csv);
        }
        fclose(f);
    }
    vTaskDelete(NULL);
}

static void _load_chains(void)
{
    s_chain_count = 0;
    DIR *d = opendir("/spiffs/chains");
    if (!d) return;
    struct dirent *e;
    while ((e = readdir(d)) != NULL && s_chain_count < MAX_CHAINS) {
        size_t len = strlen(e->d_name);
        if (len > 4 && (strcmp(e->d_name+len-4, ".csv")==0 || strcmp(e->d_name+len-4, ".CSV")==0)) {
            strncpy(s_chain_names[s_chain_count], e->d_name, 39);
            s_chain_count++;
        }
    }
    closedir(d);
    s_chains_loaded = true;
}

static void _usb_info_fetch_task(void *arg)
{
    bool connected = usb_backend_is_device_connected();
    if (!connected) {
        strncpy(s_usb_info_lines[0], "No device connected", 39);
        s_usb_info_lines[1][0] = s_usb_info_lines[2][0] = s_usb_info_lines[3][0] = 0;
    } else {
        usb_device_info_t info = {0};
        esp_err_t r = ESP_FAIL;
        /* Retry a few times in case backend isn't ready */
        for (int i = 0; i < 3 && r != ESP_OK; i++) {
            r = usb_get_device_info(&info);
            if (r != ESP_OK) vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (r == ESP_OK) {
            snprintf(s_usb_info_lines[0], 39, "VID:%04X PID:%04X", info.vid, info.pid);
            snprintf(s_usb_info_lines[1], 39, "%.38s", info.manufacturer);
            snprintf(s_usb_info_lines[2], 39, "%.38s", info.product);
            snprintf(s_usb_info_lines[3], 39, "S/N:%.34s", info.serial);
        } else {
            snprintf(s_usb_info_lines[0], 39, "Err:%s", esp_err_to_name(r));
            s_usb_info_lines[1][0] = s_usb_info_lines[2][0] = s_usb_info_lines[3][0] = 0;
        }
    }
    s_usb_info_cached = true;
    s_menu_dirty = true;
    vTaskDelete(NULL);
}

static void _draw_menu(void)
{
    _fill_rect(0, SCROLL_Y_START, 240, SCROLL_Y_END-SCROLL_Y_START, COL_BLACK);
    if (s_mode == MODE_MENU) {
        _draw_string(0, SCROLL_Y_START, "== MENU ==", COL_PROMPT, COL_BLACK, 1);
        for (int i = 0; i < MENU_ITEMS; i++) {
            uint16_t fg = (i == s_menu_sel) ? COL_BLACK : COL_WHITE;
            uint16_t bg = (i == s_menu_sel) ? COL_PROMPT : COL_BLACK;
            _draw_string(0, SCROLL_Y_START + (i+1)*CHAR_H, s_menu_items[i], fg, bg, 1);
        }
        _draw_string(0, SCROLL_Y_START + (MENU_ITEMS+2)*CHAR_H, "UP/DN=navigate ENT=select", COL_INFO, COL_BLACK, 1);
    } else if (s_mode == MODE_USB_INFO) {
        _draw_string(0, SCROLL_Y_START, "== USB INFO ==", COL_PROMPT, COL_BLACK, 1);
        for (int i = 0; i < 4; i++) {
            if (s_usb_info_lines[i][0])
                _draw_string(0, SCROLL_Y_START+CHAR_H*(i+1), s_usb_info_lines[i], COL_WHITE, COL_BLACK, 1);
        }
    } else if (s_mode == MODE_CHAINS) {
        _draw_string(0, SCROLL_Y_START, "== CHAINS ==", COL_PROMPT, COL_BLACK, 1);
        if (s_chain_count == 0) {
            _draw_string(0, SCROLL_Y_START+CHAR_H, "No chains on SPIFFS", COL_WARN, COL_BLACK, 1);
            _draw_string(0, SCROLL_Y_START+CHAR_H*2, "Upload via web UI or SD", COL_INFO, COL_BLACK, 1);
        } else {
            int visible = (SCROLL_Y_END - SCROLL_Y_START - CHAR_H) / CHAR_H;
            int start = s_chain_sel >= visible ? s_chain_sel - visible + 1 : 0;
            for (int i = 0; i < visible && (start+i) < s_chain_count; i++) {
                int idx = start + i;
                uint16_t fg = (idx == s_chain_sel) ? COL_BLACK : COL_WHITE;
                uint16_t bg = (idx == s_chain_sel) ? COL_PROMPT : COL_BLACK;
                _draw_string(0, SCROLL_Y_START+CHAR_H*(i+1), s_chain_names[idx], fg, bg, 1);
            }
        }
    }
}

static void _redraw_scroll(void)
{
    if (s_mode != MODE_SHELL) {
        if (s_menu_dirty) { _draw_menu(); s_menu_dirty = false; }
        return;
    }
    _fill_rect(0, SCROLL_Y_START, 240, SCROLL_Y_END-SCROLL_Y_START, COL_BLACK);
    int vis   = s_scroll_count < SCROLL_ROWS ? s_scroll_count : SCROLL_ROWS;
    int start = (s_scroll_head + s_scroll_count - vis + SHELL_ROWS) % SHELL_ROWS;
    for (int i=0; i<vis; i++) {
        int bi = (start+i)%SHELL_ROWS;
        _draw_string(0, SCROLL_Y_START+i*CHAR_H, s_scroll[bi].text, s_scroll[bi].colour, COL_BLACK, 1);
    }
}

static void _redraw_input(void)
{
    _fill_rect(0, INPUT_Y-2, 240, 1, COL_PROMPT);
    _fill_rect(0, INPUT_Y, 240, CHAR_H+1, COL_BLACK);
    _draw_string(0, INPUT_Y, ">", COL_PROMPT, COL_BLACK, 1);
    char d[SHELL_COLS+1]; strncpy(d, s_input, SHELL_COLS); d[SHELL_COLS]='\0';
    _draw_string(CHAR_W*2, INPUT_Y, d, COL_WHITE, COL_BLACK, 1);
    int cx = CHAR_W*2 + s_cursor*CHAR_W;
    char cc[2]={s_input[s_cursor]?s_input[s_cursor]:' ',0};
    _draw_string(cx, INPUT_Y, cc, COL_BLACK, COL_CURSOR, 1);
}

static void _uart_write(const char *s)
{
    if (s) uart_write_bytes(SHELL_UART_NUM, s, strlen(s));
}

/* ═══════════════════════════════════════════════════════════════
 * Built-in command implementations
 * ═══════════════════════════════════════════════════════════════ */

static int cmd_help(int argc, char **argv)
{
    for (int i=0; i<s_cmd_count; i++)
        debug_shell_printf("%-12s %s", s_cmds[i]->name, s_cmds[i]->help);
    return 0;
}

static int cmd_clear(int argc, char **argv)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_scroll_head=s_scroll_count=0; _redraw_scroll();
    xSemaphoreGive(s_mutex); return 0;
}

static int cmd_reboot(int argc, char **argv)
{
    debug_shell_println("Rebooting..."); vTaskDelay(pdMS_TO_TICKS(300)); esp_restart(); return 0;
}

static int cmd_status(int argc, char **argv)
{
    debug_shell_printf("USB:     %s", usb_backend_is_device_connected() ? "CONNECTED" : "not connected");
    debug_shell_printf("Backend: %s", usb_backend_type_name(usb_backend_get_type()));
    debug_shell_printf("AutoRec: %s", usb_is_auto_recovery_enabled() ? "on" : "off");
    return 0;
}

static int cmd_info(int argc, char **argv)
{
    usb_device_info_t info = {0};
    if (usb_get_device_info(&info) != ESP_OK) {
        debug_shell_println("No device / not enumerated"); return 1;
    }
    debug_shell_printf("VID:PID  %04X:%04X", info.vid, info.pid);
    debug_shell_printf("Class    %02X/%02X/%02X", info.device_class, info.device_subclass, info.device_protocol);
    debug_shell_printf("MPS      %d", info.max_packet_size);
    if (info.manufacturer[0]) debug_shell_printf("Mfr  %s", info.manufacturer);
    if (info.product[0])      debug_shell_printf("Prod %s", info.product);
    if (info.serial[0])       debug_shell_printf("Ser  %s", info.serial);
    return 0;
}

static int cmd_reset(int argc, char **argv)
{
    debug_shell_println("USB reset...");
    esp_err_t r = usb_backend_reset();
    debug_shell_printf("reset: %s", esp_err_to_name(r));
    return (r==ESP_OK)?0:1;
}

/* ctrl bmRT bReq wVal wIdx wLen [addr] */
static int cmd_ctrl(int argc, char **argv)
{
    if (argc < 6) {
        debug_shell_println("ctrl bmRT bReq wVal wIdx wLen [addr]");
        debug_shell_println("  e.g. ctrl 0x80 0x06 0x0100 0 18");
        return 1;
    }
    uint8_t rx[256]={0}; size_t rxlen=0;
    usb_packet_config_t cfg = usb_packet_config_default();
    cfg.bmRequestType       = (uint8_t) strtoul(argv[1],NULL,16);
    cfg.bRequest            = (uint8_t) strtoul(argv[2],NULL,16);
    cfg.wValue              = (uint16_t)strtoul(argv[3],NULL,16);
    cfg.wIndex              = (uint16_t)strtoul(argv[4],NULL,16);
    cfg.wLength             = (uint16_t)strtoul(argv[5],NULL,0);
    cfg.device_addr         = (argc>=7)?(uint8_t)strtoul(argv[6],NULL,0):0;
    cfg.packet_size         = USB_SETUP_PACKET_SIZE;
    cfg.expect_response     = (cfg.wLength>0);
    cfg.response_buffer     = rx;
    cfg.response_buffer_size= sizeof(rx);
    cfg.bytes_received      = &rxlen;
    cfg.timeout_ms          = USB_DEFAULT_TIMEOUT_MS;
    cfg.max_nak_retries     = -1;

    debug_shell_printf(">> %02X %02X %04X %04X len=%d addr=%d",
        cfg.bmRequestType, cfg.bRequest, cfg.wValue, cfg.wIndex, cfg.wLength, cfg.device_addr);

    esp_err_t r = usb_backend_send_packet(&cfg);
    if (r != ESP_OK) { debug_shell_printf("ERR: %s", esp_err_to_name(r)); return 1; }
    debug_shell_printf("OK rx=%d bytes", (int)rxlen);
    for (size_t i=0; i<rxlen; i+=16) {
        char line[64]={0}; int pos=0;
        for (size_t j=i; j<rxlen&&j<i+16; j++)
            pos+=snprintf(line+pos,sizeof(line)-pos,"%02X ",rx[j]);
        debug_shell_println(line);
    }
    return 0;
}

/* ep_in ep addr len [timeout_ms] */
static int cmd_ep_in(int argc, char **argv)
{
    if (argc < 4) { debug_shell_println("ep_in ep addr len [ms]"); return 1; }
    uint8_t  ep   = (uint8_t) strtoul(argv[1],NULL,0);
    uint8_t  addr = (uint8_t) strtoul(argv[2],NULL,0);
    size_t   len  = (size_t)  strtoul(argv[3],NULL,0);
    uint32_t ms   = (argc>=5)?strtoul(argv[4],NULL,0):USB_DEFAULT_TIMEOUT_MS;
    if (!len||len>512) { debug_shell_println("len: 1-512"); return 1; }
    uint8_t *buf = malloc(len);
    if (!buf) { debug_shell_println("OOM"); return 1; }
    size_t rx=0;
    esp_err_t r = usb_backend_endpoint_in(ep, addr, USB_EP_TYPE_BULK, 0, buf, len, ms, &rx);
    if (r!=ESP_OK) { debug_shell_printf("ERR ep%d: %s",ep,esp_err_to_name(r)); free(buf); return 1; }
    debug_shell_printf("ep%d rx=%d bytes", ep, (int)rx);
    for (size_t i=0; i<rx; i+=16) {
        char line[64]={0}; int pos=0;
        for (size_t j=i; j<rx&&j<i+16; j++)
            pos+=snprintf(line+pos,sizeof(line)-pos,"%02X ",buf[j]);
        debug_shell_println(line);
    }
    free(buf); return 0;
}

/* ep_out ep addr hexdata [timeout_ms] */
static int cmd_ep_out(int argc, char **argv)
{
    if (argc < 4) { debug_shell_println("ep_out ep addr hexdata [ms]"); return 1; }
    uint8_t    ep   = (uint8_t)strtoul(argv[1],NULL,0);
    uint8_t    addr = (uint8_t)strtoul(argv[2],NULL,0);
    const char *hex = argv[3];
    uint32_t   ms   = (argc>=5)?strtoul(argv[4],NULL,0):USB_DEFAULT_TIMEOUT_MS;
    size_t hexlen = strlen(hex);
    if (!hexlen||hexlen%2) { debug_shell_println("hexdata must be even-length"); return 1; }
    size_t dlen = hexlen/2;
    uint8_t *data = malloc(dlen);
    if (!data) { debug_shell_println("OOM"); return 1; }
    for (size_t i=0; i<dlen; i++) {
        char b[3]={hex[i*2],hex[i*2+1],0}; data[i]=(uint8_t)strtoul(b,NULL,16);
    }
    esp_err_t r = usb_backend_endpoint_out(ep, addr, USB_EP_TYPE_BULK, 0, data, dlen, ms);
    debug_shell_printf("ep%d out %d bytes: %s", ep, (int)dlen, esp_err_to_name(r));
    free(data); return (r==ESP_OK)?0:1;
}

static int cmd_backend(int argc, char **argv)
{
    if (argc < 2) { debug_shell_printf("backend: %s", usb_backend_type_name(usb_backend_get_type())); return 0; }
    usb_backend_type_t t;
    if      (strcmp(argv[1],"dwc2")==0) t=USB_BACKEND_DWC2;
    else if (strcmp(argv[1],"soft")==0) t=USB_BACKEND_SOFT_HOST;
    else { debug_shell_println("Usage: backend [dwc2|soft]"); return 1; }
    esp_err_t r = usb_backend_set_type(t);
    debug_shell_printf("backend %s: %s", argv[1], esp_err_to_name(r));
    return (r==ESP_OK)?0:1;
}

static int cmd_autorecov(int argc, char **argv)
{
    if (argc < 2) { debug_shell_printf("autorecov: %s", usb_is_auto_recovery_enabled()?"on":"off"); return 0; }
    bool en = (strcmp(argv[1],"on")==0);
    usb_set_auto_recovery_enabled(en);
    debug_shell_printf("autorecov: %s", en?"on":"off");
    return 0;
}

void debug_shell_enable_log_hook(void)
{
    esp_log_set_vprintf(_log_hook);
}
