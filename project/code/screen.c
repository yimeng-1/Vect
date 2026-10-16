#include "screen.h"
#include "board_cfg.h"
#include "control.h"
#include "encoder.h"
#include "track.h"

/* 页面底色。 */
#define UI_BG       (RGB565_WHITE)
/* 顶部卡片底色。 */
#define UI_CARD     (IPS200PRO_RGB888_TO_RGB565(0xF4, 0xF7, 0xFA))
/* 卡片边框颜色。 */
#define UI_BORDER   (IPS200PRO_RGB888_TO_RGB565(0xC7, 0xD3, 0xDF))
/* 普通文字颜色。 */
#define UI_TEXT     (IPS200PRO_RGB888_TO_RGB565(0x32, 0x3A, 0x43))
/* 终端面板背景，使用浅色避免黑底。 */
#define UI_TERM_BG  (IPS200PRO_RGB888_TO_RGB565(0xF4, 0xF7, 0xFA))
/* 终端正常文字。 */
#define UI_TERM_OK  (IPS200PRO_RGB888_TO_RGB565(0x1A, 0x86, 0x55))
/* 终端警告文字。 */
#define UI_TERM_WARN (IPS200PRO_RGB888_TO_RGB565(0xB2, 0x70, 0x00))
/* 终端错误文字。 */
#define UI_TERM_ERR (IPS200PRO_RGB888_TO_RGB565(0xC0, 0x32, 0x2A))

uint8 screen_ready = 0;

uint16 screen_page;
uint16 screen_head;
uint16 screen_sub;
uint16 screen_log_box;
uint16 screen_log;
uint16 screen_yaw;
uint16 screen_yaw_text;
uint16 screen_encoder;
uint16 screen_encoder_text;
uint16 screen_fb;
uint16 screen_out;

/* C251 工程的格式化函数不支持 %f 和 %02d，手动生成两位小数。 */
void screen_fixed(float value, char *out)
{
    int32 scaled;
    uint32 mag;
    uint32 whole;
    uint8 frac;
    char rev[10];
    uint8 i;
    uint8 n;
    uint8 pos;

    scaled = value >= 0.0f
           ? (int32)(value * 100.0f + 0.5f)
           : (int32)(value * 100.0f - 0.5f);
    pos = 0;
    if (scaled < 0)
    {
        out[pos++] = '-';
        mag = (uint32)(-scaled);
    }
    else
    {
        mag = (uint32)scaled;
    }

    whole = mag / 100U;
    frac = (uint8)(mag % 100U);
    n = 0;
    do
    {
        rev[n++] = (char)('0' + whole % 10U);
        whole /= 10U;
    } while (whole > 0U && n < sizeof(rev));

    for (i = 0; i < n; i++)
    {
        out[pos++] = rev[n - i - 1U];
    }
    out[pos++] = '.';
    out[pos++] = (char)('0' + frac / 10U);
    out[pos++] = (char)('0' + frac % 10U);
    out[pos] = '\0';
}

/* PWM 只显示整数，避免左右值过长被屏幕裁剪。 */
void screen_int(float value, char *out)
{
    int32 number;
    uint32 mag;
    char rev[10];
    uint8 pos;
    uint8 n;
    uint8 i;

    number = value >= 0.0f ? (int32)(value + 0.5f) : (int32)(value - 0.5f);
    pos = 0;
    if (number < 0)
    {
        out[pos++] = '-';
        mag = (uint32)(-number);
    }
    else
    {
        mag = (uint32)number;
    }

    n = 0;
    do
    {
        rev[n++] = (char)('0' + mag % 10U);
        mag /= 10U;
    } while (mag > 0U && n < sizeof(rev));

    for (i = 0; i < n; i++)
    {
        out[pos++] = rev[n - i - 1U];
    }
    out[pos] = '\0';
}

void screen_card_style(uint16 id, uint16 background)
{
    ips200pro_set_color(id, COLOR_BACKGROUND, background);
    ips200pro_set_color(id, COLOR_BORDER, UI_BORDER);
    ips200pro_container_radius(id, 1, 6);
}

void screen_label_style(uint16 id, uint16 foreground, uint16 background)
{
    ips200pro_set_color(id, COLOR_FOREGROUND, foreground);
    ips200pro_set_color(id, COLOR_BACKGROUND, background);
    ips200pro_set_color(id, COLOR_BORDER, UI_BORDER);
    ips200pro_set_font(id, FONT_SIZE_12);
    ips200pro_label_mode(id, LABEL_CLIP);
}

void screen_init(void)
{
    screen_page = ips200pro_init("", IPS200PRO_TITLE_BOTTOM, 0);
    if (screen_page == 0)
    {
        return;
    }

    screen_ready = 1;
    ips200pro_set_format(IPS200PRO_FORMAT_GBK);
    ips200pro_set_direction(IPS200PRO_PORTRAIT);
    ips200pro_set_default_font(FONT_SIZE_12);
    ips200pro_set_backlight(255);
    ips200pro_set_color(screen_page, COLOR_BACKGROUND, UI_BG);
    ips200pro_set_color(screen_page, COLOR_FOREGROUND, UI_TEXT);
    ips200pro_set_color(screen_page, COLOR_PAGE_SELECTED_BG, UI_BG);
    ips200pro_set_color(screen_page, COLOR_PAGE_SELECTED_TEXT, UI_TEXT);
    ips200pro_page_switch(screen_page, PAGE_ANIM_OFF);

    /* 顶部信息条。 */
    screen_head = ips200pro_container_create(8, 8, 224, 30);
    screen_sub = ips200pro_label_create(16, 13, 208, 20);

    /* 固定终端状态区域，显示启动状态和故障信息。 */
    screen_log_box = ips200pro_container_create(8, 44, 224, 54);
    screen_log = ips200pro_label_create(16, 50, 208, 40);

    /* 显示角速度环和速度环诊断量，不再占用 16 路 ADC 区域。 */
    screen_yaw = ips200pro_container_create(8, 104, 224, 40);
    screen_encoder = ips200pro_container_create(8, 150, 224, 72);
    screen_yaw_text = ips200pro_label_create(16, 109, 208, 30);
    /* 每行独立发送，避免三行合计超过屏幕库的格式化缓冲区。 */
    screen_encoder_text = ips200pro_label_create(16, 155, 208, 20);
    screen_fb = ips200pro_label_create(16, 177, 208, 20);
    screen_out = ips200pro_label_create(16, 199, 208, 20);

    screen_card_style(screen_head, UI_CARD);
    screen_card_style(screen_log_box, UI_TERM_BG);
    screen_card_style(screen_yaw, UI_CARD);
    screen_card_style(screen_encoder, UI_CARD);
    screen_label_style(screen_sub, UI_TEXT, UI_CARD);
    screen_label_style(screen_log, UI_TERM_OK, UI_TERM_BG);
    screen_label_style(screen_yaw_text, UI_TEXT, UI_CARD);
    screen_label_style(screen_encoder_text, UI_TEXT, UI_CARD);
    screen_label_style(screen_fb, UI_TEXT, UI_CARD);
    screen_label_style(screen_out, UI_TEXT, UI_CARD);

    ips200pro_label_printf(screen_sub, "STC32G144        STATUS");
    ips200pro_label_printf(screen_log,
                           "> IMU660RC     CHECKING\r\n"
                           "> ERROR        CHECKING");
    ips200pro_label_printf(screen_sub, "TRK I +0.00  O +0.00");
    ips200pro_label_printf(screen_yaw_text, "YAW I +0.00\nYAW O +0.00");
    ips200pro_label_printf(screen_encoder_text,
                           "T L0.00 R0.00");
    ips200pro_label_printf(screen_fb, "FB L0.00 R0.00");
    ips200pro_label_printf(screen_out, "OUT L0 R0");
}

void screen_task(void)
{
    uint16 log_color;
    char left_text[16];
    char right_text[16];
    char error_text[16];
    char steer_text[16];
    char yaw_in_text[16];
    char yaw_out_text[16];
    char left_target_text[16];
    char right_target_text[16];
    char left_out_text[16];
    char right_out_text[16];
    char *stage_text;

    if (!screen_ready)
    {
        return;
    }

    screen_fixed(encoder_left.speed, left_text);
    screen_fixed(encoder_right.speed, right_text);
    screen_fixed(steer_in, error_text);
    screen_fixed(steer_out, steer_text);
    screen_fixed(yaw_rate, yaw_in_text);
    screen_fixed(yaw_ctrl.diff_speed, yaw_out_text);
    screen_fixed(pid_speed_left.target, left_target_text);
    screen_fixed(pid_speed_right.target, right_target_text);
    screen_int(pid_speed_left.out, left_out_text);
    screen_int(pid_speed_right.out, right_out_text);
    if (steer_stage_id == 2U)
    {
        stage_text = "TURN";
    }
    else if (!track.valid)
    {
        stage_text = "WAIT";
    }
    else if (steer_stage_id == 0U)
    {
        stage_text = "STR";
    }
    else if (steer_stage_id == 1U)
    {
        stage_text = "CUR";
    }
    else
    {
        stage_text = "TURN";
    }
    ips200pro_label_printf(screen_sub, "TRK %s I%s O%s",
                           stage_text, error_text, steer_text);
    ips200pro_label_printf(screen_yaw_text, "YAW I %s\nYAW O %s",
                           yaw_in_text, yaw_out_text);
    ips200pro_label_printf(screen_encoder_text,
                           "T L%s R%s",
                           left_target_text, right_target_text);
    ips200pro_label_printf(screen_out, "OUT L%s R%s",
                           left_out_text, right_out_text);
    ips200pro_label_printf(screen_fb, "FB L%s R%s", left_text, right_text);
    log_color = UI_TERM_OK;

    if (!imu_ok)
    {
        log_color = UI_TERM_ERR;
        ips200pro_label_printf(screen_log,
                               "> IMU660RC     ERROR\r\n"
                               "> ERROR 01     IMU INIT");
    }
    else if (!pid_running || !steer_running)
    {
        log_color = UI_TERM_ERR;
        ips200pro_label_printf(screen_log,
                               "> IMU660RC     OK\r\n"
                               "> ERROR 04     CONTROL");
    }
    else if (!track.valid)
    {
        log_color = UI_TERM_WARN;
        ips200pro_label_printf(screen_log,
                               "> MOTOR        %s\r\n"
                               "> TRACK        NO LINE",
                               speed_enable ? "RUNNING" : "STOPPED");
    }
    else if (!speed_enable)
    {
        log_color = UI_TERM_WARN;
        ips200pro_label_printf(screen_log,
                               "> CONTROL      READY\r\n"
                               "> MOTOR WAIT   B3");
    }
    else
    {
        ips200pro_label_printf(screen_log,
                               "> MOTOR        RUNNING\r\n"
                               "> CONTROL      ACTIVE");
    }

    ips200pro_set_color(screen_log, COLOR_FOREGROUND, log_color);
}
