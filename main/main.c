/*
 * r3car — 阶段 A：台架测试固件
 *
 * 只做三件事：驱动两个电机、读两个编码器、串口命令控制。
 * 不含 WiFi。目的是在装车之前把「驱动 + 电机 + 编码器」验证完。
 *
 * 板：ESP32-S3-N16R8（板 B）
 * 驱动：BTS7960 / IBT-2 ×2，VCC=5V，逻辑 3.3V 已实测可用
 *
 * 上电后所有输出为 0，必须手动输入命令才会动 —— 这是故意的。
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include <stdlib.h>

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "wifi_cred.h"

static const char *TAG = "r3car";

/* ================= 引脚定义 ================= */
#define PIN_L_FWD    8      /* 左电机 RPWM */
#define PIN_L_REV    9      /* 左电机 LPWM */
#define PIN_R_FWD    14     /* 右电机 RPWM */
#define PIN_R_REV    15     /* 右电机 LPWM */

#define PIN_L_ENC_A  16
#define PIN_L_ENC_B  21
#define PIN_R_ENC_A  2
#define PIN_R_ENC_B  47

/* ================= PWM 参数 ================= */
#define PWM_RES_HZ        1000000   /* 1 MHz 计数时钟 */
#define PWM_PERIOD_TICKS  50        /* 1MHz / 50 = 20 kHz，超出人耳 */
#define DIR_GAP_MS        10        /* 换向时两路拉低的空档 */

/* 编码器：13 PPR × 30 减速比 × 4 倍频 = 1560 计数/输出轴转 */
#define ENC_COUNTS_PER_REV  1560

/* 每侧编码器的方向符号。
   左右电机在车上镜像安装，同样的接线会得到相反的计数方向，
   所以用符号位统一成"油门为正 = rpm 为正 = 车往前走"。
   2026-11-18 实测：左侧正转命令下读数为负 -> 左 = -1。 */
#define ENC_SIGN_L  (-1)
#define ENC_SIGN_R  (+1)

/* ================= 数据结构 ================= */
typedef struct {
    mcpwm_gen_handle_t  fwd;
    mcpwm_gen_handle_t  rev;
    mcpwm_cmpr_handle_t cmp;
    int                 duty;   /* -1000 ~ +1000 */
} motor_t;

typedef struct {
    pcnt_unit_handle_t unit;
    int32_t            total;   /* 累计计数 */
    int                last_edges;  /* 上一次 100ms 窗口内的边沿数 */
    int                sign;    /* 方向符号，见 ENC_SIGN_x */
} encoder_t;

static motor_t   g_left  = { 0 };
static motor_t   g_right = { 0 };
static encoder_t g_enc_l = { 0 };
static encoder_t g_enc_r = { 0 };

static volatile int g_throttle = 0;   /* -1000 ~ +1000 */
static volatile int g_steer    = 0;   /* -1000 ~ +1000，右正 */
static volatile int g_side     = 0;   /* 0=双侧 1=仅左 2=仅右 */

/* 最新一帧编码器换算结果（由 control_task 每 10 ms 更新） */
static volatile float g_rpm_l = 0.0f;
static volatile float g_rpm_r = 0.0f;

/* ---- WiFi / UDP ---- */
#define WIFI_SSID        "r3car"
#define WIFI_PASS        "r3car1234"
#define UDP_PORT         3333
#define CMD_TIMEOUT_MS   200     /* 超过这么久没包 -> 目标归零 */
#define NET_LOST_MS      2000    /* 超过这么久没包 -> 退出网络模式 */

typedef struct {
    volatile int      throttle;
    volatile int      steer;
    volatile bool     armed;
    volatile uint32_t last_ms;
    volatile uint16_t seq;
} remote_cmd_t;

static remote_cmd_t      g_rc = {0};
static volatile bool     g_net_mode = false;
static volatile uint32_t g_pkt_count = 0;
static volatile uint32_t g_loss_count = 0;
static uint16_t          g_tx_seq = 0;
static struct sockaddr_in g_peer;
static volatile bool     g_have_peer = false;
static volatile uint32_t g_serial_ms = 0;   /* 最近一次串口遥控帧的时刻 */

/* ---- 电池电压检测 ---- */
#define BAT_ADC_CH      ADC_CHANNEL_0    /* GPIO1 = ADC1_CH0 */
/* 分压：上 470k / 下 10k -> 比例 48。换回 47k 时改成 57。
 * 470k 时分压点只有 0.26V，必须用 0 dB 档（满量程约 1V）才够分辨率；
 * 换回 47k（2.2V）时衰减要改回 ADC_ATTEN_DB_12。 */
#define BAT_DIV_NUM     523              /* 实测标定：12.30V 对应读数 11.28V，比例 52.3 */
#define BAT_DIV_DEN     10               /* 10k */
#define BAT_ATTEN       ADC_ATTEN_DB_0
#define BAT_FS_MV       1000             /* 0dB 档满量程约 1V，仅作无校准时的兜底 */
static adc_oneshot_unit_handle_t g_adc1 = NULL;
static adc_cali_handle_t         g_adc_cali = NULL;
static volatile uint16_t         g_vbat_mv = 0;

/* ---- 速度闭环 ---- */
#define MAX_RPM   330.0f
#define KP_RPM    2.0f
#define KI_RPM    3.0f
#define INT_LIMIT 500.0f

typedef struct { float integ; int last_sign; } pi_t;
static pi_t g_pi_l = {0}, g_pi_r = {0};
static volatile int g_test_rpm = 0;   /* >0 时进入闭环自测，忽略摇杆 */

/* 先声明，console_task 里要用 */
static void     handle_packet(const uint8_t *p, int n, int src);
static uint16_t crc16_ccitt(const uint8_t *d, int n);
static uint32_t now_ms(void);

/* ================= 工具 ================= */
static int clamp1000(int v)
{
    if (v >  1000) return  1000;
    if (v < -1000) return -1000;
    return v;
}

/* ================= MCPWM ================= */
static void gen_bind(mcpwm_gen_handle_t gen, mcpwm_cmpr_handle_t cmp)
{
    /* 计时器归零 -> 拉高；计到比较值 -> 拉低。两路共用同一个比较器，
       靠 force_level 决定哪一路真正输出。 */
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       cmp,
                                       MCPWM_GEN_ACTION_LOW)));
}

static void motor_init(motor_t *m, int gpio_fwd, int gpio_rev,
                       mcpwm_timer_handle_t timer)
{
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    mcpwm_oper_handle_t oper = NULL;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = gpio_fwd };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &m->fwd));

    gen_cfg.gen_gpio_num = gpio_rev;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &m->rev));

    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &m->cmp));

    gen_bind(m->fwd, m->cmp);
    gen_bind(m->rev, m->cmp);

    /* 上电一律拉低，车不动 */
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->fwd, 0, true));
    ESP_ERROR_CHECK(mcpwm_generator_set_force_level(m->rev, 0, true));
    m->duty = 0;
}

/* duty: -1000 ~ +1000。正 = 正转，负 = 反转，0 = 停止（滑行） */
static void motor_set(motor_t *m, int duty)
{
    duty = clamp1000(duty);

    /* 换向时先两路拉低，等 10 ms 再给新方向，避免上下桥直通 */
    if ((m->duty > 0 && duty < 0) || (m->duty < 0 && duty > 0)) {
        mcpwm_generator_set_force_level(m->fwd, 0, true);
        mcpwm_generator_set_force_level(m->rev, 0, true);
        vTaskDelay(pdMS_TO_TICKS(DIR_GAP_MS));
    }

    int mag = (duty < 0) ? -duty : duty;
    uint32_t ticks = (uint32_t)(((int64_t)mag * PWM_PERIOD_TICKS) / 1000);
    if (mag > 0 && ticks == 0) ticks = 1;
    if (ticks >= PWM_PERIOD_TICKS) ticks = PWM_PERIOD_TICKS - 1;   /* 留一点，自举才稳 */

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(m->cmp, ticks));

    if (duty > 0) {
        mcpwm_generator_set_force_level(m->rev, 0,  true);
        mcpwm_generator_set_force_level(m->fwd, -1, true);
    } else if (duty < 0) {
        mcpwm_generator_set_force_level(m->fwd, 0,  true);
        mcpwm_generator_set_force_level(m->rev, -1, true);
    } else {
        mcpwm_generator_set_force_level(m->fwd, 0, true);
        mcpwm_generator_set_force_level(m->rev, 0, true);
    }
    m->duty = duty;
}

/* ================= 编码器 ================= */
static void enc_chan(pcnt_unit_handle_t unit, int edge_gpio, int level_gpio, bool flip)
{
    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num  = edge_gpio,
        .level_gpio_num = level_gpio,
    };
    pcnt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &chan_cfg, &chan));

    /* 正交解码。两路的方向必须相反，否则四次边沿互相抵消、计数≈0。
       这是 IDF encoder 例程的关键点，漏了会表现为"转半天只有 ±1"。 */
    if (!flip) {
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    } else {
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    }
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
}

static void encoder_init(encoder_t *e, int gpio_a, int gpio_b, int unit_index)
{
    e->sign = (unit_index == 0) ? ENC_SIGN_L : ENC_SIGN_R;
    pcnt_unit_config_t unit_cfg = {
        .high_limit = 30000,
        .low_limit  = -30000,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &e->unit));

    enc_chan(e->unit, gpio_a, gpio_b, false);  /* A 边沿 / B 电平 */
    enc_chan(e->unit, gpio_b, gpio_a, true);   /* B 边沿 / A 电平，方向相反 -> 4 倍频 */

    ESP_ERROR_CHECK(pcnt_unit_enable(e->unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(e->unit));
    ESP_ERROR_CHECK(pcnt_unit_start(e->unit));

    e->total = 0;
    e->last_edges = 0;
}

/* 每 100 ms 调一次：读出窗口内的边沿数并清零 */
static int encoder_poll(encoder_t *e)
{
    int v = 0;
    pcnt_unit_get_count(e->unit, &v);
    pcnt_unit_clear_count(e->unit);

    /* 超过 ±30000 说明窗口内转太快或漏采，钳一下防止跳变 */
    if (v >  30000) v =  30000;
    if (v < -30000) v = -30000;

    v *= e->sign;               /* 统一成"正 = 车往前走" */

    e->last_edges = v;
    e->total += v;
    return v;
}

/* ================= 底盘动作 ================= */
static void chassis_apply(void)
{
    int t = g_throttle;
    int s = g_steer;
    int l = clamp1000(t + s);
    int r = clamp1000(t - s);

    if (g_side == 1) r = 0;      /* 只测左边：右侧强制 0 */
    if (g_side == 2) l = 0;      /* 只测右边 */

    motor_set(&g_left,  l);
    motor_set(&g_right, r);
}

static void chassis_stop(void)
{
    g_throttle = 0;
    g_steer    = 0;
    motor_set(&g_left,  0);
    motor_set(&g_right, 0);
}

/* ================= 任务 ================= */
/* AP 模式下已连客户端的信号强度（没客户端返回 0）。户外测链路时看这个数。 */
static int ap_client_rssi(void)
{
    wifi_sta_list_t list;
    if (esp_wifi_ap_get_sta_list(&list) != ESP_OK || list.num <= 0) return 0;
    return list.sta[0].rssi;
}

static void telemetry_task(void *arg)
{
    (void)arg;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (g_serial_ms != 0 && now_ms() - g_serial_ms < 1000) continue;

        printf("[%s] L:%6.1f/%4d R:%6.1f/%4d rpm/duty | %.2fV | 目标%4d | pkt %lu | ap %d dBm | 车丢 %lu\n",
               g_net_mode ? "NET" : "USB",
               g_rpm_l, g_left.duty,
               g_rpm_r, g_right.duty,
               g_vbat_mv / 1000.0,
               g_test_rpm,
               (unsigned long)g_pkt_count,
               ap_client_rssi(),
               (unsigned long)g_loss_count);
    }
}

static void self_test(void)
{
    const int steps[] = { 200, 500, 800, 1000, 800, 500, 200, 0 };
    const int n = sizeof(steps) / sizeof(steps[0]);

    printf("\n=== 自检开始：正转 ===\n");
    for (int i = 0; i < n; i++) {
        g_throttle = steps[i];
        g_steer    = 0;
        chassis_apply();
        printf("  throttle=%4d\n", steps[i]);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    printf("=== 反转 ===\n");
    for (int i = 0; i < n; i++) {
        g_throttle = -steps[i];
        g_steer    = 0;
        chassis_apply();
        printf("  throttle=%4d\n", -steps[i]);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }

    chassis_stop();
    printf("=== 自检结束 ===\n\n");
}

static void console_task(void *arg)
{
    (void)arg;

    printf("\n");
    printf("=================================\n");
    printf(" r3car 阶段A 台架测试\n");
    printf(" 输入命令后回车：\n");
    printf("   f  前进 30%%      b  后退 30%%\n");
    printf("   l  左转（原地）    r  右转（原地）\n");
    printf("   s  停止\n");
    printf("   0-9 设置油门 0-90%%\n");
    printf("   t  自动自检序列\n");
    printf("   e  打印编码器累计值\n");
    printf("   L  只测左侧   R  只测右侧   A  两侧\n");
    printf("   p  打印编码器引脚原始电平（诊断用）\n");
    printf("   w  扫描周边 WiFi（看车能连哪个 2.4G）\n");
    printf("=================================\n\n");

    static uint8_t sbuf[16];
    static int     slen = 0;

    while (1) {
        int ch = getchar();
        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* --- 先看是不是二进制遥控帧 0xA5 0x5A ... 12B --- */
        uint8_t c = (uint8_t)ch;
        if (slen == 0) {
            if (c == 0xA5) { sbuf[slen++] = c; continue; }
        } else {
            sbuf[slen++] = c;
            if (slen == 2 && sbuf[1] != 0x5A) { slen = 0; continue; }
            if (slen >= 12) {
                handle_packet(sbuf, 12, 1);
                slen = 0;
            }
            continue;
        }

        switch (ch) {
        case 'f': g_throttle =  300; g_steer = 0; chassis_apply(); break;
        case 'b': g_throttle = -300; g_steer = 0; chassis_apply(); break;
        case 'l': g_throttle =  300; g_steer = -300; chassis_apply(); break;
        case 'r': g_throttle =  300; g_steer =  300; chassis_apply(); break;
        case 's': chassis_stop(); printf("STOP\n"); break;

        case '0': case '1': case '2': case '3': case '4':
        case '5': case '6': case '7': case '8': case '9': {
            int sign = (g_throttle < 0) ? -1 : 1;
            g_throttle = sign * (ch - '0') * 100;
            chassis_apply();
            printf("throttle = %d\n", g_throttle);
            break;
        }

        case 'L': chassis_stop(); g_side = 1; printf("MODE: 仅左侧\n"); break;
        case 'R': chassis_stop(); g_side = 2; printf("MODE: 仅右侧\n"); break;
        case 'A': chassis_stop(); g_side = 0; printf("MODE: 双侧\n"); break;

        case 't': self_test(); break;

        case 'y': {
            printf("\n闭环自测：目标 100 / 200 / 300 rpm，各 4 秒\n");
            for (int r = 100; r <= 300; r += 100) {
                g_test_rpm = r;
                printf("  目标 %d rpm\n", r);
                vTaskDelay(pdMS_TO_TICKS(4000));
            }
            g_test_rpm = 0;
            chassis_stop();
            printf("  自测结束\n\n");
            break;
        }

        case 'w': {
            wifi_scan_config_t sc = {0};
            esp_err_t serr = esp_wifi_scan_start(&sc, true);
            if (serr == ESP_OK) {
                uint16_t n = 0;
                esp_wifi_scan_get_ap_num(&n);
                if (n > 24) n = 24;
                wifi_ap_record_t *list = calloc(n ? n : 1, sizeof(wifi_ap_record_t));
                uint16_t got = n;
                esp_wifi_scan_get_ap_records(&got, list);
                printf("\n扫描到 %d 个：\n", got);
                for (int i = 0; i < got; i++) {
                    printf("  ch%2d rssi%4d len=%2d [%s]\n",
                           list[i].primary, list[i].rssi,
                           (int)strlen((char *)list[i].ssid),
                           (char *)list[i].ssid);
                }
                printf("\n");
                free(list);
            } else {
                printf("扫描失败 err=%d (%s)\n", serr, esp_err_to_name(serr));
            }
            break;
        }

        case 'p':
            printf("PIN A16=%d B21=%d | RA2=%d RB47=%d\n",
                   gpio_get_level(PIN_L_ENC_A), gpio_get_level(PIN_L_ENC_B),
                   gpio_get_level(PIN_R_ENC_A), gpio_get_level(PIN_R_ENC_B));
            break;

        case 'e':
            printf("encoder total: L=%ld  R=%ld\n",
                   (long)g_enc_l.total, (long)g_enc_r.total);
            break;

        case '\r': case '\n': case ' ': break;

        default:
            /* 忽略未知字符，避免误触发 */
            break;
        }
    }
}


/* ================= 速度闭环（前馈 + PI） ================= */

static int pi_step(pi_t *p, float target, float meas)
{
    if (target == 0.0f) {
        p->integ = 0.0f;
        p->last_sign = 0;
        return 0;
    }
    int sign = (target > 0) ? 1 : -1;
    if (sign != p->last_sign) { p->integ = 0.0f; p->last_sign = sign; }

    float err = target - meas;
    p->integ += err * 0.01f;                 /* 100 Hz */
    if (p->integ >  INT_LIMIT) p->integ =  INT_LIMIT;
    if (p->integ < -INT_LIMIT) p->integ = -INT_LIMIT;

    /* 前馈：按"目标转速/最高转速"直接给个底数，PI 只补残差。
       纯 PI 也能收敛，但有前馈时起步和变速都稳得多。 */
    float ff  = target / MAX_RPM * 1000.0f;
    float out = ff + KP_RPM * err + KI_RPM * p->integ;

    if (out >  1000.0f) out =  1000.0f;
    if (out < -1000.0f) out = -1000.0f;
    return (int)out;
}

/* ================= 电池电压 ================= */

static void battery_init(void)
{
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &g_adc1));

    adc_oneshot_chan_cfg_t ccfg = {
        .atten    = BAT_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc1, BAT_ADC_CH, &ccfg));

    adc_cali_curve_fitting_config_t cal = {
        .unit_id  = ADC_UNIT_1,
        .atten    = BAT_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cal, &g_adc_cali) != ESP_OK) {
        ESP_LOGW(TAG, "ADC 校准不可用，用原始值估算");
        g_adc_cali = NULL;
    }
}

/* 读一次，8 次平均。返回电池端电压 mV */
static uint16_t battery_read_mv(void)
{
    int sum = 0, ok = 0, last_raw = 0;
    esp_err_t lasterr = ESP_OK;
    for (int i = 0; i < 8; i++) {
        int raw = 0;
        lasterr = adc_oneshot_read(g_adc1, BAT_ADC_CH, &raw);
        if (lasterr == ESP_OK) {
            sum += raw;
            last_raw = raw;
            ok++;
        }
    }
    static int dbg = 0;
    if (dbg < 3 && ok > 0) {
        printf("[BAT] raw=%d cali=%p\n", last_raw, (void *)g_adc_cali);
        dbg++;
    }
    if (ok == 0) {
        printf("[BAT] 读取失败 err=%d\n", lasterr);
        return 0;
    }
    int raw_avg = sum / ok;

    int mv = 0;
    esp_err_t cerr = ESP_FAIL;
    if (g_adc_cali) {
        cerr = adc_cali_raw_to_voltage(g_adc_cali, raw_avg, &mv);
    }
    if (!g_adc_cali || cerr != ESP_OK) {
        mv = raw_avg * BAT_FS_MV / 4095;  /* 校准不可用时退回粗略换算 */
    }
    return (uint16_t)((int32_t)mv * BAT_DIV_NUM / BAT_DIV_DEN);
}

/* ================= WiFi / UDP 遥控 ================= */

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static uint16_t crc16_ccitt(const uint8_t *d, int n)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* 下行 12 字节：0xA5,0x5A,ver,flags,seq(2),throttle(2),steer(2),crc(2) */
static void handle_packet(const uint8_t *p, int n, int src)
{
    if (n != 12 || p[0] != 0xA5 || p[1] != 0x5A || p[2] != 1) {
        g_loss_count++;
        return;
    }
    uint16_t crc = (uint16_t)(p[10] | (p[11] << 8));
    if (crc != crc16_ccitt(p, 10)) {
        g_loss_count++;
        return;
    }
    int16_t thr = (int16_t)(p[6] | (p[7] << 8));
    int16_t st  = (int16_t)(p[8] | (p[9] << 8));

    g_rc.throttle = clamp1000(thr);
    g_rc.steer    = clamp1000(st);
    g_rc.armed    = (p[3] & 0x01) != 0;
    g_rc.seq      = (uint16_t)(p[4] | (p[5] << 8));
    g_rc.last_ms  = now_ms();
    g_pkt_count++;
    if (src == 1) g_serial_ms = g_rc.last_ms;
}

/* 上行 16 字节：'R','T',ver,state,rssi(1),seq(1),vbat(2),rpmL(2),rpmR(2),loss(2),crc(2)
   rssi = AP 侧看到的客户端信号强度，int8，0 表示没有客户端。放这里是为了让地面站不扫描
   也能看到信号——macOS 每扫一次信道就丢一串包，测距时等于自己打自己。 */
static void build_telemetry(uint8_t *b)
{
    b[0] = 0xA5; b[1] = 0x5B; b[2] = 1;
    b[3] = g_net_mode ? ((now_ms() - g_rc.last_ms < CMD_TIMEOUT_MS) ? 2 : 1) : 0;

    int rssi = ap_client_rssi();
    b[4] = (uint8_t)(int8_t)(rssi < -127 ? -127 : rssi);
    b[5] = (uint8_t)(g_tx_seq++ & 0xFF);

    uint16_t vbat = g_vbat_mv;
    b[6] = vbat & 0xFF; b[7] = vbat >> 8;

    int16_t rl = (int16_t)g_rpm_l, rr = (int16_t)g_rpm_r;
    b[8]  = rl & 0xFF; b[9]  = (rl >> 8) & 0xFF;
    b[10] = rr & 0xFF; b[11] = (rr >> 8) & 0xFF;

    uint16_t loss = (uint16_t)(g_loss_count & 0xFFFF);
    b[12] = loss & 0xFF; b[13] = loss >> 8;

    uint16_t crc = crc16_ccitt(b, 14);
    b[14] = crc & 0xFF; b[15] = crc >> 8;
}

static void wifi_evt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        /* 15/204 握手超时（多半密码错）  201 找不到 AP  205 连接失败 */
        ESP_LOGW(TAG, "STA 断开 reason=%d", e->reason);
    }
}

static void net_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket 创建失败");
        vTaskDelete(NULL);
        return;
    }
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind 失败");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP 监听 %d", UDP_PORT);

    uint8_t buf[64];
    uint32_t last_tx = 0;

    while (1) {
        struct sockaddr_in src;
        socklen_t slen = sizeof(src);
        int n = recvfrom(sock, buf, sizeof(buf), MSG_DONTWAIT,
                         (struct sockaddr *)&src, &slen);
        if (n > 0) {
            handle_packet(buf, n, 2);
            g_peer = src;
            g_have_peer = true;
        }

        uint32_t t = now_ms();
        if (t - last_tx >= 100) {
            last_tx = t;
            uint8_t tb[16];
            build_telemetry(tb);
            if (g_have_peer) {
                sendto(sock, tb, sizeof(tb), 0, (struct sockaddr *)&g_peer, sizeof(g_peer));
            }
            if (g_serial_ms != 0 && t - g_serial_ms < 2000) {
                fwrite(tb, 1, sizeof(tb), stdout);
                fflush(stdout);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/* 100 Hz 控制环：失效保护 -> 斜坡限速 -> 混控 */
static void control_task(void *arg)
{
    (void)arg;

    int cur_t = 0, cur_s = 0;
    bool was_net = false;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));

        /* 编码器：10 ms 窗口 */
        int el = encoder_poll(&g_enc_l);
        int er = encoder_poll(&g_enc_r);

        static int bat_tick = 0;
        if (++bat_tick >= 10) {          /* 10 Hz 够用 */
            bat_tick = 0;
            g_vbat_mv = battery_read_mv();
        }
        g_rpm_l = (float)el * 100.0f * 60.0f / (float)ENC_COUNTS_PER_REV;
        g_rpm_r = (float)er * 100.0f * 60.0f / (float)ENC_COUNTS_PER_REV;

        if (g_test_rpm > 0) {
            float tr = (float)g_test_rpm;
            motor_set(&g_left,  pi_step(&g_pi_l, tr, g_rpm_l));
            motor_set(&g_right, pi_step(&g_pi_r, tr, g_rpm_r));
            continue;
        }

        uint32_t age = now_ms() - g_rc.last_ms;
        bool net = (g_rc.last_ms != 0) && (age < NET_LOST_MS);

        if (net != was_net) {
            was_net = net;
            g_net_mode = net;
            if (!net) {                 /* 刚掉线：目标清零 */
                printf("\n[NET] 链路丢失，输出归零\n");
            } else {
                printf("\n[NET] 进入网络控制\n");
            }
        }

        if (net) {
            int tgt_t = 0, tgt_s = 0;
            if (age < CMD_TIMEOUT_MS && g_rc.armed) {
                tgt_t = g_rc.throttle;
                tgt_s = g_rc.steer;
            }
            /* 低电保护：3S 锂电，3.3V/节=9.9V 限速，3.2V/节=9.6V 停 */
            if (g_vbat_mv > 5000) {          /* 低于 5V 视为没接分压，不做保护 */
                if (g_vbat_mv < 9600) {
                    tgt_t = 0;
                    tgt_s = 0;
                } else if (g_vbat_mv < 9900) {
                    tgt_t = tgt_t / 2;
                    tgt_s = tgt_s / 2;
                }
            }

            /* 斜坡：每 10 ms 最多走 20 -> 0 到满 0.5 秒 */
            if (tgt_t > cur_t + 20) tgt_t = cur_t + 20;
            if (tgt_t < cur_t - 20) tgt_t = cur_t - 20;
            if (tgt_s > cur_s + 20) tgt_s = cur_s + 20;
            if (tgt_s < cur_s - 20) tgt_s = cur_s - 20;

            cur_t = tgt_t;
            cur_s = tgt_s;
            g_throttle = cur_t;
            g_steer    = cur_s;

            /* 摇杆 -> 两侧目标转速 -> 各自闭环 */
            int op_l = clamp1000(cur_t + cur_s);
            int op_r = clamp1000(cur_t - cur_s);
            if (g_side == 1) op_r = 0;
            if (g_side == 2) op_l = 0;

            float tl = op_l * (MAX_RPM / 1000.0f);
            float tr = op_r * (MAX_RPM / 1000.0f);
            motor_set(&g_left,  pi_step(&g_pi_l, tl, g_rpm_l));
            motor_set(&g_right, pi_step(&g_pi_r, tr, g_rpm_r));
        } else {
            /* 串口模式：控制权在 console_task，这里只维持斜坡变量同步 */
            cur_t = g_throttle;
            cur_s = g_steer;
        }
    }
}

/* ================= 入口 ================= */
void app_main(void)
{
    ESP_LOGI(TAG, "r3car start");

    /* --- MCPWM --- */
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RES_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = PWM_PERIOD_TICKS,
    };
    mcpwm_timer_handle_t timer = NULL;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    motor_init(&g_left,  PIN_L_FWD, PIN_L_REV, timer);
    motor_init(&g_right, PIN_R_FWD, PIN_R_REV, timer);

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    /* --- 编码器 --- */
    battery_init();

    encoder_init(&g_enc_l, PIN_L_ENC_A, PIN_L_ENC_B, 0);
    encoder_init(&g_enc_r, PIN_R_ENC_A, PIN_R_ENC_B, 1);


    /* --- WiFi --- */
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* --- 任务：必须在 lwIP 初始化之后，net_task 一开机就 socket() --- */
    xTaskCreatePinnedToCore(control_task,   "control",   4096, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(net_task,       "net",       4096, NULL,  6, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL,  4, NULL, 0);
    xTaskCreatePinnedToCore(console_task,   "console",   4096, NULL,  3, NULL, 0);

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    const char *sta_ssid = STA_SSID;
    bool use_sta = (strlen(sta_ssid) > 0);

    if (use_sta) {
        /* ---- STA：连家里路由器 ---- */
        esp_netif_create_default_wifi_sta();
        wifi_config_t sta_cfg = {0};
        strlcpy((char *)sta_cfg.sta.ssid, STA_SSID, sizeof(sta_cfg.sta.ssid));
        strlcpy((char *)sta_cfg.sta.password, STA_PASS, sizeof(sta_cfg.sta.password));
        /* 默认是 FAST_SCAN，扫到同名就停；有的路由器会漏，改成全信道按信号选 */
        sta_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
        sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
        sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                          &wifi_evt, NULL, NULL));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGI(TAG, "STA 目标 len=%d [%s]", (int)strlen(STA_SSID), STA_SSID);

        /* 最多等 8 秒，连不上就退回 AP */
        bool ok = false;
        for (int i = 0; i < 16; i++) {
            vTaskDelay(pdMS_TO_TICKS(500));
            wifi_ap_record_t apinfo;
            if (esp_wifi_sta_get_ap_info(&apinfo) == ESP_OK) { ok = true; break; }
        }

        if (ok) {
            esp_netif_ip_info_t ip;
            esp_netif_t *nif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
            if (nif && esp_netif_get_ip_info(nif, &ip) == ESP_OK) {
                ESP_LOGW(TAG, "STA 已连上，本车 IP " IPSTR, IP2STR(&ip.ip));
                ESP_LOGW(TAG, "Mac 上运行 ground/drive.py --host 自动发现即可");
            }
            ESP_LOGI(TAG, "UDP 监听 %d", UDP_PORT);
        } else {
            ESP_LOGW(TAG, "STA 连接超时，改用 AP 模式");
            use_sta = false;
            esp_wifi_stop();
        }
    }

    if (!use_sta) {
        esp_netif_create_default_wifi_ap();
        wifi_config_t ap_cfg = {0};
        strlcpy((char *)ap_cfg.ap.ssid, WIFI_SSID, sizeof(ap_cfg.ap.ssid));
        ap_cfg.ap.ssid_len = strlen(WIFI_SSID);
        strlcpy((char *)ap_cfg.ap.password, WIFI_PASS, sizeof(ap_cfg.ap.password));
        ap_cfg.ap.channel = 11;   /* 家里 2.4G 挤在 ch6（HUAWEI-805 实测 -14 dBm），改 11 避开 */
        ap_cfg.ap.max_connection = 4;
        ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

        /* APSTA 而不是纯 AP：纯 AP 模式下驱动拒绝扫描，诊断时看不见周边 */
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGI(TAG, "AP \"%s\"  pass \"%s\"  ch %d   IP 192.168.4.1  UDP %d",
                 WIFI_SSID, WIFI_PASS, ap_cfg.ap.channel, UDP_PORT);
    }


    ESP_LOGI(TAG, "ready. outputs are 0 until a command arrives.");
}
