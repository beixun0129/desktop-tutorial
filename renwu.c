#include "renwu.h"
#include "HC-SR04.h"

#define WIFI_SSID       "zbcc"
#define WIFI_PWD        "ghs123456"

#define BROKER_IP       "gz-3-mqtt.iot-api.com"
#define BROKER_PORT     1883
#define MQTT_CLIENTID   "STM32_CB_001"
#define MQTT_USER       "39mcki471tkbirkc"
#define MQTT_PASS       "RFs4kqpX9d"

#define TOPIC_PUB       "attributes"
#define TOPIC_SUB       "attributes/push"


static uint32_t last_relink_ms = 0;
static uint8_t wifi_state = 0;
static uint8_t vision_cmd = VISION_UART_CMD_WALK_NONE;
static uint32_t last_vision_cmd_ms = 0;
static uint32_t vib_until_ms = 0;
static uint8_t emergency_enable = 0;
static uint32_t last_emergency_cfg_pub_ms = 0;
static uint8_t status_pending = 0;
static uint8_t sw520d_state = 0;
static uint32_t sw520d_state_until_ms = 0;
static uint8_t fall_voice_pending = 0;
static uint32_t fall_voice_last_ms = 0;
static uint8_t help_from_fall = 0;
static uint8_t help_from_cloud = 0;
static uint8_t help_stage = 0;
static uint32_t help_next_ms = 0;
static uint32_t help_cloud_next_ms = 0;
static uint8_t fall_recover_pending = 0;
static uint32_t fall_recover_due_ms = 0;
static uint8_t fall_confirm_pending = 0;
static uint32_t fall_confirm_due_ms = 0;
static uint32_t fall_hold_until_ms = 0;
static uint8_t vision_voice_cmd = VISION_UART_CMD_WALK_NONE;
static uint32_t vision_voice_due_ms = 0;
static uint32_t vision_voice_last_ms = 0;
static uint16_t vision_dist_cm = 0;
static uint8_t vision_latched_cmd = VISION_UART_CMD_WALK_NONE;
static uint16_t vision_latched_dist_cm = 0;
static uint32_t vision_latched_until_ms = 0;
static uint32_t sr04_mm[3] = {0, 0, 0};
static uint32_t sr04_ms[3] = {0, 0, 0};
static uint32_t last_sr04_trig_ms = 0;
static uint8_t sr04_trig_ch = 0;
static uint32_t sr04_voice_last_ms = 0;
static uint8_t sr04_voice_last_pos = 0;

#define VISION_VOICE_PERIOD_MS 5000u
#define FALL_VOICE_HOLD_MS 5000u
#define FALL_CONFIRM_MS 5000u
#define FALL_RECOVER_MS 2000u
#define CLOUD_PUB_PERIOD_MS 3000u
#define SR04_DEBUG_ONLY 0u
#define SR04_VOICE_TH_MM 500u
#define SR04_VOICE_PERIOD_MS 1500u
#define HELP_VOICE_PERIOD_MS 6000u

static void oled_pad_16(char line[17])
{
    size_t n = strlen(line);
    if (n > 16) n = 16;
    for (size_t i = n; i < 16; i++) line[i] = ' ';
    line[16] = '\0';
}

static const char *vision_cmd_short(uint8_t cmd)
{
    if (cmd == VISION_UART_CMD_STOP_RED) return "RED";
    if (cmd == VISION_UART_CMD_STOP_CAR) return "CAR";
    if (cmd == VISION_UART_CMD_WALK_GREEN) return "GRN";
    if (cmd == VISION_UART_CMD_WAIT_ZEBRA) return "ZEB";
    if (cmd == VISION_UART_CMD_WAIT_ZEBRA_PERSON) return "ZP";
    return "---";
}

static void oled_render(uint32_t now,
                        uint8_t action,
                        uint8_t vision_effective,
                        uint16_t vision_effective_dist_cm,
                        uint32_t sr04_mm,
                        uint8_t sr04_valid,
                        uint8_t sr04_pos)
{
    char line0[17];
    char line2[17];
    char line4[17];
    char line6[17];

    const char *emg = (emergency_enable || sw520d_state) ? "ON" : "OFF";
    const char *wifi = (wifi_state == 2) ? "ON" : ((wifi_state == 3) ? "ERR" : "OFF");
    (void)snprintf(line0, sizeof(line0), "EMG:%s WIFI:%s", emg, wifi);
    oled_pad_16(line0);

    if (sw520d_state)
    {
        (void)snprintf(line2, sizeof(line2), "FALL! CALL HELP");
    }
    else if (fall_confirm_pending && fall_confirm_due_ms != 0 && (int32_t)(now - fall_confirm_due_ms) < 0)
    {
        uint32_t left_ms = fall_confirm_due_ms - now;
        uint32_t left_s = (left_ms + 999u) / 1000u;
        if (left_s > 99u) left_s = 99u;
        (void)snprintf(line2, sizeof(line2), "FALL:WAIT %02us", (unsigned)left_s);
    }
    else if (vib_until_ms != 0 && (int32_t)(now - vib_until_ms) < 0)
    {
        (void)snprintf(line2, sizeof(line2), "VIBRATION TRIG");
    }
    else
    {
        (void)snprintf(line2, sizeof(line2), "FALL:NORMAL");
    }
    oled_pad_16(line2);

    if (action == 1)
    {
        if (vision_effective == VISION_UART_CMD_STOP_RED)
        {
            (void)snprintf(line4, sizeof(line4), "ACT:STOP RED");
        }
        else if (vision_effective == VISION_UART_CMD_STOP_CAR)
        {
            if (vision_effective_dist_cm != 0)
            {
                (void)snprintf(line4, sizeof(line4), "ACT:CAR STOP %ucm", (unsigned)vision_effective_dist_cm);
            }
            else
            {
                (void)snprintf(line4, sizeof(line4), "ACT:CAR STOP");
            }
        }
        else
        {
            (void)snprintf(line4, sizeof(line4), "ACT:STOP");
        }
    }
    else if (action == 2)
    {
        if (vision_effective_dist_cm != 0)
        {
            (void)snprintf(line4, sizeof(line4), "ACT:WAIT ZEB %ucm", (unsigned)vision_effective_dist_cm);
        }
        else
        {
            (void)snprintf(line4, sizeof(line4), "ACT:WAIT ZEBRA");
        }
    }
    else
    {
        if (vision_effective == VISION_UART_CMD_WALK_GREEN)
        {
            (void)snprintf(line4, sizeof(line4), "ACT:WALK GREEN");
        }
        else
        {
            (void)snprintf(line4, sizeof(line4), "ACT:WALK");
        }
    }
    oled_pad_16(line4);

    if (vision_effective != VISION_UART_CMD_WALK_NONE)
    {
        const char *v = vision_cmd_short(vision_effective);
        if (vision_effective == VISION_UART_CMD_STOP_CAR ||
            vision_effective == VISION_UART_CMD_WAIT_ZEBRA ||
            vision_effective == VISION_UART_CMD_WAIT_ZEBRA_PERSON)
        {
            if (vision_effective_dist_cm != 0)
            {
                (void)snprintf(line6, sizeof(line6), "VIS:%s %ucm", v, (unsigned)vision_effective_dist_cm);
            }
            else
            {
                (void)snprintf(line6, sizeof(line6), "VIS:%s ----", v);
            }
        }
        else
        {
            (void)snprintf(line6, sizeof(line6), "VIS:%s", v);
        }
    }
    else
    {
        if (sr04_valid)
        {
            char d = 'F';
            if (sr04_pos == 1) d = 'L';
            else if (sr04_pos == 3) d = 'R';
            uint32_t cm = (sr04_mm + 5u) / 10u;
            if (cm > 999u) cm = 999u;
            (void)snprintf(line6, sizeof(line6), "SR04:%c%3ucm", d, (unsigned)cm);
        }
        else
        {
            (void)snprintf(line6, sizeof(line6), "SR04:----cm");
        }
    }
    oled_pad_16(line6);

    OLED_ShowString(0, 0, line0, 16);
    OLED_ShowString(0, 2, line2, 16);
    OLED_ShowString(0, 4, line4, 16);
    OLED_ShowString(0, 6, line6, 16);
}

void setup(void)
{
#if SR04_DEBUG_ONLY
    OLED_Init();
    OLED_Clear();
    HC_SR04_Init();
    OLED_ShowString(0, 0, "SR04 DEBUG    ", 16);
    OLED_ShowString(0, 2, "TRIG:PB1      ", 16);
    OLED_ShowString(0, 4, "ECHO:PA6      ", 16);
    OLED_ShowString(0, 6, "WAIT...       ", 16);
#else
    CL1302_Init();
    CL1302_WriteData(CL1302_CMD_VOL_MAX, CL1302_TYPE_1);
    HAL_Delay(50);
    CL1302_WriteData(CL1302_CMD_VOL_MAX, CL1302_TYPE_1);
    SW520D_Init();
    VisionUart_Start();
    HC_SR04_Init();
    OLED_Init();
    OLED_Clear();

    OLED_ShowString(0, 0, "App Mode...", 16);
    OLED_ShowString(0, 2, "Resetting...", 16);
    atk_mb026_hw_reset();

    if (atk_mb026_init(115200) == 0)
    {
        atk_mb026_set_mode(1);
        atk_mb026_sw_reset();
        HAL_Delay(500);
        atk_mb026_ate_config(0);

        OLED_ShowString(0, 2, "Joining WiFi...", 16);
        if (atk_mb026_join_ap(WIFI_SSID, WIFI_PWD) == 0)
        {
            OLED_ShowString(0, 4, "MQTT Config...", 16);
            if (atk_mb026_mqtt_cfg(MQTT_CLIENTID, MQTT_USER, MQTT_PASS) != 0)
            {
                OLED_ShowString(0, 6, "Cfg Fail     ", 16);
                wifi_state = 3;
            }
            else
            {
                OLED_ShowString(0, 4, "MQTT Connect ", 16);
                if (atk_mb026_mqtt_connect(BROKER_IP, BROKER_PORT) != 0)
                {
                    OLED_ShowString(0, 6, "Conn Fail!   ", 16);
                    wifi_state = 3;
                }
                else
                {
                    atk_mb026_mqtt_sub(TOPIC_SUB);
                    HAL_Delay(500);
                    OLED_ShowString(0, 6, "Online!      ", 16);
                    wifi_state = 2;
                }
            }
        }
        else
        {
            OLED_ShowString(0, 4, "WiFi Fail    ", 16);
            wifi_state = 3;
        }
    }
    else
    {
        OLED_ShowString(0, 0, "Mod Error    ", 16);
        wifi_state = 3;
    }

    HAL_Delay(1000);
    OLED_Clear();
#endif
}

void loop(void)
{
#if SR04_DEBUG_ONLY
    uint32_t now = HAL_GetTick();
    uint32_t mm;
    if (HC_SR04_TakeDistanceMm(&mm))
    {
        sr04_last_mm = mm;
        sr04_last_ms = now;
    }
    if ((now - last_sr04_trig_ms) >= 80u)
    {
        if (HC_SR04_IsBusy() == 0 && HC_SR04_Trigger())
        {
            last_sr04_trig_ms = now;
        }
    }
    {
        static uint32_t last_oled_ms = 0;
        if ((now - last_oled_ms) >= 200u)
        {
            last_oled_ms = now;
            char l0[17];
            char l2[17];
            char l4[17];
            char l6[17];

            (void)snprintf(l0, sizeof(l0), "SR04 PB1->PA6");
            oled_pad_16(l0);

            (void)snprintf(l2, sizeof(l2), "B:%u S:%u I:%lu",
                           (unsigned)HC_SR04_IsBusy(),
                           (unsigned)HC_SR04_LastState,
                           (unsigned long)HC_SR04_IrqCount);
            oled_pad_16(l2);

            if (sr04_last_ms != 0 && (now - sr04_last_ms) <= 800u)
            {
                (void)snprintf(l4, sizeof(l4), "MM:%4lu US:%5lu",
                               (unsigned long)sr04_last_mm,
                               (unsigned long)HC_SR04_LastPulseUs);
            }
            else
            {
                (void)snprintf(l4, sizeof(l4), "MM:---- US:%5lu",
                               (unsigned long)HC_SR04_LastPulseUs);
            }
            oled_pad_16(l4);

            (void)snprintf(l6, sizeof(l6), "D:%5lu A:%4lu",
                           (unsigned long)HC_SR04_LastDiffTicks,
                           (unsigned long)(now - HC_SR04_LastEventMs));
            oled_pad_16(l6);

            OLED_ShowString(0, 0, l0, 16);
            OLED_ShowString(0, 2, l2, 16);
            OLED_ShowString(0, 4, l4, 16);
            OLED_ShowString(0, 6, l6, 16);
        }
    }
#else
    CL1302_Tick();
    {
        uint8_t cmd;
        uint16_t dist;
        if (VisionUart_TakeFrame(&cmd, &dist))
        {
            vision_cmd = cmd;
            vision_dist_cm = dist;
            uint32_t t = HAL_GetTick();
            last_vision_cmd_ms = t;
            if (cmd != VISION_UART_CMD_WALK_NONE)
            {
                vision_latched_cmd = cmd;
                vision_latched_dist_cm = dist;
                vision_latched_until_ms = t + 800u;
            }
        }
    }
    uint32_t now = HAL_GetTick();
    uint32_t sr04_best_mm = 0;
    uint8_t sr04_best_pos = 0;
    uint8_t sr04_best_valid = 0;
    {
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            uint32_t mm;
            if (HC_SR04_TakeDistanceMmCh(ch, &mm))
            {
                sr04_mm[ch] = mm;
                sr04_ms[ch] = now;
            }
        }

        if ((now - last_sr04_trig_ms) >= 80u)
        {
            if (HC_SR04_TriggerCh(sr04_trig_ch))
            {
                last_sr04_trig_ms = now;
                sr04_trig_ch++;
                if (sr04_trig_ch >= 3) sr04_trig_ch = 0;
            }
        }

        for (uint8_t ch = 0; ch < 3; ch++)
        {
            if (sr04_ms[ch] == 0) continue;
            if ((now - sr04_ms[ch]) > 800u) continue;
            if (sr04_mm[ch] == 0) continue;
            if (sr04_best_valid == 0 || sr04_mm[ch] < sr04_best_mm)
            {
                sr04_best_valid = 1;
                sr04_best_mm = sr04_mm[ch];
                sr04_best_pos = (uint8_t)(ch + 1u);
            }
        }
    }
    uint8_t vision_effective = VISION_UART_CMD_WALK_NONE;
    uint16_t vision_effective_dist_cm = 0;
    if (vision_latched_cmd != VISION_UART_CMD_WALK_NONE && (int32_t)(now - vision_latched_until_ms) < 0)
    {
        vision_effective = vision_latched_cmd;
        vision_effective_dist_cm = vision_latched_dist_cm;
    }
    else
    {
        vision_latched_cmd = VISION_UART_CMD_WALK_NONE;
        vision_latched_until_ms = 0;
        vision_effective = vision_cmd;
        vision_effective_dist_cm = vision_dist_cm;
    }
    if (last_vision_cmd_ms == 0 || (now - last_vision_cmd_ms) > 300u)
    {
        vision_effective = VISION_UART_CMD_WALK_NONE;
        vision_effective_dist_cm = 0;
    }
    if (vision_effective != vision_voice_cmd)
    {
        vision_voice_cmd = vision_effective;
        if (vision_effective == VISION_UART_CMD_WALK_NONE)
        {
            vision_voice_due_ms = 0;
        }
        else
        {
            if (vision_voice_last_ms == 0 || (now - vision_voice_last_ms) >= VISION_VOICE_PERIOD_MS)
            {
                vision_voice_due_ms = now;
            }
            else
            {
                vision_voice_due_ms = vision_voice_last_ms + VISION_VOICE_PERIOD_MS;
            }
        }
    }

    {
        uint32_t t;
        if (SW520D_TakeEvent(&t))
        {
            vib_until_ms = now + 2000u;
            if (sw520d_state == 0 &&
                fall_confirm_pending == 0 &&
                (fall_voice_last_ms == 0 || (now - fall_voice_last_ms) >= FALL_VOICE_HOLD_MS))
            {
                fall_confirm_pending = 1;
                fall_confirm_due_ms = now + FALL_CONFIRM_MS;
                SW520D_BlockFor(FALL_CONFIRM_MS);
            }
        }
        if (fall_confirm_pending && fall_confirm_due_ms != 0 && (int32_t)(now - fall_confirm_due_ms) >= 0)
        {
            fall_confirm_pending = 0;
            fall_confirm_due_ms = 0;
            if (HAL_GPIO_ReadPin(SW520D_GPIO_Port, SW520D_Pin) == GPIO_PIN_RESET)
            {
                SW520D_BlockFor(FALL_VOICE_HOLD_MS);
                sw520d_state = 1;
                sw520d_state_until_ms = 0;
                status_pending = 1;
                fall_voice_pending = 1;
                help_from_fall = 1;
                help_stage = 1;
                help_next_ms = 0;
                fall_recover_pending = 0;
                fall_recover_due_ms = 0;
                fall_hold_until_ms = now + FALL_VOICE_HOLD_MS;
                if (vision_voice_cmd != VISION_UART_CMD_WALK_NONE)
                {
                    uint32_t min_due = fall_hold_until_ms;
                    if ((int32_t)(min_due - vision_voice_due_ms) > 0) vision_voice_due_ms = min_due;
                }
            }
        }
        if (sw520d_state)
        {
            if (HAL_GPIO_ReadPin(SW520D_GPIO_Port, SW520D_Pin) == GPIO_PIN_SET)
            {
                if (fall_recover_pending == 0)
                {
                    fall_recover_pending = 1;
                    fall_recover_due_ms = now + FALL_RECOVER_MS;
                }
                else if (fall_recover_due_ms != 0 && (int32_t)(now - fall_recover_due_ms) >= 0)
                {
                    sw520d_state = 0;
                    sw520d_state_until_ms = 0;
                    status_pending = 1;
                    help_from_fall = 0;
                    help_stage = 0;
                    help_next_ms = 0;
                    fall_recover_pending = 0;
                    fall_recover_due_ms = 0;
                }
            }
            else
            {
                fall_recover_pending = 0;
                fall_recover_due_ms = 0;
            }
        }
    }

    uint8_t action = 0;
    if (vision_effective == VISION_UART_CMD_STOP_RED || vision_effective == VISION_UART_CMD_STOP_CAR)
    {
        action = 1;
    }
    else if (vision_effective == VISION_UART_CMD_WAIT_ZEBRA || vision_effective == VISION_UART_CMD_WAIT_ZEBRA_PERSON)
    {
        action = 2;
    }
    else
    {
        action = 0;
    }

    if (fall_voice_pending)
    {
        if (CL1302_IsBusy() == 0)
        {
            CL1302_VoiceBroadcastFall();
            fall_voice_pending = 0;
            fall_voice_last_ms = now;
            if (help_from_fall)
            {
                help_stage = 2;
                help_next_ms = 0;
            }
        }
    }

    if (help_from_fall)
    {
        if (help_stage == 0)
        {
            help_stage = 1;
            help_next_ms = 0;
        }
    }
    else
    {
        help_stage = 0;
        help_next_ms = 0;
    }

    if (help_stage == 2)
    {
        if (CL1302_IsBusy() == 0 && (help_next_ms == 0 || (int32_t)(now - help_next_ms) >= 0))
        {
            CL1302_VoiceBroadcastHelp();
            help_next_ms = now + HELP_VOICE_PERIOD_MS;
        }
    }

    if (help_from_cloud && !help_from_fall)
    {
        if (CL1302_IsBusy() == 0 && (help_cloud_next_ms == 0 || (int32_t)(now - help_cloud_next_ms) >= 0))
        {
            CL1302_VoiceBroadcastHelp();
            help_cloud_next_ms = now + HELP_VOICE_PERIOD_MS;
        }
    }
    else if (!help_from_cloud)
    {
        help_cloud_next_ms = 0;
    }

    if (help_stage == 0 && !help_from_cloud &&
        vision_voice_cmd != VISION_UART_CMD_WALK_NONE &&
        vision_voice_due_ms != 0 &&
        (fall_hold_until_ms == 0 || (int32_t)(now - fall_hold_until_ms) >= 0) &&
        (int32_t)(now - vision_voice_due_ms) >= 0)
    {
        if (CL1302_IsBusy() == 0)
        {
            uint16_t dist = 0;
            if (vision_voice_cmd == VISION_UART_CMD_STOP_CAR ||
                vision_voice_cmd == VISION_UART_CMD_WAIT_ZEBRA ||
                vision_voice_cmd == VISION_UART_CMD_WAIT_ZEBRA_PERSON)
            {
                dist = vision_effective_dist_cm;
            }
            CL1302_VoiceBroadcastVision(vision_voice_cmd, dist);
            vision_voice_last_ms = now;
            vision_voice_due_ms = now + VISION_VOICE_PERIOD_MS;
        }
    }

    if (help_stage == 0 && !help_from_cloud &&
        (fall_hold_until_ms == 0 || (int32_t)(now - fall_hold_until_ms) >= 0) &&
        sr04_best_valid &&
        sr04_best_mm <= SR04_VOICE_TH_MM)
    {
        if (CL1302_IsBusy() == 0 &&
            (sr04_voice_last_ms == 0 ||
             (now - sr04_voice_last_ms) >= SR04_VOICE_PERIOD_MS ||
             sr04_voice_last_pos != sr04_best_pos))
        {
            uint16_t cm = (uint16_t)((sr04_best_mm + 5u) / 10u);
            CL1302_VoiceBroadcastUltrasonic(sr04_best_pos, cm);
            sr04_voice_last_ms = now;
            sr04_voice_last_pos = sr04_best_pos;
        }
    }
    else
    {
        sr04_voice_last_pos = 0;
    }

    {
        static uint32_t last_oled_ms = 0;
        if ((now - last_oled_ms) >= 200u)
        {
            last_oled_ms = now;
            oled_render(now, action, vision_effective, vision_effective_dist_cm, sr04_best_mm, sr04_best_valid, sr04_best_pos);
        }
    }

    if (wifi_state == 2)
    {
        if (last_emergency_cfg_pub_ms == 0 || (now - last_emergency_cfg_pub_ms) >= CLOUD_PUB_PERIOD_MS)
        {
            char json_body[128];
            last_emergency_cfg_pub_ms = now;
            memset(json_body, 0, sizeof(json_body));
            (void)snprintf(json_body, sizeof(json_body), "{\"emergency_enable\":%s,\"sw520d\":%s}",
                           (emergency_enable || sw520d_state) ? "true" : "false",
                           sw520d_state ? "true" : "false");
            if (atk_mb026_mqtt_pub(TOPIC_PUB, json_body) == 0)
            {
                status_pending = 0;
            }
        }

        {
            uint8_t *recv_data = atk_mb026_uart_rx_get_frame();
            if (recv_data != NULL)
            {
                if (strstr((char *)recv_data, "+MQTTSUBRECV") != NULL)
                {
                    char *p = strstr((char *)recv_data, "emergency_enable");
                    if (p == NULL) p = strstr((char *)recv_data, "emergencyEnabled");
                    if (p == NULL) p = strstr((char *)recv_data, "sw520d_enable");
                    if (p == NULL) p = strstr((char *)recv_data, "sw520dEnable");
                    if (p != NULL)
                    {
                        char *c = strchr(p, ':');
                        if (c != NULL)
                        {
                            uint8_t new_enable = emergency_enable;
                            uint8_t ok = 0;
                            if (strstr(c, "true") != NULL)
                            {
                                new_enable = 1;
                                ok = 1;
                            }
                            else if (strstr(c, "false") != NULL)
                            {
                                new_enable = 0;
                                ok = 1;
                            }
                            else if (strstr(c, ":1") != NULL)
                            {
                                new_enable = 1;
                                ok = 1;
                            }
                            else if (strstr(c, ":0") != NULL)
                            {
                                new_enable = 0;
                                ok = 1;
                            }

                            if (ok && new_enable != emergency_enable)
                            {
                                emergency_enable = new_enable;
                                help_from_cloud = new_enable;
                                help_cloud_next_ms = 0;
                                status_pending = 1;
                            }
                        }
                    }
                }
                else
                {
                }

                atk_mb026_uart_rx_restart();
            }
        }
    }
    else if (wifi_state == 3)
    {
        if (now - last_relink_ms > 10000)
        {
            OLED_ShowString(0, 0, "Re-Linking...", 16);
            if (atk_mb026_join_ap(WIFI_SSID, WIFI_PWD) == 0)
            {
                if (atk_mb026_mqtt_cfg(MQTT_CLIENTID, MQTT_USER, MQTT_PASS) == 0 &&
                    atk_mb026_mqtt_connect(BROKER_IP, BROKER_PORT) == 0)
                {
                    atk_mb026_mqtt_sub(TOPIC_SUB);
                    OLED_ShowString(0, 0, "Recovered!   ", 16);
                    wifi_state = 2;
                }
                else
                {
                    OLED_ShowString(0, 0, "MQTT Fail    ", 16);
                }
            }
            else
            {
                atk_mb026_sw_reset();
                OLED_ShowString(0, 0, "WiFi Fail    ", 16);
            }
            last_relink_ms = now;
        }
    }
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
}
