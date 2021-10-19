/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_lighting_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"

#define CID_ESP 0x02E5

extern struct _led_state led_state[3];

static uint8_t dev_uuid[16] = { 0xdd, 0xdd };

/* Configuration Server related context */
static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

/* Generic OnOff Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub, 2 + 3, ROLE_NODE);
static esp_ble_mesh_gen_onoff_srv_t onoff_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

/* Generic Default Transition Time Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(def_trans_time_pub, 2 + 1, ROLE_NODE);
static esp_ble_mesh_gen_def_trans_time_srv_t def_trans_time_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

/* Generic Power OnOff state related context */
static esp_ble_mesh_gen_onpowerup_state_t onpowerup_state;

/* Generic Power OnOff Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(power_onoff_pub, 2 + 1, ROLE_NODE);
static esp_ble_mesh_gen_power_onoff_srv_t power_onoff_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &onpowerup_state,
};

/* Generic Power OnOff Setup Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(power_onoff_setup_pub, 2 + 5, ROLE_NODE);
static esp_ble_mesh_gen_power_onoff_setup_srv_t power_onoff_setup_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &onpowerup_state,
};

/* Generic Level Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub_0, 2 + 5, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server_0 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

/* Light Lightness state related context */
static esp_ble_mesh_light_lightness_state_t lightness_state;

/* Light Lightness Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(lightness_pub, 2 + 5, ROLE_NODE);
static esp_ble_mesh_light_lightness_srv_t lightness_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &lightness_state,
};

/* Light Lightness Setup Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(lightness_setup_pub, 2 + 5, ROLE_NODE);
static esp_ble_mesh_light_lightness_setup_srv_t lightness_setup_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &lightness_state,
};

/* Light HSL state related context */
static esp_ble_mesh_light_hsl_state_t hsl_state;

/* Light HSL Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(hsl_pub, 2 + 9, ROLE_NODE);
static esp_ble_mesh_light_hsl_srv_t hsl_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &hsl_state,
};

/* Light HSL Setup Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(hsl_setup_pub, 2 + 9, ROLE_NODE);
static esp_ble_mesh_light_hsl_setup_srv_t hsl_setup_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &hsl_state,
};

/* Light CTL state related context */
static esp_ble_mesh_light_ctl_state_t ctl_state;

/* Light CTL Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(ctl_pub, 2 + 9, ROLE_NODE);
static esp_ble_mesh_light_ctl_srv_t ctl_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &ctl_state,
};

/* Light CTL Setup Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(ctl_setup_pub, 2 + 6, ROLE_NODE);
static esp_ble_mesh_light_ctl_setup_srv_t ctl_setup_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &ctl_state,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub, &onoff_server),
    ESP_BLE_MESH_MODEL_GEN_DEF_TRANS_TIME_SRV(&def_trans_time_pub, &def_trans_time_server),
    ESP_BLE_MESH_MODEL_GEN_POWER_ONOFF_SRV(&power_onoff_pub, &power_onoff_server),
    ESP_BLE_MESH_MODEL_GEN_POWER_ONOFF_SETUP_SRV(&power_onoff_setup_pub, &power_onoff_setup_server),
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub_0, &level_server_0),
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_SRV(&lightness_pub, &lightness_server),
    ESP_BLE_MESH_MODEL_LIGHT_LIGHTNESS_SETUP_SRV(&lightness_setup_pub, &lightness_setup_server),
    ESP_BLE_MESH_MODEL_LIGHT_HSL_SRV(&hsl_pub, &hsl_server),
    ESP_BLE_MESH_MODEL_LIGHT_HSL_SETUP_SRV(&hsl_setup_pub, &hsl_setup_server),
    ESP_BLE_MESH_MODEL_LIGHT_CTL_SRV(&ctl_pub, &ctl_server),
    ESP_BLE_MESH_MODEL_LIGHT_CTL_SETUP_SRV(&ctl_setup_pub, &ctl_setup_server),
};

/* Generic Level Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub_1, 2 + 5, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server_1 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

/* Light HSL Hue Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(hsl_hue_pub, 2 + 5, ROLE_NODE);
static esp_ble_mesh_light_hsl_hue_srv_t hsl_hue_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &hsl_state,
};
static esp_ble_mesh_model_t hue_models[] = {
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub_1, &level_server_1),
    ESP_BLE_MESH_MODEL_LIGHT_HSL_HUE_SRV(&hsl_hue_pub, &hsl_hue_server),
};

/* Generic Level Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub_2, 2 + 5, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server_2 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

/* Light HSL Saturation Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(hsl_saturation_pub, 2 + 5, ROLE_NODE);
static esp_ble_mesh_light_hsl_sat_srv_t hsl_saturation_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &hsl_state,
};
static esp_ble_mesh_model_t saturation_models[] = {
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub_2, &level_server_2),
    ESP_BLE_MESH_MODEL_LIGHT_HSL_HUE_SRV(&hsl_saturation_pub, &hsl_saturation_server),
};

/* Generic Level Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(level_pub_3, 2 + 5, ROLE_NODE);
static esp_ble_mesh_gen_level_srv_t level_server_3 = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
};

/* Light CTL Temperature Server related context */
ESP_BLE_MESH_MODEL_PUB_DEFINE(ctl_temperature_pub, 2 + 9, ROLE_NODE);
static esp_ble_mesh_light_ctl_temp_srv_t ctl_temperature_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    .state = &ctl_state,
};
static esp_ble_mesh_model_t temperature_models[] = {
    ESP_BLE_MESH_MODEL_GEN_LEVEL_SRV(&level_pub_3, &level_server_3),
    ESP_BLE_MESH_MODEL_LIGHT_CTL_TEMP_SRV(&ctl_temperature_pub, &ctl_temperature_server),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, hue_models, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, saturation_models, ESP_BLE_MESH_MODEL_NONE),
    ESP_BLE_MESH_ELEMENT(0, temperature_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08x", flags, iv_index);
    board_led_operation(LED_G, LED_OFF);
}

static void example_change_led_state(esp_ble_mesh_model_t *model,
                                     esp_ble_mesh_msg_ctx_t *ctx, uint8_t onoff)
{
    uint16_t primary_addr = esp_ble_mesh_get_primary_element_address();
    uint8_t elem_count = esp_ble_mesh_get_element_count();
    struct _led_state *led = NULL;
    uint8_t i;

    if (ESP_BLE_MESH_ADDR_IS_UNICAST(ctx->recv_dst)) {
        for (i = 0; i < elem_count; i++) {
            if (ctx->recv_dst == (primary_addr + i)) {
                led = &led_state[i];
                board_led_operation(led->pin, onoff);
            }
        }
    } else if (ESP_BLE_MESH_ADDR_IS_GROUP(ctx->recv_dst)) {
        if (esp_ble_mesh_is_model_subscribed_to_group(model, ctx->recv_dst)) {
            led = &led_state[model->element->element_addr - primary_addr];
            board_led_operation(led->pin, onoff);
        }
    } else if (ctx->recv_dst == 0xFFFF) {
        led = &led_state[model->element->element_addr - primary_addr];
        board_led_operation(led->pin, onoff);
    }
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
                      param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04x, src 0x%04x, dst 0x%04x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

#define MINDIFF (2.25e-308)

static float bt_mesh_sqrt(float square)
{
    float root, last, diff;

    root = square / 3.0;
    diff = 1;

    if (square <= 0) {
        return 0;
    }

    do {
        last = root;
        root = (root + square / root) / 2.0;
        diff = root - last;
    } while (diff > MINDIFF || diff < -MINDIFF);

    return root;
}

static int32_t bt_mesh_ceiling(float num)
{
    int32_t inum = (int32_t)num;
    if (num == (float)inum) {
        return inum;
    }
    return inum + 1;
}

static uint16_t convert_lightness_actual_to_linear(uint16_t actual)
{
    float tmp = ((float) actual / UINT16_MAX);
    return bt_mesh_ceiling(UINT16_MAX * tmp * tmp);
}

static uint16_t convert_lightness_linear_to_actual(uint16_t linear)
{
    return (uint16_t)(UINT16_MAX * bt_mesh_sqrt(((float) linear / UINT16_MAX)));
}

static int16_t convert_temperature_to_level(uint16_t temp, uint16_t min, uint16_t max)
{
    float tmp = (temp - min) * UINT16_MAX / (max - min);
    return (int16_t) (tmp + INT16_MIN);
}

uint16_t covert_level_to_temperature(int16_t level, uint16_t min, uint16_t max)
{
    float diff = (float) (max - min) / UINT16_MAX;
    uint16_t tmp = (uint16_t) ((level - INT16_MIN) * diff);
    return (uint16_t) (min + tmp);
}

static void example_ble_mesh_generic_server_cb(esp_ble_mesh_generic_server_cb_event_t event,
                                               esp_ble_mesh_generic_server_cb_param_t *param)
{
    esp_ble_mesh_server_state_value_t state = {0};
    uint16_t primary_addr;
    uint16_t lightness;
    int16_t level;

    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04x, src 0x%04x, dst 0x%04x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    primary_addr = esp_ble_mesh_get_primary_element_address();

    switch (event) {
    case ESP_BLE_MESH_GENERIC_SERVER_STATE_CHANGE_EVT:
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK:
            /* Generic OnOff Server Model - Primary Element */
            example_change_led_state(param->model, &param->ctx, param->value.state_change.onoff_set.onoff);
            /* Update bound states */
            if (param->value.state_change.onoff_set.onoff == LED_ON) {
                if (lightness_state.lightness_default == 0x0000) {
                    lightness = lightness_state.lightness_last;
                } else {
                    lightness = lightness_state.lightness_default;
                }
                state.light_lightness_actual.lightness = lightness;
                esp_ble_mesh_server_model_update_state(lightness_server.model, ESP_BLE_MESH_LIGHT_LIGHTNESS_ACTUAL_STATE, &state);
                state.light_lightness_linear.lightness = convert_lightness_actual_to_linear(lightness);
                esp_ble_mesh_server_model_update_state(lightness_server.model, ESP_BLE_MESH_LIGHT_LIGHTNESS_LINEAR_STATE, &state);
                state.gen_level.level = lightness - 32768;
                esp_ble_mesh_server_model_update_state(level_server_0.model, ESP_BLE_MESH_GENERIC_LEVEL_STATE, &state);
                state.light_hsl_lightness.lightness = lightness;
                esp_ble_mesh_server_model_update_state(hsl_server.model, ESP_BLE_MESH_LIGHT_HSL_LIGHTNESS_STATE, &state);
                state.light_ctl_lightness.lightness = lightness;
                esp_ble_mesh_server_model_update_state(ctl_server.model, ESP_BLE_MESH_LIGHT_CTL_LIGHTNESS_STATE, &state);
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET:
        case ESP_BLE_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK:
            /* Generic Level Server Model - Primary/Secondary Element(s) */
            ESP_LOGI(TAG, "level 0x%04x", param->value.state_change.level_set.level);
            /* Update bound states */
            level = param->value.state_change.level_set.level;
            if (param->model->element->element_addr == primary_addr) {
                /* Change corresponding bound states in root element */
                state.light_lightness_actual.lightness = level + 32768;
                esp_ble_mesh_server_model_update_state(lightness_server.model, ESP_BLE_MESH_LIGHT_LIGHTNESS_ACTUAL_STATE, &state);
                state.light_lightness_linear.lightness = convert_lightness_actual_to_linear(level + 32768);
                esp_ble_mesh_server_model_update_state(lightness_server.model, ESP_BLE_MESH_LIGHT_LIGHTNESS_LINEAR_STATE, &state);
                state.gen_onoff.onoff = (level + 32768) ? LED_ON : LED_OFF;
                esp_ble_mesh_server_model_update_state(onoff_server.model, ESP_BLE_MESH_GENERIC_ONOFF_STATE, &state);
                state.light_hsl_lightness.lightness = level + 32768;
                esp_ble_mesh_server_model_update_state(hsl_server.model, ESP_BLE_MESH_LIGHT_HSL_LIGHTNESS_STATE, &state);
                state.light_ctl_lightness.lightness = level + 32768;
                esp_ble_mesh_server_model_update_state(ctl_server.model, ESP_BLE_MESH_LIGHT_CTL_LIGHTNESS_STATE, &state);
            } else if (param->model->element->element_addr == primary_addr + 1) {
                /* Change corresponding bound states in hue element */
                state.light_hsl_hue.hue = level + 32768;
                esp_ble_mesh_server_model_update_state(hsl_hue_server.model, ESP_BLE_MESH_LIGHT_HSL_HUE_STATE, &state);
            } else if (param->model->element->element_addr == primary_addr + 2) {
                /* Change corresponding bound states in saturation element */
                state.light_hsl_saturation.saturation = level + 32768;
                esp_ble_mesh_server_model_update_state(hsl_saturation_server.model, ESP_BLE_MESH_LIGHT_HSL_SATURATION_STATE, &state);
            } else if (param->model->element->element_addr == primary_addr + 3) {
                /* Change corresponding bound states in temperature element */
                state.light_ctl_temp_delta_uv.temperature = covert_level_to_temperature(level,
                    ctl_temperature_server.state->temperature_range_min, ctl_temperature_server.state->temperature_range_max);
                state.light_ctl_temp_delta_uv.delta_uv = ctl_temperature_server.state->delta_uv;
                esp_ble_mesh_server_model_update_state(ctl_temperature_server.model, ESP_BLE_MESH_LIGHT_HSL_SATURATION_STATE, &state);
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_GEN_DEF_TRANS_TIME_SET:
        case ESP_BLE_MESH_MODEL_OP_GEN_DEF_TRANS_TIME_SET_UNACK:
            /* Generic Default Transition Time Server Model - Primary Element */
            ESP_LOGI(TAG, "Default transition time 0x%02x", param->value.state_change.def_trans_time_set.trans_time);
            break;
        case ESP_BLE_MESH_MODEL_OP_GEN_ONPOWERUP_SET:
        case ESP_BLE_MESH_MODEL_OP_GEN_ONPOWERUP_SET_UNACK:
            /* Generic Power OnOff Setup Server Model - Primary Element */
            ESP_LOGI(TAG, "onpowerup 0x%02x", param->value.state_change.onpowerup_set.onpowerup);
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Generic Server event 0x%02x", event);
        break;
    }
}

static void example_ble_mesh_lighting_server_cb(esp_ble_mesh_lighting_server_cb_event_t event,
                                                esp_ble_mesh_lighting_server_cb_param_t *param)
{
    esp_ble_mesh_server_state_value_t state = {0};
    uint16_t lightness;

    ESP_LOGI(TAG, "event 0x%02x, opcode 0x%04x, src 0x%04x, dst 0x%04x",
        event, param->ctx.recv_op, param->ctx.addr, param->ctx.recv_dst);

    switch (event) {
    case ESP_BLE_MESH_LIGHTING_SERVER_STATE_CHANGE_EVT:
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_SET_UNACK:
            /* Light Lightness Server Model - Primary Element */
            ESP_LOGI(TAG, "lightness actual 0x%04x", param->value.state_change.lightness_set.lightness);
            /* Update bound states */
            lightness = param->value.state_change.lightness_set.lightness;
            state.light_lightness_linear.lightness = convert_lightness_actual_to_linear(lightness);
            esp_ble_mesh_server_model_update_state(lightness_server.model, ESP_BLE_MESH_LIGHT_LIGHTNESS_LINEAR_STATE, &state);
            state.gen_level.level = lightness - 32768;
            esp_ble_mesh_server_model_update_state(level_server_0.model, ESP_BLE_MESH_GENERIC_LEVEL_STATE, &state);
            state.gen_onoff.onoff = lightness ? LED_ON : LED_OFF;
            esp_ble_mesh_server_model_update_state(onoff_server.model, ESP_BLE_MESH_GENERIC_ONOFF_STATE, &state);
            state.light_hsl_lightness.lightness = lightness;
            esp_ble_mesh_server_model_update_state(hsl_server.model, ESP_BLE_MESH_LIGHT_HSL_LIGHTNESS_STATE, &state);
            state.light_ctl_lightness.lightness = lightness;
            esp_ble_mesh_server_model_update_state(ctl_server.model, ESP_BLE_MESH_LIGHT_CTL_LIGHTNESS_STATE, &state);
            break;
        case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_LINEAR_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_LINEAR_SET_UNACK:
            /* Light Lightness Server Model - Primary Element */
            ESP_LOGI(TAG, "lightness linear 0x%04x", param->value.state_change.lightness_linear_set.lightness);
            /* Update bound states */
            lightness = convert_lightness_linear_to_actual(param->value.state_change.lightness_linear_set.lightness);
            state.light_lightness_actual.lightness = lightness;
            esp_ble_mesh_server_model_update_state(lightness_server.model, ESP_BLE_MESH_LIGHT_LIGHTNESS_ACTUAL_STATE, &state);
            state.gen_level.level = lightness - 32768;
            esp_ble_mesh_server_model_update_state(level_server_0.model, ESP_BLE_MESH_GENERIC_LEVEL_STATE, &state);
            state.gen_onoff.onoff = lightness ? LED_ON : LED_OFF;
            esp_ble_mesh_server_model_update_state(onoff_server.model, ESP_BLE_MESH_GENERIC_ONOFF_STATE, &state);
            state.light_hsl_lightness.lightness = lightness;
            esp_ble_mesh_server_model_update_state(hsl_server.model, ESP_BLE_MESH_LIGHT_HSL_LIGHTNESS_STATE, &state);
            state.light_ctl_lightness.lightness = lightness;
            esp_ble_mesh_server_model_update_state(ctl_server.model, ESP_BLE_MESH_LIGHT_CTL_LIGHTNESS_STATE, &state);
            break;
        case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_DEFAULT_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_DEFAULT_SET_UNACK:
            /* Light Lightness Setup Server Model - Primary Element */
            ESP_LOGI(TAG, "lightness default 0x%04x", param->value.state_change.lightness_default_set.lightness);
            break;
        case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_RANGE_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_LIGHTNESS_RANGE_SET_UNACK:
            /* Light Lightness Setup Server Model - Primary Element */
            ESP_LOGI(TAG, "lightness min 0x%04x, max 0x%04x",
                param->value.state_change.lightness_range_set.range_min,
                param->value.state_change.lightness_range_set.range_max);
            break;
        case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_SET_UNACK:
            /* Light CTL Server Model - Primary Element */
            ESP_LOGI(TAG, "lightness 0x%04x, temperature 0x%04x, delta uv 0x%04x",
                param->value.state_change.ctl_set.lightness,
                param->value.state_change.ctl_set.temperature,
                param->value.state_change.ctl_set.delta_uv);
            /* Update bound states */
            lightness = param->value.state_change.ctl_set.lightness;
            state.light_lightness_actual.lightness = lightness;
            esp_ble_mesh_server_model_update_state(lightness_server.model, ESP_BLE_MESH_LIGHT_LIGHTNESS_ACTUAL_STATE, &state);
            state.light_lightness_linear.lightness = convert_lightness_actual_to_linear(lightness);
            esp_ble_mesh_server_model_update_state(lightness_server.model, ESP_BLE_MESH_LIGHT_LIGHTNESS_LINEAR_STATE, &state);
            state.gen_level.level = lightness - 32768;
            esp_ble_mesh_server_model_update_state(level_server_0.model, ESP_BLE_MESH_GENERIC_LEVEL_STATE, &state);
            state.gen_onoff.onoff = lightness ? LED_ON : LED_OFF;
            esp_ble_mesh_server_model_update_state(onoff_server.model, ESP_BLE_MESH_GENERIC_ONOFF_STATE, &state);
            state.light_hsl_lightness.lightness = lightness;
            esp_ble_mesh_server_model_update_state(hsl_server.model, ESP_BLE_MESH_LIGHT_HSL_LIGHTNESS_STATE, &state);
            state.gen_level.level = convert_temperature_to_level(param->value.state_change.ctl_set.temperature,
                ctl_state.temperature_range_min, ctl_state.temperature_range_max);
            esp_ble_mesh_server_model_update_state(level_server_3.model, ESP_BLE_MESH_GENERIC_LEVEL_STATE, &state);
            break;
        case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_SET_UNACK:
            /* Light CTL Temperature Server Model - Secondary Element */
            ESP_LOGI(TAG, "temperature 0x%04x, delta uv 0x%04x",
                param->value.state_change.ctl_temp_set.temperature,
                param->value.state_change.ctl_temp_set.delta_uv);
            state.gen_level.level = convert_temperature_to_level(param->value.state_change.ctl_temp_set.temperature,
                ctl_state.temperature_range_min, ctl_state.temperature_range_max);
            esp_ble_mesh_server_model_update_state(level_server_3.model, ESP_BLE_MESH_GENERIC_LEVEL_STATE, &state);
            break;
        case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_DEFAULT_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_DEFAULT_SET_UNACK:
            /* Light CTL Setup Server Model - Primary Element */
            ESP_LOGI(TAG, "lightness 0x%04x, temperature 0x%04x, delta uv 0x%04x",
                param->value.state_change.ctl_default_set.lightness,
                param->value.state_change.ctl_default_set.temperature,
                param->value.state_change.ctl_default_set.delta_uv);
            break;
        case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_RANGE_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_CTL_TEMPERATURE_RANGE_SET_UNACK:
            /* Light CTL Setup Server Model - Primary Element */
            ESP_LOGI(TAG, "temperature min 0x%04x, max 0x%04x",
                param->value.state_change.ctl_temp_range_set.range_min,
                param->value.state_change.ctl_temp_range_set.range_max);
            break;
        case ESP_BLE_MESH_MODEL_OP_LIGHT_HSL_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_HSL_SET_UNACK:
            /* Light HSL Server Model - Primary Element */
            ESP_LOGI(TAG, "lightness 0x%04x, hue 0x%04x, saturation 0x%04x",
                param->value.state_change.hsl_set.lightness,
                param->value.state_change.hsl_set.hue,
                param->value.state_change.hsl_set.saturation);
            /* Update bound states */
            lightness = param->value.state_change.hsl_set.lightness;
            state.light_lightness_actual.lightness = lightness;
            esp_ble_mesh_server_model_update_state(lightness_server.model, ESP_BLE_MESH_LIGHT_LIGHTNESS_ACTUAL_STATE, &state);
            state.light_lightness_linear.lightness = convert_lightness_actual_to_linear(lightness);
            esp_ble_mesh_server_model_update_state(lightness_server.model, ESP_BLE_MESH_LIGHT_LIGHTNESS_LINEAR_STATE, &state);
            state.gen_level.level = lightness - 32768;
            esp_ble_mesh_server_model_update_state(level_server_0.model, ESP_BLE_MESH_GENERIC_LEVEL_STATE, &state);
            state.gen_onoff.onoff = lightness ? LED_ON : LED_OFF;
            esp_ble_mesh_server_model_update_state(onoff_server.model, ESP_BLE_MESH_GENERIC_ONOFF_STATE, &state);
            state.light_ctl_lightness.lightness = lightness;
            esp_ble_mesh_server_model_update_state(ctl_server.model, ESP_BLE_MESH_LIGHT_CTL_LIGHTNESS_STATE, &state);
            break;
        case ESP_BLE_MESH_MODEL_OP_LIGHT_HSL_HUE_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_HSL_HUE_SET_UNACK:
            /* Light HSL Hue Server Model - Secondary Element */
            ESP_LOGI(TAG, "hue 0x%04x", param->value.state_change.hsl_hue_set.hue);
            /* Update bound states */
            state.gen_level.level = param->value.state_change.hsl_hue_set.hue - 32768;
            esp_ble_mesh_server_model_update_state(level_server_1.model, ESP_BLE_MESH_GENERIC_LEVEL_STATE, &state);
            break;
        case ESP_BLE_MESH_MODEL_OP_LIGHT_HSL_SATURATION_SET:
        case ESP_BLE_MESH_MODEL_OP_LIGHT_HSL_SATURATION_SET_UNACK:
            /* Light HSL Saturation Server Model - Secondary Element */
            ESP_LOGI(TAG, "saturation 0x%04x", param->value.state_change.hsl_saturation_set.saturation);
            /* Update bound states */
            state.gen_level.level = param->value.state_change.hsl_saturation_set.saturation - 32768;
            esp_ble_mesh_server_model_update_state(level_server_2.model, ESP_BLE_MESH_GENERIC_LEVEL_STATE, &state);
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Lighting Server event 0x%02x", event);
        break;
    }
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_SERVER_MODEL_UPDATE_STATE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_SERVER_MODEL_UPDATE_STATE_COMP_EVT");
        ESP_LOGI(TAG, "result %d, model id 0x%04x, type 0x%02x",
            param->server_model_update_state.err_code,
            param->server_model_update_state.model->model_id,
            param->server_model_update_state.type);
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_generic_server_callback(example_ble_mesh_generic_server_cb);
    esp_ble_mesh_register_lighting_server_callback(example_ble_mesh_lighting_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err) {
        ESP_LOGE(TAG, "Initializing mesh failed (err %d)", err);
        return err;
    }

    esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    board_led_operation(LED_G, LED_ON);

    return err;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    board_init();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}
