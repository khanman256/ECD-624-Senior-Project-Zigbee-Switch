#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl_utility.h"
#include "aps/esp_zigbee_aps.h"
#include "esp_zb_switch.h"

#if defined ZB_ED_ROLE
#error Define ZB_COORDINATOR_ROLE in idf.py menuconfig to compile light switch source code.
#endif

static const char *TAG = "ESP_ZB_SWITCH";

/* Custom APS message config */
#define TEXT_MESSAGE_CLUSTER_ID   0xFC01U
#define HA_PROFILE_ID             0x0104U

/* Light target — update after join if needed */
#define TARGET_SHORT_ADDR         0x47F1U
#define TARGET_ENDPOINT           10U

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}
};

/* Forward declarations */
static void send_text_message(const char *text, uint32_t length);

static void zb_buttons_handler(switch_func_pair_t *button)
{
    if (button->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        ESP_LOGI(TAG, "Button pressed — sending APS text message");
        send_text_message("HELLO THERE MY GUY");
    }
}

/********************* APS sender **********************************/
static void send_text_message(const char *text, uint32_t length)
{
    if (!text) {
        return;
    }

    esp_zb_apsde_data_req_t req;
    memset(&req, 0, sizeof(req));

    req.dst_addr_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    req.dst_addr.addr_short = TARGET_SHORT_ADDR;
    req.dst_endpoint = TARGET_ENDPOINT;
    req.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
    req.profile_id = HA_PROFILE_ID;
    req.cluster_id = TEXT_MESSAGE_CLUSTER_ID;

    req.asdu = (uint8_t *)text;
    req.asdu_length = strlen(text) + 1;
    req.tx_options = ESP_ZB_APSDE_TX_OPT_ACK_TX;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t res = esp_zb_aps_data_request(&req);
    esp_zb_lock_release();

    ESP_LOGI(TAG,
             "Sent APS text '%s' to 0x%04x ep %u, result: %s",
             text, TARGET_SHORT_ADDR, TARGET_ENDPOINT,
             esp_err_to_name(res));
}

static esp_err_t deferred_driver_init(void)
{
    ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair),zb_buttons_handler),
        ESP_FAIL, TAG, "Failed to initialize switch driver");

    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(
        esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

/********************* ZDO signal handler **************************/
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {

    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s",
                     deferred_driver_init() ? "failed" : "successful");

            ESP_LOGI(TAG, "Device started up in %s factory-reset mode",
                     esp_zb_bdb_is_factory_new() ? "" : "non");

            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network formation");
                esp_zb_bdb_start_top_level_commissioning(
                    ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                esp_zb_bdb_open_network(180);
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)",
                     esp_err_to_name(err_status));
        }
        break;

    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);

            ESP_LOGI(TAG,
                     "Formed network (PAN ID: 0x%04hx, Channel:%d, Short: 0x%04hx)",
                     esp_zb_get_pan_id(),
                     esp_zb_get_current_channel(),
                     esp_zb_get_short_address());

            esp_zb_bdb_start_top_level_commissioning(
                ESP_ZB_BDB_MODE_NETWORK_STEERING);
            esp_zb_bdb_open_network(180);
        } else {
            ESP_LOGI(TAG, "Restart network formation (status: %s)",
                     esp_err_to_name(err_status));

            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
        esp_zb_zdo_signal_device_annce_params_t *dev =
            (esp_zb_zdo_signal_device_annce_params_t *)
            esp_zb_app_signal_get_params(p_sg_p);

        ESP_LOGI(TAG, "New device joined/rejoined (short: 0x%04hx)",
                 dev->device_short_addr);
        break;
    }

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type),
                 sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

/********************* Zigbee task *********************************/
static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_on_off_switch_cfg_t switch_cfg =
        ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();

    esp_zb_ep_list_t *ep =
        esp_zb_on_off_switch_ep_create(HA_ONOFF_SWITCH_ENDPOINT, &switch_cfg);

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier  = ESP_MODEL_IDENTIFIER,
    };

    esp_zcl_utility_add_ep_basic_manufacturer_info(
        ep, HA_ONOFF_SWITCH_ENDPOINT, &info);

    esp_zb_device_register(ep);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/********************* app_main *************************************/
void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
