/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_message_client.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "example_common.h"
#include "ble_softdevice_support.h"

#define APP_STATE_OFF                (0)
#define APP_STATE_ON                 (1)

#define APP_UNACK_MSG_REPEAT_COUNT   (2)
#define APP_GET_REQ_SIZE            (11)   // size of the get request

static volatile uint16_t test_byte_counter = 0;
static volatile uint16_t missed_transfer_counter = 0;
static volatile int32_t rssi_sum = 0;
static volatile uint16_t rssi_count = 0;
static volatile bool run = false;
static volatile uint32_t test_timer_start;

static generic_message_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static bool                   m_device_provisioned;

/* Forward declaration */
static void app_gen_message_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_message_client_status_cb(const generic_message_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_message_get_params_t * p_in);
static void app_gen_message_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);
static void timeout_handler(void * p_context);
static void send_get_message();
static void send_set_message();
static void run_test();
static void start_test();
static void stop_test();

APP_TIMER_DEF(test_timer);

const generic_message_client_callbacks_t client_cbs =
{
    .message_status_cb = app_generic_message_client_status_cb,
    .ack_transaction_status_cb = app_gen_message_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_message_client_publish_interval_cb
};

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(BSP_LED_2_MASK  | BSP_LED_3_MASK,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_message_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_gen_message_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            ++missed_transfer_counter;
            run_test();
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            ++missed_transfer_counter;
            run_test();
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            ++missed_transfer_counter;
            run_test();
            break;
    }
}

/* Generic message client model interface: Process the received status message in this callback */
static void app_generic_message_client_status_cb(const generic_message_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_message_get_params_t * p_in)
{    
    ++rssi_count;
    rssi_sum += p_meta->p_core_metadata->params.scanner.rssi;
    test_byte_counter += p_in->msg_len;
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Bytes received: %d\n", test_byte_counter);
    // __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Message server: 0x%04x: %s\n", p_meta->src.value, &p_in->message);
    
    // __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "rssi: %d\n", p_meta->p_core_metadata->params.scanner.rssi);

    run_test();
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void run_test()
{
    if(run && test_byte_counter < APP_CONFIG_BYTE_TRANSFER_CNT) {
      send_get_message();
    } else {
      stop_test(); // and print results
    }
}

static void start_test()
{
    if(!run) {
      test_byte_counter = 0;
      missed_transfer_counter = 0;
      rssi_count = 0;
      rssi_sum = 0;
      ERROR_CHECK(app_timer_start(test_timer, APP_TIMER_TICKS(1000 * 60 * 60), timeout_handler));
      test_timer_start = MODEL_TIMER_PERIOD_MS_GET(app_timer_cnt_get());
      run = true;
      run_test();
    }
}

static void stop_test()
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Stopping test\n");


    //uint32_t time_ticks   = app_timer_cnt_diff_compute(app_timer_cnt_get(), test_timer_start);
    uint32_t time_ms      = MODEL_TIMER_PERIOD_MS_GET(app_timer_cnt_get()) - test_timer_start;
    ERROR_CHECK(app_timer_stop(test_timer));

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "=============================\n");
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Time: %u.%.2u seconds elapsed.\n", (time_ms / 1000), (time_ms % 1000));

    uint32_t bit_count    = (test_byte_counter) * 8000;
    uint32_t throughput_kbps = bit_count / time_ms;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Throughput: %u.%.2u Kbps.\n", (throughput_kbps / 1000), (throughput_kbps % 1000));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Received %u bytes of payload.\n", test_byte_counter);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Missed %u acknowledged transfers.\n", missed_transfer_counter);

    if(0 != rssi_sum) 
    {
        int32_t avg_rssi = (int32_t)((float)rssi_sum / (float)rssi_count);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Average RSSI: %d dBm\n", avg_rssi);
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "=============================\n");

    run = false;
}

static void send_get_message()
{
    uint32_t status = NRF_SUCCESS;

    // __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Requesting message\n");

    //(void)access_model_reliable_cancel(m_clients[0].model_handle);
    status = generic_message_client_get(&m_clients[0]);
    //test_byte_counter += APP_GET_REQ_SIZE;

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", 1);
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", 1);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void send_set_message()
{
    uint32_t status = NRF_SUCCESS;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending message\n");

    //(void)access_model_reliable_cancel(m_clients[0].model_handle);
    //status = generic_message_client_set(&m_clients[0]);

}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Button %u pressed\n", button_number);


    /* Button 1: Run test, Button 2, 3, 4: Stop, Client[0]
     */
    switch (button_number)
    {
        case 0:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Starting test.\n");
            start_test();
            break;
        default:
            stop_test();
            break;
    }
}

static void rtt_input_handler(int key)
{
    if (key >= '0' && key <= '1')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void timeout_handler(void * p_context)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Timeout handler called\n");
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_CONFIG_MIC_SIZE;

        ERROR_CHECK(generic_message_client_init(&m_clients[i], i + 1));
    }
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    ERROR_CHECK(app_timer_init());
    ERROR_CHECK(app_timer_create(&test_timer, APP_TIMER_MODE_SINGLE_SHOT, timeout_handler));
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_LS_CLIENT
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
