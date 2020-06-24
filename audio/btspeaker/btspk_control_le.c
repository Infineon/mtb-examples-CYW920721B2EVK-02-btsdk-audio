/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * This file implement BTLE controls.
 * The GATT database is defined in this file.
 *
 */
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_app.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include <wiced_bt_ota_firmware_upgrade.h>
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_timer.h"
#include "bt_hs_spk_control.h"
#include "wiced_memory.h"
#include "wiced_app_cfg.h"
#include "btspk_nvram.h"
#include "btspk_control_le.h"
#ifdef FASTPAIR_ENABLE
#include "wiced_bt_gfps.h"
#endif

/******************************************************
 *                     Constants
 ******************************************************/
#define LE_CONTROL_MAX_CONNECTIONS          20
#define LE_CONTROL_CONNECT_TIMEOUT          10

/* UUID value of the Hello Sensor Service */
#define UUID_HELLO_SERVICE                    0x23, 0x20, 0x56, 0x7c, 0x05, 0xcf, 0x6e, 0xb4, 0xc3, 0x41, 0x77, 0x28, 0x51, 0x82, 0x7e, 0x1b
/* UUID value of the Hello Sensor Characteristic, Value Notification */
#define UUID_HELLO_CHARACTERISTIC_NOTIFY      0x26, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8a
/* UUID value of the Hello Sensor Characteristic, Configuration */
#define UUID_HELLO_CHARACTERISTIC_CONFIG      0x1a, 0x89, 0x07, 0x4a, 0x2f, 0x3b, 0x7e, 0xa6, 0x81, 0x44, 0x3f, 0xf9, 0xa8, 0xf2, 0x9b, 0x5e
/* UUID value of the Hello Sensor Characteristic, Configuration */
#define UUID_HELLO_CHARACTERISTIC_LONG_MSG    0x2a, 0x99, 0x17, 0x5a, 0x3f, 0x4b, 0x8e, 0xb6, 0x91, 0x54, 0x2f, 0x09, 0xb8, 0x02, 0xab, 0x6e

#ifdef FASTPAIR_ENABLE
/* MODEL-specific definitions */
#if defined(CYW20721B2) || defined(CYW43012C0)
#define FASTPAIR_MODEL_ID                   0x82DA6E
#else
#define FASTPAIR_MODEL_ID                   0xCE948F //0xB49236 //0x000107 //0x140A02 // 0xCE948F
#endif

#if (FASTPAIR_MODEL_ID == 0x82DA6E)
const uint8_t anti_spoofing_public_key[] =  { 0x95, 0xcf, 0xdb, 0xae, 0xc0, 0xef, 0xc5, 0x1f, 0x39, 0x0f, 0x2a, 0xe0, 0x16, 0x5a, 0x2b, 0x59,\
		                                      0x62, 0xb2, 0xfe, 0x82, 0xfa, 0xf0, 0xd4, 0x1e, 0xa3, 0x4f, 0x07, 0x7e, 0xf7, 0x3d, 0xc0, 0x44,\
		                                      0x3d, 0xd0, 0x38, 0xb2, 0x31, 0x5d, 0xc6, 0x45, 0x72, 0x8a, 0x08, 0x0e, 0xc7, 0x4f, 0xc7, 0x76,\
		                                      0xd1, 0x19, 0xed, 0x8b, 0x17, 0x50, 0xb3, 0xa6, 0x94, 0x2e, 0xc8, 0x6b, 0xbb, 0x02, 0xc7, 0x4d };

const uint8_t anti_spoofing_private_key[] = { 0x84, 0xee, 0x67, 0xc3, 0x67, 0xea, 0x57, 0x38, 0xa7, 0x7e, 0xe2, 0x4d, 0x68, 0xaa, 0x9c, 0xf0,\
                                              0xc7, 0x9f, 0xc8, 0x07, 0x7e, 0x4e, 0x20, 0x35, 0x4c, 0x15, 0x43, 0x4d, 0xb5, 0xd2, 0xd1, 0xc3 };

#elif (FASTPAIR_MODEL_ID == 0xCE948F)
const uint8_t anti_spoofing_public_key[] =  { 0x0e, 0xe2, 0xbf, 0xe7, 0x96, 0xc6, 0xe1, 0x13, 0xf6, 0x57, 0x4a, 0xa8, 0x8c, 0x3a, 0x1b, 0x9c,\
                                              0x67, 0x1e, 0x36, 0xdf, 0x62, 0x69, 0xd8, 0xe5, 0x07, 0xe6, 0x8a, 0x72, 0x66, 0x4c, 0x9c, 0x90,\
                                              0xfc, 0xff, 0x00, 0x4f, 0x0f, 0x95, 0xde, 0x63, 0xe1, 0xc0, 0xbb, 0xa0, 0x75, 0xb1, 0xd2, 0x76,\
                                              0xfd, 0xe9, 0x66, 0x25, 0x0d, 0x45, 0x43, 0x7d, 0x5b, 0xf9, 0xce, 0xc0, 0xeb, 0x11, 0x03, 0xbe };

const uint8_t anti_spoofing_private_key[] = { 0x71, 0x11, 0x42, 0xb5, 0xe4, 0xa0, 0x6c, 0xa2, 0x8b, 0x74, 0xd4, 0x87, 0x7d, 0xac, 0x15, 0xc5,\
                                              0x42, 0x38, 0x1d, 0xb7, 0xba, 0x21, 0x19, 0x60, 0x17, 0x67, 0xfc, 0xba, 0x67, 0x47, 0x44, 0xc6 };

#else
const uint8_t anti_spoofing_public_key[] =  "";
const uint8_t anti_spoofing_private_key[] = "";
#warning "No Anti-Spooging key"

#endif
#endif //FASTPAIR_ENABLE

/******************************************************
 *                     Structures
 ******************************************************/
typedef struct
{
#define LE_CONTROL_STATE_IDLE                       0
#define LE_CONTROL_STATE_DISCOVER_PRIMARY_SERVICES  1
#define LE_CONTROL_STATE_DISCOVER_CHARACTERISTICS   2
#define LE_CONTROL_STATE_DISCOVER_DESCRIPTORS       3
#define LE_CONTROL_STATE_READ_VALUE                 4
#define LE_CONTROL_STATE_WRITE_VALUE                5
#define LE_CONTROL_STATE_WRITE_NO_RESPONSE_VALUE    6
#define LE_CONTROL_STATE_NOTIFY_VALUE               7
#define LE_CONTROL_STATE_INDICATE_VALUE             8
#define LE_CONTROL_STATE_WRITE_DESCRIPTOR_VALUE     9
#define LE_CONTROL_STATE_DISCONNECTING              10

    uint8_t           state;                // Application discovery state
    wiced_bool_t      indication_sent;      // TRUE if indication sent and not acked
    BD_ADDR           bd_addr;
    uint16_t          conn_id;              // Connection ID used for exchange with the stack
    uint16_t          peer_mtu;             // MTU received in the MTU request (or 23 if peer did not send MTU request)

    uint8_t           role;                 // HCI_ROLE_MASTER or HCI_ROLE_SLAVE
} hci_control_le_conn_state_t;

typedef struct
{
    hci_control_le_conn_state_t conn[LE_CONTROL_MAX_CONNECTIONS + 1];
} hci_control_le_cb_t;

hci_control_le_cb_t le_control_cb;

typedef struct t_hci_control_le_pending_tx_buffer_t
{
    wiced_bool_t        tx_buf_saved;
    uint16_t            tx_buf_conn_id;
    uint16_t            tx_buf_type;
    uint16_t            tx_buf_len;
    uint16_t            tx_buf_handle;
    uint8_t             tx_buf_data[HCI_CONTROL_GATT_COMMAND_MAX_TX_BUFFER];
} hci_control_le_pending_tx_buffer_t;

wiced_timer_t hci_control_le_connect_timer;


/******************************************************
 *               Variables Definitions
 ******************************************************/

//uint8_t  hci_control_le_data_xfer_buf[20] = {0xff, 0xfe};
BD_ADDR  hci_control_le_remote_bdaddr;

hci_control_le_pending_tx_buffer_t hci_control_le_pending_tx_buffer;

/******************************************************************************
 *                                GATT DATABASE
 ******************************************************************************/
/*
 * This is the GATT database for the Hello Sensor application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t gatt_server_db[]=
{
    /* Declare mandatory GATT service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GATT_SERVICE, UUID_SERVICE_GATT ),

    /* Declare mandatory GAP service. Device Name and Appearance are mandatory
     * characteristics of GAP service                                        */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_GAP_SERVICE, UUID_SERVICE_GAP ),

        /* Declare mandatory GAP service characteristic: Dev Name */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,
                GATT_UUID_GAP_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Declare mandatory GAP service characteristic: Appearance */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
                GATT_UUID_GAP_ICON, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    /* Declare proprietary Hello Service with 128 byte UUID */
    PRIMARY_SERVICE_UUID128( HANDLE_HSENS_SERVICE, UUID_HELLO_SERVICE ),

        /* Declare characteristic used to notify/indicate change */
        CHARACTERISTIC_UUID128( HANDLE_HSENS_SERVICE_CHAR_NOTIFY, HANDLE_HSENS_SERVICE_CHAR_NOTIFY_VAL,
            UUID_HELLO_CHARACTERISTIC_NOTIFY, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_READABLE ),

            /* Declare client characteristic configuration descriptor
             * Value of the descriptor can be modified by the client
             * Value modified shall be retained during connection and across connection
             * for bonded devices.  Setting value to 1 tells this application to send notification
             * when value of the characteristic changes.  Value 2 is to allow indications. */
            CHAR_DESCRIPTOR_UUID16_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_CFG_DESC, GATT_UUID_CHAR_CLIENT_CONFIG,
                LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

        /* Declare characteristic Hello Configuration */
        CHARACTERISTIC_UUID128_WRITABLE( HANDLE_HSENS_SERVICE_CHAR_BLINK, HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,
            UUID_HELLO_CHARACTERISTIC_CONFIG, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ ),

    /* Declare Device info service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE, UUID_SERVCLASS_DEVICE_INFO ),

        /* Handle 0x4e: characteristic Manufacturer Name, handle 0x4f characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
                GATT_UUID_MANU_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Handle 0x50: characteristic Model Number, handle 0x51 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,
                GATT_UUID_MODEL_NUMBER_STR, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

        /* Handle 0x52: characteristic System ID, handle 0x53 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID, HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL,
                GATT_UUID_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

    /* Declare Battery service */
    PRIMARY_SERVICE_UUID16( HANDLE_HSENS_BATTERY_SERVICE, UUID_SERVCLASS_BATTERY ),

        /* Handle 0x62: characteristic Battery Level, handle 0x63 characteristic value */
        CHARACTERISTIC_UUID16( HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL, HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
                GATT_UUID_BATTERY_LEVEL, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE),

#ifdef FASTPAIR_ENABLE
    // Declare Fast Pair service
    PRIMARY_SERVICE_UUID16 (HANDLE_FASTPAIR_SERVICE, WICED_BT_GFPS_UUID16),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING,
                                    HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_KEY_PAIRING,
                                    LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY,
                                    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_WRITE_REQ),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY,
                                    HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_PASSKEY,
                                    LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY,
                                    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_WRITE_REQ),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY,
                                    HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_ACCOUNT_KEY,
                                    LEGATTDB_CHAR_PROP_WRITE,
                                    LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    LEGATTDB_PERM_AUTH_READABLE | LEGATTDB_PERM_WRITE_REQ),
#endif

    /* WICED Upgrade Service. */
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_SEC_FW_UPGRADE_SERVICE),
#else
    PRIMARY_SERVICE_UUID128(HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_FW_UPGRADE_SERVICE),
#endif
        /* characteristic Control Point */
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT, LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
            LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ),

        /* client characteristic configuration descriptor */
        CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
            LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ),

        /* characteristic Data. */
        CHARACTERISTIC_UUID128_WRITABLE(HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, HANDLE_OTA_FW_UPGRADE_DATA,
            UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA, LEGATTDB_CHAR_PROP_WRITE,
            LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_RELIABLE_WRITE),
};

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;

uint8_t btspeaker_sensor_device_name[]          = "BTSpeakerPro";
uint8_t btspeaker_sensor_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };
char    btspeaker_sensor_char_notify_value[]    = { 'H', 'e', 'l', 'l', 'o', ' ', '0', };
char    btspeaker_sensor_char_mfr_name_value[]  = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0, };
char    btspeaker_sensor_char_model_num_value[] = { '1', '2', '3', '4',   0,   0,   0,   0 };
uint8_t btspeaker_sensor_char_system_id_value[] = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71};

static uint8_t btspeaker_battery_level;
static char blink_value;

static char *p_btspk_control_le_dev_name = NULL;
static wiced_bt_ble_advert_elem_t btspk_control_le_adv_elem = {0};

attribute_t gauAttributes[] =
{
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof( btspeaker_sensor_device_name ),         btspeaker_sensor_device_name },
    { HANDLE_HSENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof(btspeaker_sensor_appearance_name),       btspeaker_sensor_appearance_name },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,  sizeof(btspeaker_sensor_char_mfr_name_value),   btspeaker_sensor_char_mfr_name_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL, sizeof(btspeaker_sensor_char_model_num_value),  btspeaker_sensor_char_model_num_value },
    { HANDLE_HSENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, sizeof(btspeaker_sensor_char_system_id_value),  btspeaker_sensor_char_system_id_value },
    { HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,      1,                                            &btspeaker_battery_level },
    { HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL,      1,                                            &blink_value },
};
/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_bt_gatt_status_t hci_control_le_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
static wiced_result_t         hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t         hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status );
static void                   hci_control_le_connect_timeout( uint32_t count );
static wiced_result_t         hci_control_le_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static void                   hci_control_le_notification_handler( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );
static void                   hci_control_le_indication_handler( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len );
static void                   hci_control_le_indication_confirmation_handler( uint16_t conn_id, uint16_t handle );
static void                   hci_control_le_num_complete_callback( void );
static void                   hci_control_le_handle_gatt_db_init(uint8_t* data, uint32_t length);

/*
 * Initialize LE Control
 */
void hci_control_le_init( void )
{
    memset( &le_control_cb, 0, sizeof( le_control_cb ) );
    memset( &hci_control_le_pending_tx_buffer, 0, sizeof( hci_control_le_pending_tx_buffer ) );
}

static void headset_control_le_discoverabilty_change_callback(wiced_bool_t discoverable)
{
#ifdef FASTPAIR_ENABLE
    wiced_bt_gfps_provider_discoverablility_set(discoverable);
#endif
}

/*
 * Enable LE Control
 */
void hci_control_le_enable( void )
{
    wiced_bt_gatt_status_t     gatt_status;
#ifdef FASTPAIR_ENABLE
    wiced_bt_gfps_provider_conf_t fastpair_conf = {0};
#endif
    char appended_ble_dev_name[] = " LE";
    uint8_t *p_index;
    uint16_t dev_name_len;

    WICED_BT_TRACE( "hci_control_le_enable\n" );

    /*  GATT DB Initialization */
    gatt_status = wiced_bt_gatt_db_init(gatt_server_db, sizeof(gatt_server_db));

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

#ifdef FASTPAIR_ENABLE
    // set Tx power level data type in ble advertisement
#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1)
    fastpair_conf.ble_tx_pwr_level = wiced_bt_cfg_settings.default_ble_power_level;
#else
    fastpair_conf.ble_tx_pwr_level = 0;
#endif

    // set GATT event callback
    fastpair_conf.p_gatt_cb = hci_control_le_gatt_callback;

    // set assigned handles for GATT attributes
    fastpair_conf.gatt_db_handle.key_pairing_val        = HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_VAL;
    fastpair_conf.gatt_db_handle.key_pairing_cfg_desc   = HANDLE_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_CFG_DESC;
    fastpair_conf.gatt_db_handle.passkey_val            = HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_VAL;
    fastpair_conf.gatt_db_handle.passkey_cfg_desc       = HANDLE_FASTPAIR_SERVICE_CHAR_PASSKEY_CFG_DESC;
    fastpair_conf.gatt_db_handle.account_key_val        = HANDLE_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_VAL;

    // model id
    fastpair_conf.model_id = FASTPAIR_MODEL_ID;

    // anti-spoofing public key
    memcpy((void *) &fastpair_conf.anti_spoofing_key.public[0],
           (void *) &anti_spoofing_public_key[0],
           WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PUBLIC);

    // anti-spoofing private key
    memcpy((void *) &fastpair_conf.anti_spoofing_key.private[0],
           (void *) &anti_spoofing_private_key[0],
           WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PRIVATE);

    // Account Key Filter generate format
    fastpair_conf.account_key_filter_generate_random = WICED_TRUE;;

    // Account Key list size
    fastpair_conf.account_key_list_size = FASTPAIR_ACCOUNT_KEY_NUM;

    // NVRAM id for Account Key list
    fastpair_conf.account_key_list_nvram_id = BTSPK_NVRAM_ID_GFPS_ACCOUNT_KEY;

    // BLE advertisement appended to fast pair advertisement data
    dev_name_len = strlen((char *) wiced_bt_cfg_settings.device_name) +
                   strlen(appended_ble_dev_name);

    p_btspk_control_le_dev_name = (char *) wiced_memory_allocate(dev_name_len);

    if (p_btspk_control_le_dev_name)
    {
        p_index = (uint8_t *) p_btspk_control_le_dev_name;

        memcpy((void *) p_index,
               (void *) wiced_bt_cfg_settings.device_name,
               strlen((char *) wiced_bt_cfg_settings.device_name));

        p_index += strlen((char *) wiced_bt_cfg_settings.device_name);

        memcpy((void *) p_index,
               (void *) appended_ble_dev_name,
               strlen(appended_ble_dev_name));
    }
    else
    {
        dev_name_len = 0;
    }

    btspk_control_le_adv_elem.advert_type   = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    btspk_control_le_adv_elem.len           = dev_name_len;
    btspk_control_le_adv_elem.p_data        = (uint8_t *) p_btspk_control_le_dev_name;

    fastpair_conf.appended_adv_data.p_elem      = &btspk_control_le_adv_elem;
    fastpair_conf.appended_adv_data.elem_num    = 1;

    /* Initialize Google Fast Pair Service. */
    if (wiced_bt_gfps_provider_init(&fastpair_conf) == WICED_FALSE)
    {
        WICED_BT_TRACE("wiced_bt_gfps_provider_init fail\n");
    }
#else
    /* GATT registration */
    gatt_status = wiced_bt_gatt_register( hci_control_le_gatt_callback );
    WICED_BT_TRACE( "wiced_bt_gatt_register status %d\n", gatt_status );

#endif

    /* Register the BLE discoverability change callback. */
    bt_hs_spk_ble_discoverability_change_callback_register(&headset_control_le_discoverabilty_change_callback);

    /* Initialize connection timer */
    wiced_init_timer( &hci_control_le_connect_timer, &hci_control_le_connect_timeout, 0, WICED_SECONDS_TIMER );
}


/*
 * Process advertisement packet received
 */
void hci_control_le_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    if ( p_scan_result )
    {
        WICED_BT_TRACE( " Device : %B\n", p_scan_result->remote_bd_addr );
        //hci_control_le_send_advertisement_report( p_scan_result, p_adv_data );
    }
    else
    {
        WICED_BT_TRACE( " Scan completed\n" );
    }
}

/*
 * Process connection up event
 */
wiced_result_t hci_control_le_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    uint32_t       conn_id = p_status->conn_id;
    uint8_t        role;

    wiced_bt_dev_get_role( p_status->bd_addr, &role, p_status->transport);
    le_control_cb.conn[conn_id].role = role;

    WICED_BT_TRACE( "hci_control_le_connection_up, id:%d bd (%B) role:%d\n:", p_status->conn_id, p_status->bd_addr, role );

    /* Stop le connection timer*/
    wiced_stop_timer( &hci_control_le_connect_timer );

    memcpy( le_control_cb.conn[conn_id].bd_addr, p_status->bd_addr, BD_ADDR_LEN );
    le_control_cb.conn[conn_id].state      = LE_CONTROL_STATE_IDLE;
    le_control_cb.conn[conn_id].conn_id    = p_status->conn_id;
    le_control_cb.conn[conn_id].peer_mtu   = GATT_DEF_BLE_MTU_SIZE;

#if (WICED_APP_LE_SLAVE_CLIENT_INCLUDED == TRUE)
    // ask master to set preferred connection parameters
    wiced_bt_l2cap_update_ble_conn_params( p_status->bd_addr, 36, 72, 0, 200 );
    le_slave_connection_up(p_status);
#endif

    //hci_control_le_send_connect_event(0 /* TBD should come from p_status */,
    //        p_status->bd_addr, conn_id, role);
    return ( WICED_SUCCESS );
}

/*
* Process connection down event
*/
wiced_result_t hci_control_le_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    uint16_t          conn_id = p_status->conn_id;

    WICED_BT_TRACE( "le_connection_down conn_id:%x Disc_Reason: %02x\n", conn_id, p_status->reason );

    /* Stop le connection timer*/
    wiced_stop_timer( &hci_control_le_connect_timer );

    le_control_cb.conn[conn_id].state   = LE_CONTROL_STATE_IDLE;
    le_control_cb.conn[conn_id].conn_id = 0;

    if ( ( conn_id == hci_control_le_pending_tx_buffer.tx_buf_conn_id ) &&
                    ( hci_control_le_pending_tx_buffer.tx_buf_saved ) )
    {
        hci_control_le_pending_tx_buffer.tx_buf_saved = WICED_FALSE;
    }

#if (WICED_APP_LE_SLAVE_CLIENT_INCLUDED == TRUE)
    if (le_control_cb.conn[conn_id].role != HCI_ROLE_MASTER)
    {
        le_slave_connection_down(p_status);
    }
#endif

    //hci_control_le_send_disconnect_evt( p_status->reason, conn_id );

    return ( WICED_SUCCESS );
}


/*
* Process connection status callback
*/
wiced_result_t hci_control_le_conn_status_callback( wiced_bt_gatt_connection_status_t *p_status )
{
#ifdef OTA_FW_UPGRADE
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif

    if ( p_status->connected )
    {
        return hci_control_le_connection_up( p_status );
    }
    else
    {
        return hci_control_le_connection_down( p_status );
    }
}

/*
 * Operation complete received from the GATT server
 */
wiced_result_t hci_control_le_gatt_operation_comp_cb( wiced_bt_gatt_operation_complete_t *p_complete )
{
    uint16_t conn_id = p_complete->conn_id;

    switch ( p_complete->op )
    {
    case GATTC_OPTYPE_DISCOVERY:
        WICED_BT_TRACE( "!!! Disc compl conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state );
        break;

    case GATTC_OPTYPE_READ:
        // read response received, pass it up and set state to idle
        WICED_BT_TRACE( "Read response conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state );
        if ( le_control_cb.conn[conn_id].state == LE_CONTROL_STATE_READ_VALUE )
        {
            le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_IDLE;
        }
        break;

    case GATTC_OPTYPE_WRITE:
    case GATTC_OPTYPE_EXE_WRITE:
        // write response received, pass it up and set state to idle
        WICED_BT_TRACE( "Write response conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state );
        if ( le_control_cb.conn[conn_id].state == LE_CONTROL_STATE_WRITE_VALUE )
        {
            le_control_cb.conn[conn_id].state = LE_CONTROL_STATE_IDLE;
            //hci_control_le_send_write_completed( conn_id, p_complete->status );
        }
        break;

    case GATTC_OPTYPE_CONFIG:
        WICED_BT_TRACE( "Config conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state );
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        WICED_BT_TRACE( "Notification conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state );
        hci_control_le_notification_handler( conn_id,
                p_complete->response_data.att_value.handle,
                p_complete->response_data.att_value.p_data,
                p_complete->response_data.att_value.len );
        break;

    case GATTC_OPTYPE_INDICATION:
        WICED_BT_TRACE( "Indication conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state );
        hci_control_le_indication_handler( conn_id,
                p_complete->response_data.att_value.handle,
                p_complete->response_data.att_value.p_data,
                p_complete->response_data.att_value.len );
        break;
    }
    return ( WICED_SUCCESS );
}

/*
 * Discovery result received from the GATT server
 */
wiced_result_t hci_control_le_gatt_disc_result_cb( wiced_bt_gatt_discovery_result_t *p_result )
{
    uint16_t conn_id = p_result->conn_id;

    WICED_BT_TRACE( "Discovery result conn_id:%d state:%d\n", conn_id, le_control_cb.conn[conn_id].state );

    switch ( le_control_cb.conn[conn_id].state )
    {
    case LE_CONTROL_STATE_DISCOVER_PRIMARY_SERVICES:
        if ( ( p_result->discovery_type == GATT_DISCOVER_SERVICES_ALL ) ||
            ( p_result->discovery_type == GATT_DISCOVER_SERVICES_BY_UUID ) )
        {
            WICED_BT_TRACE( "Service s:%04x e:%04x uuid:%04x\n", p_result->discovery_data.group_value.s_handle,
                    p_result->discovery_data.group_value.e_handle, p_result->discovery_data.group_value.service_type.uu.uuid16 );
#if 0
            // services on the server can be based on 2 or 16 bytes UUIDs
            if ( p_result->discovery_data.group_value.service_type.len == 2 )
            {
                //hci_control_le_send_discovered_service16( conn_id, p_result->discovery_data.group_value.service_type.uu.uuid16,
                //        p_result->discovery_data.group_value.s_handle, p_result->discovery_data.group_value.e_handle );
            }
            else
            {
                //hci_control_le_send_discovered_service128( conn_id, p_result->discovery_data.group_value.service_type.uu.uuid128,
                //        p_result->discovery_data.group_value.s_handle, p_result->discovery_data.group_value.e_handle );
            }
#endif
        }
        break;

    case LE_CONTROL_STATE_DISCOVER_CHARACTERISTICS:
        if ( p_result->discovery_type == GATT_DISCOVER_CHARACTERISTICS )
        {
            WICED_BT_TRACE( "Found cha - uuid:%04x, hdl:%04x\n", p_result->discovery_data.characteristic_declaration.char_uuid.uu.uuid16, p_result->discovery_data.characteristic_declaration.handle );
        }
        break;

    case LE_CONTROL_STATE_DISCOVER_DESCRIPTORS:
        if ( p_result->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS )
        {
            WICED_BT_TRACE( "Found descr - uuid:%04x handle:%04x\n", p_result->discovery_data.char_descr_info.type.uu.uuid16, p_result->discovery_data.char_descr_info.handle );
#if 0
            // descriptor can be based on 2 or 16 bytes UUIDs
            if ( p_result->discovery_data.char_descr_info.type.len == 2 )
            {
                //hci_control_le_send_discovered_descriptor16( conn_id, p_result->discovery_data.char_descr_info.handle, p_result->discovery_data.char_descr_info.type.uu.uuid16 );
            }
            else
            {
                //hci_control_le_send_discovered_descriptor128( conn_id, p_result->discovery_data.char_descr_info.handle, p_result->discovery_data.char_descr_info.type.uu.uuid128 );
            }
#endif
        }
        break;

    default:
        WICED_BT_TRACE( "ignored\n" );
        break;
    }
    return ( WICED_SUCCESS );
}

/*
 * process discovery complete notification from the stack
 */
wiced_result_t hci_control_le_gatt_disc_comp_cb( wiced_bt_gatt_discovery_complete_t *p_data )
{
    // if we got here peer returned no more services, or we read up to the handle asked by client, report complete
    le_control_cb.conn[p_data->conn_id].state = LE_CONTROL_STATE_IDLE;
    return ( WICED_SUCCESS );
}

/*
 * Find attribute description by handle
 */
attribute_t * hci_control_get_attribute( uint16_t handle )
            {
    int i;
    for ( i = 0; i <  sizeof( gauAttributes ) / sizeof( gauAttributes[0] ); i++ )
    {
        if ( gauAttributes[i].handle == handle )
        {
            return ( &gauAttributes[i] );
            }
    }
    WICED_BT_TRACE( "attr not found:%x\n", handle );
    return NULL;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t hci_control_le_get_value( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = hci_control_get_attribute(p_read_data->handle) ) == NULL)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    /* Dummy battery value read increment */
    if( p_read_data->handle == HANDLE_HSENS_BATTERY_SERVICE_CHAR_LEVEL_VAL)
    {
        if ( btspeaker_battery_level++ > 5)
        {
            btspeaker_battery_level = 0;
        }
    }

    attr_len_to_copy = puAttribute->attr_len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy );

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is called when peer issues a Read Request to access characteristics values
 * in the GATT database.  Application can fill the provided buffer and return SUCCESS,
 * return error if something not appropriate, or return PENDING and send Read Response
 * when data is ready.
 */

wiced_bt_gatt_status_t hci_control_le_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t *p_req )
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_req->handle))
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_req);
    }
#endif

    WICED_BT_TRACE("[%s] [%d] [handle:%d] [conn_id:%d] [val:%d] [length:%d] \n",
            __func__, __LINE__, p_req->handle, conn_id, p_req->p_val,  p_req->p_val_len);

    return  hci_control_le_get_value(conn_id, p_req);
}

/*
 * The function invoked on timeout of hci_control_le_connect_timer
 */
void hci_control_le_connect_timeout( uint32_t count )
{
    /* Stop le connection timer*/
    wiced_stop_timer( &hci_control_le_connect_timer );

    /* Cancel connection request */
    wiced_bt_gatt_cancel_connect( hci_control_le_remote_bdaddr, WICED_TRUE );
}

/*
 * This function is called when peer issues a Write request to access characteristics values
 * in the GATT database
 */
wiced_result_t hci_control_le_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t *p_req )
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_req->handle))
    {
        return wiced_ota_fw_upgrade_write_handler(conn_id, p_req);
    }
#endif

    wiced_bt_gatt_status_t status = WICED_BT_GATT_PENDING;
    uint8_t attribute_value = *(uint8_t *)p_req->p_val;


    WICED_BT_TRACE( "hci_control_le_write_handler: conn_id:%d handle:%04x value:%i\n", conn_id, p_req->handle, attribute_value );
    switch ( p_req->handle )
    {
    case HANDLE_HSENS_SERVICE_CHAR_BLINK_VAL:
        WICED_BT_TRACE( "hci_control_le_write_handler: conn_id:%d handle:%04x value:%d length : %d \n", conn_id, p_req->handle, attribute_value, p_req->val_len );
        blink_value = attribute_value;
        status = WICED_BT_GATT_SUCCESS;
        break;
    default:
	if (status == WICED_BT_GATT_PENDING)
		WICED_BT_TRACE( "hci_control_le_write_handler: conn_id:%d handle:%04x value:%d length : %d \n", conn_id, p_req->handle, attribute_value, p_req->val_len );
	break;
    }
    return ( status );
}

wiced_result_t hci_control_le_write_exec_handler( uint16_t conn_id, wiced_bt_gatt_exec_flag_t flag )
{
    return ( WICED_SUCCESS );
}

wiced_result_t hci_control_le_mtu_handler( uint16_t conn_id, uint16_t mtu )
{
    le_control_cb.conn[conn_id].peer_mtu   = mtu;

    return ( WICED_SUCCESS );
}

/*
 * Process indication confirm.
 */
wiced_result_t  hci_control_le_conf_handler( uint16_t conn_id, uint16_t handle )
{
#ifdef OTA_FW_UPGRADE
    if (wiced_ota_fw_upgrade_is_gatt_handle(handle))
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    WICED_BT_TRACE( "hci_control_le_conf_handler conn_id:%d state:%d handle:%x\n", conn_id, le_control_cb.conn[conn_id].state, handle );

    return WICED_SUCCESS;
}

/*
 * This is a GATT request callback
 */
wiced_bt_gatt_status_t hci_control_le_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_req )
{
    wiced_bt_gatt_status_t result  = WICED_BT_GATT_SUCCESS;
    uint16_t               conn_id = p_req->conn_id;

    switch ( p_req->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = hci_control_le_read_handler( p_req->conn_id, &p_req->data.read_req );
            break;

        case GATTS_REQ_TYPE_WRITE:
        case GATTS_REQ_TYPE_PREP_WRITE:
            result = hci_control_le_write_handler( p_req->conn_id, &p_req->data.write_req );
             break;

        case GATTS_REQ_TYPE_WRITE_EXEC:
            result = hci_control_le_write_exec_handler( p_req->conn_id, p_req->data.exec_write );
            break;

        case GATTS_REQ_TYPE_MTU:
            result = hci_control_le_mtu_handler( p_req->conn_id, p_req->data.mtu );
            break;

        case GATTS_REQ_TYPE_CONF:
            result = hci_control_le_conf_handler( p_req->conn_id, p_req->data.handle );
            break;

       default:
            WICED_BT_TRACE("Invalid GATT request conn_id:%d type:%d\n", conn_id, p_req->request_type);
            break;
    }

    return result;
}

wiced_bt_gatt_status_t hci_control_le_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_SUCCESS;
    switch( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = hci_control_le_conn_status_callback( &p_data->connection_status );
        break;

    case GATT_OPERATION_CPLT_EVT:
#if (WICED_APP_LE_SLAVE_CLIENT_INCLUDED == TRUE)
        if (le_control_cb.conn[p_data->operation_complete.conn_id].role == HCI_ROLE_SLAVE)
            result = le_slave_gatt_operation_complete(&p_data->operation_complete);
        else
#endif
            result = hci_control_le_gatt_operation_comp_cb( &p_data->operation_complete );
        break;

    case GATT_DISCOVERY_RESULT_EVT:
#if (WICED_APP_LE_SLAVE_CLIENT_INCLUDED == TRUE)
        if (le_control_cb.conn[p_data->discovery_result.conn_id].role == HCI_ROLE_SLAVE)
            result = le_slave_gatt_discovery_result( &p_data->discovery_result );
        else
#endif
            result = hci_control_le_gatt_disc_result_cb( &p_data->discovery_result );
        break;

    case GATT_DISCOVERY_CPLT_EVT:
#if (WICED_APP_LE_SLAVE_CLIENT_INCLUDED == TRUE)
        if (le_control_cb.conn[p_data->discovery_complete.conn_id].role == HCI_ROLE_SLAVE)
            result = le_slave_gatt_discovery_complete( &p_data->discovery_complete );
        else
#endif
            result = hci_control_le_gatt_disc_comp_cb( &p_data->discovery_complete );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = hci_control_le_gatt_req_cb( &p_data->attribute_request );
        break;

    default:
        break;
    }

    return result;
}

/*
 * This function sends write to the peer GATT server
 * */
wiced_bt_gatt_status_t hci_control_le_send_write( uint8_t conn_id, uint16_t attr_handle, uint8_t *p_data, uint16_t len, wiced_bt_gatt_write_type_t type )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INSUF_RESOURCE;

    // Allocating a buffer to send the write request
    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( GATT_RESPONSE_SIZE( 2 ) );

    if ( p_write )
    {
        p_write->handle   = attr_handle;
        p_write->offset   = 0;
        p_write->len      = len;
        p_write->auth_req = GATT_AUTH_REQ_NONE;
        memcpy( p_write->value, p_data, len );

        // Register with the server to receive notification
        status = wiced_bt_gatt_send_write ( conn_id, type, p_write );

        WICED_BT_TRACE( "wiced_bt_gatt_send_write ", status );

        wiced_bt_free_buffer( p_write );
    }
    return ( status );
}

/*
 * Num complete callback
 */
void hci_control_le_num_complete_callback( void )
{
    uint16_t             handle;
    uint16_t             conn_id;
    static int           last_connection_serviced = 0;
    int                  i;

    //WICED_BT_TRACE( "hci_control_le_num_complete_callback available buffs:%d\n", wiced_bt_ble_get_available_tx_buffers( ) );

    if ( hci_control_le_pending_tx_buffer.tx_buf_saved && ( wiced_bt_ble_get_available_tx_buffers( ) > 1 ) )
    {
        WICED_BT_TRACE( "service conn_id:%d\n", hci_control_le_pending_tx_buffer.tx_buf_conn_id );

        // saved tx buffer can be write command, or notification.
        if ( hci_control_le_pending_tx_buffer.tx_buf_type == HCI_CONTROL_GATT_COMMAND_WRITE_COMMAND )
        {
            if ( hci_control_le_send_write( hci_control_le_pending_tx_buffer.tx_buf_conn_id, hci_control_le_pending_tx_buffer.tx_buf_handle,
                            hci_control_le_pending_tx_buffer.tx_buf_data, hci_control_le_pending_tx_buffer.tx_buf_len, GATT_WRITE_NO_RSP ) != WICED_BT_GATT_SUCCESS )
            {
                    return;
            }
        }
        else if ( hci_control_le_pending_tx_buffer.tx_buf_type == HCI_CONTROL_GATT_COMMAND_NOTIFY )
        {
            if ( wiced_bt_gatt_send_notification( hci_control_le_pending_tx_buffer.tx_buf_conn_id, hci_control_le_pending_tx_buffer.tx_buf_handle,
                            hci_control_le_pending_tx_buffer.tx_buf_len, hci_control_le_pending_tx_buffer.tx_buf_data ) != WICED_BT_GATT_SUCCESS )
            {
                    return;
            }
        }
        else
        {
            WICED_BT_TRACE( "bad packet queued:%d\n", hci_control_le_pending_tx_buffer.tx_buf_type );
        }
        // tx_buffer sent successfully, clear tx_buf_saved flag
        hci_control_le_pending_tx_buffer.tx_buf_saved = WICED_FALSE;
    }
}

/*
 * Stack runs the advertisement state machine switching between high duty, low
 * duty, no advertisements, based on the wiced_cfg.  All changes are notified
 * through this callback.
 */
void hci_control_le_advert_state_changed( wiced_bt_ble_advert_mode_t mode )
{
#if 0
    uint8_t hci_control_le_event;

    WICED_BT_TRACE( "Advertisement State Change:%d\n", mode );

    switch ( mode )
    {
    case BTM_BLE_ADVERT_OFF:
        hci_control_le_event = LE_ADV_STATE_NO_DISCOVERABLE;
        break;
    case BTM_BLE_ADVERT_DIRECTED_HIGH:
    case BTM_BLE_ADVERT_UNDIRECTED_HIGH:
    case BTM_BLE_ADVERT_NONCONN_HIGH:
    case BTM_BLE_ADVERT_DISCOVERABLE_HIGH:
        hci_control_le_event = LE_ADV_STATE_HIGH_DISCOVERABLE;
        break;
    case BTM_BLE_ADVERT_DIRECTED_LOW:
    case BTM_BLE_ADVERT_UNDIRECTED_LOW:
    case BTM_BLE_ADVERT_NONCONN_LOW:
    case BTM_BLE_ADVERT_DISCOVERABLE_LOW:
        hci_control_le_event = LE_ADV_STATE_LOW_DISCOVERABLE;
        break;
    }
    //hci_control_le_send_advertisement_state_event( hci_control_le_event );
#endif
}

/*
 * Stack runs the scan state machine switching between high duty, low
 * duty, no scan, based on the wiced_cfg.  All changes are notified
 * through this callback.
 */
void hci_control_le_scan_state_changed( wiced_bt_ble_scan_type_t state )
{
#if 0
    uint8_t hci_control_le_event;

    WICED_BT_TRACE( "Scan State Changed:%d\n", state );

    switch ( state )
    {
    case BTM_BLE_SCAN_TYPE_NONE:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_NO_SCAN;
        break;
    case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_HIGH_SCAN;
        break;
    case BTM_BLE_SCAN_TYPE_LOW_DUTY:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_LOW_SCAN;
        break;
    }
#endif
}

/*
 * This function is called when notification is received from the connected GATT Server
 */
void hci_control_le_notification_handler( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    WICED_BT_TRACE( "Notification conn_id:%d handle:%04x len:%d\n", conn_id, handle, len );

    wiced_bt_gatt_send_notification ( conn_id, handle, len, p_data );
}

/*
 * This function is called when indication is received from the connected GATT Server
 */
void hci_control_le_indication_handler( uint16_t conn_id, uint16_t handle, uint8_t *p_data, uint16_t len )
{
    WICED_BT_TRACE( "Indication conn_id:%d handle:%04x len:%d\n", conn_id, handle, len );

    wiced_bt_gatt_send_indication ( conn_id, handle, len, p_data );
}

void hci_control_le_indication_confirmation_handler( uint16_t conn_id, uint16_t handle )
{
    WICED_BT_TRACE("INDICATION CONFIRMATION\n");
    wiced_bt_gatt_send_indication_confirm( conn_id, handle );
}

static void hci_control_le_handle_gatt_db_init(uint8_t* data, uint32_t length)
{
    wiced_bt_gatt_status_t gatt_status;

    WICED_BT_TRACE("Initializing GATT database...\n");

    gatt_status = wiced_bt_gatt_db_init(data, length);
    if( gatt_status != WICED_BT_GATT_SUCCESS )
    {
        WICED_BT_TRACE("Error initializing GATT callback...error: %d\n", gatt_status);
        return;
    }
}
