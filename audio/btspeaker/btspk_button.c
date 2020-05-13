/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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
/** @btspk_button.c
 *
 * BT Speaker Pro Sample Application for the Audio Shield platform.
 *
 * The sample app demonstrates Bluetooth A2DP sink, HFP and AVRCP Controller (and Target for absolute volume control).
 *
 * Features demonstrated
 *  - A2DP Sink and AVRCP Controller (Target for absolute volume)
 *  - Handsfree Device
 *  - GATT
 *  - SDP and GATT descriptor/attribute configuration
 *  - This app is targeted for the Audio Shield platform
 *  - This App doesn't support HCI UART for logging, PUART is supported.
 *  - HCI Client Control is not supported.
 *
 * Setting up Connection
 * 1. press and hold the SW15 on BT board for at least 2 seconds.
 * 2. This will set device in discovery mode(A2DP,HFP and BLE) and LED will start blinking.
 * 3. Scan for 'BTSpeakerPro' device on the peer source device, and pair with the BTSpeakerPro.
 * 4. Once connected LED will stop blinking and turns on.
 * 5. If no connection is established within 30sec,LED will turn off and device is not be discoverable,repeat instructions from step 1 to start again.
 *
 * A2DP Play back
 * 1. Start music play back from peer device, you should be able to hear music from Speaker (use J39 speaker terminal)
 * 2. You can control play back and volume from peer device (Play, Pause, Stop) controls.
 *
 * AVRCP
 * 1. We can use buttons connected to the BT EVAL board for AVRCP control
 * 2. SW15 - Discoverable/Play/Pause    - Long press this button to enter discoverable mode. Click the button to Play/Pause the music play back.
 * 3. SW16 -                            - No function
 * 4. SW17 - Volume Up/Forward          - Click this button to increase volume or long press the button to forward
 * 5. SW18 - Volume Down/Backward       - Click this button to decrease volume or long press the button to backward
 *                                      (There are 16 volume steps).
 * 6. SW19 - Voice Recognition          - Long press to voice control
 *
 * Hands-free
 * 1. Make a phone call to the peer device.
 * 2. In case of in-band ring mode is supported from peer device, you will hear the set ring tone
 * 3. In case of out-of-band ring tone, no tone will be heard on speaker.
 * 4. SW15  is used as multi-function button to accept,hang-up or reject call.
 * 5. Long press SW15 to reject the incoming call.
 * 6. Click SW15 to accept the call or hang-up the active call.
 * 7. If the call is on hold click SW15 to hang-up the call.
 * 8. Every click of SW17(Volume Up) button will increase the volume
 * 9. Every click of SW18(Volume down) button will decrease the volume
 *
 * BLE
 *  - To connect Ble device: set bt speaker in discovery mode by long press of SW15 button
 *    search for 'BTSpeakerPro' device in peer side phone app (Ex:BLEScanner for Android and LightBlue for iOS) and connect.
 *  - From the peer side app you should be able to do GATT read/write of the elements listed.
 */

#include "wiced.h"
#include "bt_hs_spk_control.h"
#include "bt_hs_spk_button.h"
#include "wiced_button_manager.h"

/******************************************************
 *                      Macros
 ******************************************************/

static wiced_button_manager_configuration_t app_button_manager_configuration =
{
    .short_hold_duration     = 500, /*msec*/
    .medium_hold_duration    = 700,
    .long_hold_duration      = 1000,
    .very_long_hold_duration = 1500,
    .debounce_duration       = 150, /* typically a click takes around ~150-200 ms */
    .continuous_hold_detect  = WICED_FALSE,
    /*if NULL button events are handled by bt_hs_spk library*/
    .event_handler = NULL,
};

/* Static button configuration */
static wiced_button_configuration_t app_button_configurations[] =
{
#if defined(CYW43012C0)
    [ PLAY_PAUSE_BUTTON ]                   = { PLATFORM_BUTTON_1, BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_VERY_LONG_DURATION_EVENT , 0 },
#else
    [ PLAY_PAUSE_BUTTON ]                   = { PLATFORM_BUTTON_1, BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_VERY_LONG_DURATION_EVENT , 0 },
    [ VOLUME_UP_NEXT_TRACK_BUTTON ]         = { PLATFORM_BUTTON_2, BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_VERY_LONG_DURATION_EVENT , 0 },
    [ VOLUME_DOWN_PREVIOUS_TRACK_BUTTON ]   = { PLATFORM_BUTTON_3, BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_VERY_LONG_DURATION_EVENT , 0 },
    [ VOICE_REC_BUTTON ]                    = { PLATFORM_BUTTON_4, BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_VERY_LONG_DURATION_EVENT, 0 },
#endif
};

/* Button objects for the button manager */
button_manager_button_t app_buttons[] =
{
#if defined(CYW43012C0)
    [ PLAY_PAUSE_BUTTON ]                   = { &app_button_configurations[ PLAY_PAUSE_BUTTON ]        },
#else
    [ PLAY_PAUSE_BUTTON ]                   = { &app_button_configurations[ PLAY_PAUSE_BUTTON ]        },
    [ VOLUME_UP_NEXT_TRACK_BUTTON ]         = { &app_button_configurations[ VOLUME_UP_NEXT_TRACK_BUTTON ]     },
    [ VOLUME_DOWN_PREVIOUS_TRACK_BUTTON ]   = { &app_button_configurations[ VOLUME_DOWN_PREVIOUS_TRACK_BUTTON ]  },
    [ VOICE_REC_BUTTON ]                    = { &app_button_configurations[ VOICE_REC_BUTTON ] },
#endif
};

static button_manager_t app_button_manager;

static bt_hs_spk_button_action_t app_button_action[] =
{
#if defined(CYW43012C0)
    /* PLAY_PAUSE_BUTTON */
    {
        .action = ACTION_PAUSE_PLAY,
        .button = PLAY_PAUSE_BUTTON,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = ACTION_BT_DISCOVERABLE,
        .button = PLAY_PAUSE_BUTTON,
        .event  = BUTTON_VERY_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_HELD,
    },
#else
    /* PLAY_PAUSE_BUTTON */
    {
        .action = ACTION_PAUSE_PLAY,
        .button = PLAY_PAUSE_BUTTON,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = ACTION_BT_DISCOVERABLE,
        .button = PLAY_PAUSE_BUTTON,
        .event  = BUTTON_VERY_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_HELD,
    },

    /* VOLUME_UP_NEXT_TRACK_BUTTON */
    {
        .action = ACTION_VOLUME_UP,
        .button = VOLUME_UP_NEXT_TRACK_BUTTON,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = ACTION_FORWARD,
        .button = VOLUME_UP_NEXT_TRACK_BUTTON,
        .event  = BUTTON_VERY_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },

    /* VOLUME_DOWN_PREVIOUS_TRACK_BUTTON */
    {
        .action = ACTION_VOLUME_DOWN,
        .button = VOLUME_DOWN_PREVIOUS_TRACK_BUTTON,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
    {
        .action = ACTION_BACKWARD,
        .button = VOLUME_DOWN_PREVIOUS_TRACK_BUTTON,
        .event  = BUTTON_VERY_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },

    /* VOICE_REC_BUTTON */
    {
        .action = ACTION_VOICE_RECOGNITION,
        .button = VOICE_REC_BUTTON,
        .event  = BUTTON_LONG_DURATION_EVENT,
        .state  = BUTTON_STATE_HELD,
    },
    {
        .action = ACTION_TRANSPORT_DETECT_ON,
        .button = VOICE_REC_BUTTON,
        .event  = BUTTON_CLICK_EVENT,
        .state  = BUTTON_STATE_RELEASED,
    },
#endif
};

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t btspk_init_button_interface(void)
{
    wiced_result_t result;
    bt_hs_spk_button_config_t config;

    config.p_manager                                = &app_button_manager;
    config.p_configuration                          = &app_button_manager_configuration;
    config.p_app_buttons                            = app_buttons;
    config.number_of_buttons                        = ARRAY_SIZE(app_buttons);
    config.p_pre_handler                            = NULL;
    config.button_action_config.p_action            = app_button_action;
    config.button_action_config.number_of_actions   = ARRAY_SIZE(app_button_action);

    result = bt_hs_spk_init_button_interface(&config);
    return result;
}
