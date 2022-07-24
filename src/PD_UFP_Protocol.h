/*                               | \ | |               | |(_)             | |
  __      ____      ____      __ |  \| |  ___  _ __  __| | _  _   _     __| |  ___
  \ \ /\ / /\ \ /\ / /\ \ /\ / / | . ` | / _ \| '__|/ _` || || | | |   / _` | / _ \
   \ V  V /  \ V  V /  \ V  V /_ | |\  ||  __/| |  | (_| || || |_| | _| (_| ||  __/
    \_/\_/    \_/\_/    \_/\_/(_)|_| \_| \___||_|   \__,_||_| \__, |(_)\__,_| \___|
                                                               __/ |
                                                              |___/
    PD_UFP library by Fabian Steppat
    Contact: nerdiy.de

    This is a PD and PPS library which makes it possible to request PD profiles or PPS progs from a suitable Power Delivery capable power supply.
    The implementation is heavily based on the PD_Micro project by Ryan Ma (https://github.com/ryan-ma/PD_Micro). Most credits belong to him!
    It's just reworked a bit so that it can be used by other HW setups.
    So far this library is mostly tested on a custom PCB containing an ESP32 and an FUSB302 PD phy.

    ============================================
    
    MIT License

    Copyright (c) 2020 Ryan Ma

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef PD_UFP_PROTOCOL_H
#define PD_UFP_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/* For use in PD_protocol_get_power_info() */
#define PD_V(v)     ((uint16_t)(v * 20 + 0.01))
#define PD_A(a)     ((uint16_t)(a * 100 + 0.01))

/* For use in PD_protocol_set_PPS_option() */
#define PPS_V(v)    ((uint16_t)(v * 50 + 0.01))
#define PPS_A(a)    ((uint8_t)(a * 20 + 0.01))

#define PD_PROTOCOL_MAX_NUM_OF_PDO      7

#define PD_PROTOCOL_EVENT_SRC_CAP       (1 << 0)
#define PD_PROTOCOL_EVENT_PS_RDY        (1 << 1)
#define PD_PROTOCOL_EVENT_ACCEPT        (1 << 2)
#define PD_PROTOCOL_EVENT_REJECT        (1 << 3)
#define PD_PROTOCOL_EVENT_PPS_STATUS    (1 << 4)

typedef uint8_t PD_protocol_event_t;

enum PD_power_option_t {
    PD_POWER_OPTION_MAX_5V      = 0,
    PD_POWER_OPTION_MAX_9V      = 1,
    PD_POWER_OPTION_MAX_12V     = 2,
    PD_POWER_OPTION_MAX_15V     = 3,
    PD_POWER_OPTION_MAX_20V     = 4,
    PD_POWER_OPTION_MAX_VOLTAGE = 5,
    PD_POWER_OPTION_MAX_CURRENT = 6,
    PD_POWER_OPTION_MAX_POWER   = 7,
};

enum PD_power_data_obj_type_t {   /* Power data object type */
    PD_PDO_TYPE_FIXED_SUPPLY    = 0,
    PD_PDO_TYPE_BATTERY         = 1,
    PD_PDO_TYPE_VARIABLE_SUPPLY = 2,
    PD_PDO_TYPE_AUGMENTED_PDO   = 3     /* USB PD 3.0 */
};

enum PPS_PTF_t {
    PPS_PTF_NOT_SUPPORT         = 0,
    PPS_PTF_NORMAL              = 1,
    PPS_PTF_WARNING             = 2,
    PPS_PTF_OVER_TEMPERATURE    = 3
};

enum PPS_OMF_t {
    PPS_OMF_VOLTAGE_MODE        = 0,
    PPS_OMF_CURRENT_LIMIT_MODE  = 1
};

typedef struct {
    uint16_t output_voltage;    /* Voltage in 20mV units, 0xFFFF if not supported */
    uint8_t output_current;     /* Current in 50mV units, 0xFF if not supported */
    enum PPS_PTF_t flag_PTF;
    enum PPS_OMF_t flag_OMF;
} PPS_status_t;

typedef struct {
    const char * name;
    uint8_t id;
    uint8_t spec_rev;
    uint8_t num_of_obj;
    uint8_t extended;
} PD_msg_info_t;

typedef struct {
    enum PD_power_data_obj_type_t type;
    uint16_t min_v;     /* Voltage in 50mV units */
    uint16_t max_v;     /* Voltage in 50mV units */
    uint16_t max_i;     /* Current in 10mA units */
    uint16_t max_p;     /* Power in 250mW units */
} PD_power_info_t;

struct PD_msg_state_t;
typedef struct {
    const struct PD_msg_state_t *msg_state;
    uint16_t tx_msg_header;
    uint16_t rx_msg_header;
    uint8_t message_id;

    uint16_t PPS_voltage;
    uint8_t PPS_current;
    uint8_t PPSSDB[4];  /* PPS Status Data Block */

    enum PD_power_option_t power_option;
    uint32_t power_data_obj[PD_PROTOCOL_MAX_NUM_OF_PDO];
    uint8_t power_data_obj_count;
    uint8_t power_data_obj_selected;
} PD_protocol_t;

/* Message handler */
void PD_protocol_handle_msg(PD_protocol_t *p, uint16_t header, uint32_t *obj, PD_protocol_event_t *events);
bool PD_protocol_respond(PD_protocol_t *p, uint16_t *h, uint32_t *obj);

/* PD Message creation */
void PD_protocol_create_get_src_cap(PD_protocol_t *p, uint16_t *header);
void PD_protocol_create_get_PPS_status(PD_protocol_t *p, uint16_t *header);
void PD_protocol_create_request(PD_protocol_t *p, uint16_t *header, uint32_t *obj);

/* Get functions */
static inline uint8_t  PD_protocol_get_selected_power(PD_protocol_t *p) { return p->power_data_obj_selected; }
static inline uint16_t PD_protocol_get_PPS_voltage(PD_protocol_t *p) { return p->PPS_voltage; } /* Voltage in 20mV units */
static inline uint8_t  PD_protocol_get_PPS_current(PD_protocol_t *p) { return p->PPS_current; } /* Current in 50mA units */

static inline uint16_t PD_protocol_get_tx_msg_header(PD_protocol_t *p) { return p->tx_msg_header; }
static inline uint16_t PD_protocol_get_rx_msg_header(PD_protocol_t *p) { return p->rx_msg_header; }

bool PD_protocol_get_msg_info(uint16_t header, PD_msg_info_t * msg_info);

bool PD_protocol_get_power_info(PD_protocol_t *p, uint8_t index, PD_power_info_t *power_info);
bool PD_protocol_get_PPS_status(PD_protocol_t *p, PPS_status_t * PPS_status);

/* Set Fixed and Variable power option */
bool PD_protocol_set_power_option(PD_protocol_t *p, enum PD_power_option_t option);
bool PD_protocol_select_power(PD_protocol_t *p, uint8_t index);

/* Set PPS Voltage in 20mV units, Current in 50mA units. return true if re-send request is needed
   strict=true, If PPS setting is not qualified, return false, nothing is changed.
   strict=false, if PPS setting is not qualified, fall back to regular power option */
bool PD_protocol_set_PPS(PD_protocol_t * p, uint16_t PPS_voltage, uint8_t PPS_current, bool strict);  

void PD_protocol_reset(PD_protocol_t *p);
void PD_protocol_init(PD_protocol_t *p);

#endif
