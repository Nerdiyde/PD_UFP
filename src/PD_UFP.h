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

#ifndef PD_UFP_H
#define PD_UFP_H

#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

#define FUSB302_INT_STANDARD_PIN 7 // This is the standard interrupt pin. It can be changed during the init procedure

extern "C"
{
#include "FUSB302_UFP.h"
#include "PD_UFP_Protocol.h"
}

enum
{
    STATUS_POWER_NA = 0,
    STATUS_POWER_TYP,
    STATUS_POWER_PPS
};
typedef uint8_t status_power_t;

///////////////////////////////////////////////////////////////////////////////////////////////////
// PD_UFP_core_c
///////////////////////////////////////////////////////////////////////////////////////////////////
class PD_UFP_core_c
{
public:
    PD_UFP_core_c();
    // Init
    bool init(enum PD_power_option_t power_option = PD_POWER_OPTION_MAX_5V, uint8_t interrupt_pin = FUSB302_INT_STANDARD_PIN);
    bool init_PPS(uint16_t PPS_voltage, uint8_t PPS_current, enum PD_power_option_t power_option = PD_POWER_OPTION_MAX_5V, uint8_t interrupt_pin = FUSB302_INT_STANDARD_PIN);
    // Task
    void run(void);
    // Status
    bool is_power_ready(void) { return status_power == STATUS_POWER_TYP; }
    bool is_PPS_ready(void) { return status_power == STATUS_POWER_PPS; }
    bool is_ps_transition(void) { return send_request || wait_ps_rdy; }
    // Get
    uint16_t get_voltage(void) { return ready_voltage; } // Voltage in 50mV units, 20mV(PPS)
    uint16_t get_current(void) { return ready_current; } // Current in 10mA units, 50mA(PPS)
    // get_source_capabilities
    // Set
    bool set_PPS(uint16_t PPS_voltage, uint8_t PPS_current);
    void set_power_option(enum PD_power_option_t power_option);
    // Clock
    static void clock_prescale_set(uint8_t prescaler);

protected:
    static FUSB302_ret_t FUSB302_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count);
    static FUSB302_ret_t FUSB302_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t count);
    static FUSB302_ret_t FUSB302_delay_ms(uint32_t t);
    void handle_protocol_event(PD_protocol_event_t events);
    void handle_FUSB302_event(FUSB302_event_t events);
    bool timer(void);
    void set_default_power(void);
    // Device
    FUSB302_dev_t FUSB302;
    PD_protocol_t protocol;
    // Power ready power
    uint16_t ready_voltage;
    uint16_t ready_current;
    // PPS setup
    uint16_t PPS_voltage_next;
    uint8_t PPS_current_next;
    // Status
    virtual void status_power_ready(status_power_t status, uint16_t voltage, uint16_t current);
    uint8_t status_initialized;
    uint8_t status_src_cap_received;
    status_power_t status_power;
    // Timer and counter for PD Policy
    uint16_t time_polling;
    uint16_t time_wait_src_cap;
    uint16_t time_wait_ps_rdy;
    uint16_t time_PPS_request;
    uint8_t get_src_cap_retry_count;
    uint8_t wait_src_cap;
    uint8_t wait_ps_rdy;
    uint8_t send_request;
    uint8_t int_pin;
    static uint8_t clock_prescaler;
    // Time functions
    void delay_ms(uint16_t ms);
    uint16_t clock_ms(void);
    // Status logging
    virtual void status_log_event(uint8_t status, uint32_t *obj = 0) {}
};

///////////////////////////////////////////////////////////////////////////////////////////////////
// Optional: PD_UFP_log_c, extended from PD_UFP_c to provide logging function.
//           Asynchronous, minimal impact on PD timing.
///////////////////////////////////////////////////////////////////////////////////////////////////
struct status_log_t
{
    uint16_t time;
    uint16_t msg_header;
    uint8_t obj_count;
    uint8_t status;
};

enum pd_log_level_t
{
    PD_LOG_LEVEL_INFO,
    PD_LOG_LEVEL_VERBOSE
};

class PD_UFP_log_c : public PD_UFP_core_c
{
public:
    PD_UFP_log_c(pd_log_level_t log_level = PD_LOG_LEVEL_INFO);
    // Task
    // void print_status(Serial_ & serial);
    void print_status(HardwareSerial &serial);
    // Get
    int status_log_readline(char *buffer, int maxlen);

protected:
    int status_log_readline_msg(char *buffer, int maxlen, status_log_t *log);
    int status_log_readline_src_cap(char *buffer, int maxlen);
    // Status log functions
    uint8_t status_log_obj_add(uint16_t header, uint32_t *obj);
    virtual void status_log_event(uint8_t status, uint32_t *obj);
    // status log event queue
    status_log_t status_log[16]; // array size must be power of 2 and <=256
    uint8_t status_log_read;
    uint8_t status_log_write;
    // status log object queue
    uint32_t status_log_obj[16]; // array size must be power of 2 and <=256
    uint8_t status_log_obj_read;
    uint8_t status_log_obj_write;
    // state variables
    pd_log_level_t status_log_level;
    uint8_t status_log_counter;
    char status_log_time[8];
};

#endif
