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

#include <Wire.h>
#include <PD_UFP.h>

#define DEBUG_PD_COMMUNICATION // uncomment this to activate additional PD debug output

#ifdef DEBUG_PD_COMMUNICATION
class PD_UFP_log_c PD_UFP(PD_LOG_LEVEL_VERBOSE);
#else
class PD_UFP_core_c PD_UFP;
#endif

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  PD_UFP.init(PD_POWER_OPTION_MAX_20V);
}

void loop()
{
  PD_UFP.run();
  if (PD_UFP.is_power_ready())
  {
    if (PD_UFP.get_voltage() == PD_V(20.0) &&
        PD_UFP.get_current() >= PD_A(1.5))
    {
      Serial.println("Current PD values: Voltage: 20V; Current:>=1.5A");
    }
  }
}
