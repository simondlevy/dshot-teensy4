/*
   Test ESCs

   This file is part of DSHOT-Teensy4.

   DSHOT-Teensy4 is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   DSHOT-Teensy4 is distributed in the hope that it will be useful, but
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
   or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   DSHOT-Teensy4. If not, see <https://www.gnu.org/licenses/>.
 */

#include <dshot-teensy4.hpp>
#include <vector>

#include "input_keyboard.hpp"

static const std::vector<uint8_t> PINS = {6, 5, 4, 3};

static float MAX = 0.5;
static float INC = 1e-5;

static bool running;

static float val;
static float dir;

static auto motors = DShotTeensy4(PINS);

void serialEvent()
{
  while (Serial.available()) {
    Serial.read();
  }

  running = !running;
}

static void prompt()
{
    static uint32_t tprev;
    const uint32_t tcurr = millis();
    if (tcurr - tprev > 1000) {
      tprev = tcurr;
      Serial.print("Hit Enter to ");
      Serial.println(running ? "stop" : "start");
    }
}

static float inputGet()
{
    prompt();

    if (running) {

        val += INC * dir;

        if (val >= MAX) {
            dir = -1;
        }

        if (val <= 0) {
            dir = +1;
        }
    }
    else {
        val = 0;
    }

    return val;
}

static void inputInit(void)
{
    dir = +1;
}

void setup() 
{
    Serial.begin(115200);

}

void loop() 
{
    /*
    auto pulseWidth = (uint8_t)(125 * (inputGet() + 1));

    for (uint8_t k=0; k<PINS.size(); ++k) {
        motors.set(k, pulseWidth);
    }

    motors.run();*/
}
