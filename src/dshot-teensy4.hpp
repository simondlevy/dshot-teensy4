/*
   DSHOT for Teensy4

   Adapted from https://www.rcgroups.com/forums/showpost.php?p=54235485

   Copyright (C) 2026 Simon D. Levy

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, in version 3.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

#include <Arduino.h>

#include <vector>

// throttle ranges
#define DSHOT_MIN_THROTTLE 48
#define DSHOT_MAX_THROTTLE 2047
#define ONESHOT_MIN_THROTTLE 125
#define ONESHOT_MAX_THROTTLE 250

/*
 * Can be configured at DSHOT 300, 600 speeds
 * Defaults to DSHOT300 - sufficient for the default 2k loop speed
 * Tunable:
 */


class DshotTeensy4 {

    public:

        /*
         * idle_throttle needs to be high enough to avoid desyncs, low enough
         * to avoid liftoff. Somewhere between 50 and 400 - offset: adjustment
         * to DSHOT short pulse in case ESC rejects frames.
         */

        DshotTeensy4(
                const std::vector<uint8_t> pins,
                const uint16_t idle_throttle=100
                )
        {
            for (auto pin : pins) {
                _pins.push_back(pin);
                _frames.push_back(0);
                _is1s.push_back(0);
                pinMode(pin, OUTPUT);
            }

            _idle_throttle = idle_throttle;
        }

        void run(const bool armedFly, const float * pwms)
        {
            static bool cycle_ctr_enabled = false;

            if (!cycle_ctr_enabled) {
                ARM_DEMCR |= ARM_DEMCR_TRCENA;
                ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
                cycle_ctr_enabled = true; // set flag so it won’t run again
            }

            // DSHOT timers
            // In DSHOT the high time for a 1 is always double that of a 0.
            uint32_t cpuHz = F_CPU_ACTUAL;

            // DSHOT300 - should be sufficient for dRehmFlight
            //uint32_t stepsfor0high = (cpuHz * 1.25f) / 1'000'000;
            //uint32_t stepsfor1high = (cpuHz * 2.50f) / 1'000'000;
            //uint32_t stepsforbit = (cpuHz * 3.33f) / 1'000'000;
            //uint32_t offset = 80; // tuned adjustment for minimizing out-of-frame errors
            // depending on ESC, range is somewhere between 0 and 160

            // DSHOT600 - faster, but less robust.
            uint32_t stepsfor0high = (cpuHz * 0.625f) / 1'000'000;
            uint32_t stepsfor1high = (cpuHz * 1.25f) / 1'000'000;
            uint32_t stepsforbit = (cpuHz * 1.67) / 1'000'000;
            //uint32_t offset = 40; // tuned adjustment for minimizing out-of-frame errors
            // depending on ESC, range is somewhere between 20 and 80

            /*
             * relies on teensy4.0 ARM_DWT_CYCCNT CPU cycle counter. 1 cycle ≈ 1.67 nanoseconds
             */

            // Prepare DShot frames for all motors.
            for (uint8_t k=0; k<_pins.size(); ++k) {
                _frames[k] = calc_dshot_frame(pwms[k], armedFly);
            }

            noInterrupts();

            uint32_t bit_start_cycle = ARM_DWT_CYCCNT;

            // In DSHOT the high time for a 1 is always double that of a 0.

            // Iterate through each of the 16 bits in the DShot frame, from MSB to LSB.
            for (int i = 15; i >= 0; --i) {

                // start high pulses
                for (auto pin : _pins) {
                    *portSetRegister(pin) = digitalPinToBitMask(pin);
                }

                // adjust the duration of the 0 high pulse ( short pulse ) to
                // exclude the time it took to set pins high if included, under
                // DSHOT600, 0 high pulses are too long, resulting in frame
                // rejection
                uint32_t offset = ARM_DWT_CYCCNT - bit_start_cycle;

                uint32_t timeout0high = bit_start_cycle + stepsfor0high; // start + 375
                uint32_t timeout1high = bit_start_cycle + stepsfor1high; // start + 750
                uint32_t timeoutbit = bit_start_cycle + stepsforbit; // start + 1002

                // is 'i'th bit 0 or 1
                for (uint8_t k=0; k<_pins.size(); ++k) {
                    _is1s[k] = ((_frames[k] >> i) & 1);
                }

                // busy wait until the 0 high pulses are complete
                while (ARM_DWT_CYCCNT < (timeout0high - offset)) {;} 

                // end signal for 0 high pulses
                for (uint8_t k=0; k<_pins.size(); ++k) {
                    if (!_is1s[k]) {
                        pin_down(_pins[k]);
                    };
                }

                // busy wait until the 1 high pulses are complete
                while (ARM_DWT_CYCCNT < timeout1high) {;} 

                // end signal for 1 high pulses
                for (uint8_t k=0; k<_pins.size(); ++k) {
                    if (_is1s[k]) {
                        pin_down(_pins[k]);
                    };
                }

                bit_start_cycle += stepsforbit; // Advance to the start time of the next bit

                // busy wait until the 1 high pulses are complete ( 1.67
                // microseconds / 1002 cycles )
                while (ARM_DWT_CYCCNT < timeoutbit) {;} 

            }
            interrupts();
        }

    private:

        std::vector<uint8_t> _pins;

        std::vector<uint16_t> _frames;

        std::vector<uint16_t> _is1s;

        uint16_t _idle_throttle;

        static void pin_down(const uint8_t pin)
        {
            *portClearRegister(pin) = digitalPinToBitMask(pin);
        }

        static float clamp(float val, float minv, float maxv) 
        {
            if (val < minv) return minv;
            if (val > maxv) return maxv;
            return val;
        }

        static float map_range(float in, float in_min, float in_max,
                float out_min, float out_max)
        {
            return out_min + ((clamp(in, in_min, in_max) - in_min) *
                    (out_max - out_min)) / (in_max - in_min);
        }

        uint16_t calc_dshot_frame(float in, const bool armedFly)
        {

            uint16_t throttle = 0;
            uint16_t frame = 0;
            uint8_t crc = 0;

            if (!armedFly) {
                throttle = 0; // disarm.
            } else {
                throttle = round(map_range(
                            in,
                            ONESHOT_MIN_THROTTLE,
                            ONESHOT_MAX_THROTTLE,
                            DSHOT_MIN_THROTTLE + _idle_throttle,
                            DSHOT_MAX_THROTTLE));
            }

            frame = (throttle << 1) | 0; // [11 bits throttle][1 bit telemetry]
            crc = (frame ^ (frame >> 4) ^ (frame >> 8)) & 0x0F; // [4 bits CRC]

            return (frame << 4) | crc; // [16 bits dshot frame]
        }

};
