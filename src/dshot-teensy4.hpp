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

class DshotTeensy4 {

    public:

        typedef enum {

            DSHOT_300,
            DSHOT_600

        } dshot_type_e;

        /*
         * idle_throttle needs to be high enough to avoid desyncs, low enough
         * to avoid liftoff. Somewhere between 50 and 400 - offset: adjustment
         * to DSHOT short pulse in case ESC rejects frames.
         */
        DshotTeensy4(
                const std::vector<uint8_t> pins,
                const dshot_type_e dshot_type=DSHOT_600,
                const uint16_t idle_throttle=100
                )
        {
            for (auto pin : pins) {
                _pins.push_back(pin);
                _frames.push_back(0);
                _is1s.push_back(0);
                _armingPwms.push_back(125);
                pinMode(pin, OUTPUT);
            }

            _idle_throttle = idle_throttle;

            const float div = dshot_type == DSHOT_600 ? 2 : 1;

             // Relies on teensy4.0 ARM_DWT_CYCCNT CPU cycle counter.
             // 1 cycle ≈ 1.67 nanoseconds
            _stepsfor0high = (F_CPU_ACTUAL * 1.25 / div) / 1'000'000;
            _stepsfor1high = (F_CPU_ACTUAL * 2.50 / div) / 1'000'000;
            _stepsforbit = (F_CPU_ACTUAL * 3.33 / div) / 1'000'000;
        }

        void arm() {

            for (uint8_t i=0; i <=50; i++) {
                run(false, _armingPwms.data());
                delay(2);
            }
        }

        void run(const bool armed, const float * pwms)
        {
            static bool cycle_ctr_enabled = false;

            if (!cycle_ctr_enabled) {
                ARM_DEMCR |= ARM_DEMCR_TRCENA;
                ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
                cycle_ctr_enabled = true; // set flag so it won’t run again
            }

            //uint32_t offset = 40; // tuned adjustment for minimizing out-of-frame errors
            // depending on ESC, range is somewhere between 20 and 80

            // Prepare DShot frames for all motors.
            for (uint8_t k=0; k<_pins.size(); ++k) {
                _frames[k] = calc_dshot_frame(pwms[k], armed, _idle_throttle);
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

                uint32_t timeout0high = bit_start_cycle + _stepsfor0high; // start + 375
                uint32_t timeout1high = bit_start_cycle + _stepsfor1high; // start + 750
                uint32_t timeoutbit = bit_start_cycle + _stepsforbit; // start + 1002

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

                bit_start_cycle += _stepsforbit; // Advance to the start time of the next bit

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

        std::vector<float> _armingPwms;

        uint16_t _idle_throttle;

        uint32_t _stepsfor0high; 
        uint32_t _stepsfor1high;
        uint32_t _stepsforbit; 

        static void pin_down(const uint8_t pin)
        {
            *portClearRegister(pin) = digitalPinToBitMask(pin);
        }

        static float clamp(
                const float val, const float minv, const float maxv) 
        {
            return val < minv ? minv : val > maxv ? maxv : val;
        }

        static float map_range(const float in,
                const float in_min, const float in_max,
                const float out_min, const float out_max)
        {
            return out_min + ((clamp(in, in_min, in_max) - in_min) *
                    (out_max - out_min)) / (in_max - in_min);
        }

        static uint16_t calc_dshot_frame(
                const float in, const bool armed, const uint16_t idle)
        {
            const uint16_t throttle = !armed ? 0 :
                round(map_range(
                            in,
                            0,
                            1,
                            48 + idle,
                            2047));

            const uint16_t frame = (throttle << 1) | 0; // [11 bits throttle][1 bit telemetry]

            const uint8_t crc = (frame ^ (frame >> 4) ^ (frame >> 8)) & 0x0F; // [4 bits CRC]

            return (frame << 4) | crc; // [16 bits dshot frame]
        }

};
