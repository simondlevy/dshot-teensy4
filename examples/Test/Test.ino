#include <dshot-teensy4.hpp>

// Motors
static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});

static float _motor_pwms[4];

static void armMotors() {

    for (int i = 0; i <= 50; i++) {
        _motors.run(armedFly, _motor_pwms);
         delay(2);
    }
}

static void loopRate(int freq) {

    float invFreq = 1.0/freq*1000000.0;
    unsigned long checker = micros();

    while (invFreq > (checker - current_time)) {
        checker = micros();
    }
}

void setup()
{
    Serial.begin(500000); 

    _motor_pwms[0] = 125; 
    _motor_pwms[1] = 125;
    _motor_pwms[2] = 125;
    _motor_pwms[3] = 125;

    armMotors(); 
}

void loop()
{
    _motors.run(armedFly, _motor_pwms);

    loopRate(2000); 
}

