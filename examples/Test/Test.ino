#include <dshot-teensy4.hpp>  

static DshotTeensy4 _motors = DshotTeensy4({6, 5, 4, 3});

static uint32_t current_time, prev_time;

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

    delay(500);

    _motors.arm(); 
}

void loop()
{
    prev_time = current_time;      
    current_time = micros();      

    float motorvals[4] = {};

    _motors.run(false, motorvals);

    loopRate(2000); 
}

