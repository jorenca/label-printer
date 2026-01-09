#include <Arduino.h>
#include <Servo.h>





int linearMapPercentage(float percent, int servoMinDeg, int servoMaxDeg) {
    return map(
        percent * 10000,
        0 * 10000,
        1 * 10000,
        servoMinDeg,
        servoMaxDeg
    );
  
}


int scotchYokeMapPercentage(float percent, int servoMinDeg, int servoMaxDeg) {
    
//    float minTheta = servoMinDeg * PI / 180.0 - (PI / 2.0f); 
//    float maxTheta = servoMaxDeg * PI / 180.0 - (PI / 2.0f);
//    float minDist = sin(minTheta);
//    float maxDist = sin(maxTheta);
//    float totalTravel = maxDist - minDist;
//    float travelDist = minDist + percent * totalTravel;

        // Inverse Scotch yoke: linear position -> crank angle
    // θ in range [-π/2, +π/2]
    float theta = asin(2.0f * percent - 1.0f);

    // Normalize angle to [0.0, 1.0]
    float angleNorm = (theta + (PI / 2.0f)) / PI;

    // Map angle to servo degrees
    // Assuming min and max servo degrees correspond to bottom and top positions
    return linearMapPercentage(angleNorm, servoMinDeg, servoMaxDeg);
    //return (theta + (PI / 2.0f)) * 180.0 / PI;
  
}


class ServoStepper {
public:
    // Public endstop indicators
    bool endstopPlus  = false;
    bool endstopMinus = false;

    // Constructor
    ServoStepper(
        uint8_t servoPin,
        int (*mappingStrategyFn)(float, int, int),
        int servoMinDeg = 0,
        int servoMaxDeg = 180,
        int totalSteps = 100
    )
        : _pin(servoPin),
          _mappingStrategy(mappingStrategyFn),
          _servoMin(servoMinDeg),
          _servoMax(servoMaxDeg),
          _totalSteps(totalSteps)
    {
    }

    void begin() {
        _servo.attach(_pin);
        step(0);
    }

    void step(int32_t stepsCount) {

        float percent = (float)stepsCount / (float)_totalSteps;
        
        endstopPlus = stepsCount >= _totalSteps;
        endstopMinus = stepsCount <= 0;

        int currentAngleDeg = (*_mappingStrategy)(
            percent,
            _servoMin,
            _servoMax
        );
        
        _servo.write(currentAngleDeg);
    }

private:
    Servo _servo;
    uint8_t _pin;

    int _servoMin;
    int _servoMax;
    int _totalSteps;
    int (*_mappingStrategy)(float, int, int);

};

//////////////////////////////////////////////

#define CHAN_0_STEP_PIN 3  // needs interrupt pin
#define CHAN_0_DDR DDRD
#define CHAN_0_DIR_PORT PIND
#define CHAN_0_DIR_PIN PD6
#define CHAN_0_ENDSTOP_PIN 8

#define CHAN_0_SERVO_OUT_PIN 9

#define CHAN_1_STEP_PIN 2  // needs interrupt pin
#define CHAN_1_DDR DDRD
#define CHAN_1_DIR_PORT PIND
#define CHAN_1_DIR_PIN PD5
#define CHAN_1_ENDSTOP_PIN 7

#define CHAN_1_SERVO_OUT_PIN 10

#define CHAN_0_TOTAL_STEPS 460
#define CHAN_1_TOTAL_STEPS 100

volatile int32_t stepCountA = 0;
volatile int32_t stepCountB = 0;

void ISR_stepA() {
    // digitalRead is too slow?
    //bool dir = digitalRead(CHAN_0_DIR_PIN);
    bool dir = CHAN_0_DIR_PORT & _BV(CHAN_0_DIR_PIN);
    
    //constrain() is not ISR-safe (it’s a macro with comparisons).
    //stepCountA = constrain(stepCountA, 0, CHAN_0_TOTAL_STEPS);
    if (dir) {
        if (stepCountA < CHAN_0_TOTAL_STEPS) stepCountA++;
    } else {
        if (stepCountA > 0) stepCountA--;
    }
}

void ISR_stepB() {
    // digitalRead is too slow?
    //bool dir = digitalRead(CHAN_1_DIR_PIN);
    bool dir = CHAN_1_DIR_PORT & _BV(CHAN_1_DIR_PIN);

    
    //constrain() is not ISR-safe (it’s a macro with comparisons).
    //stepCountB = constrain(stepCountB, 0, CHAN_1_TOTAL_STEPS);
    if (dir) {
        if (stepCountB < CHAN_1_TOTAL_STEPS) stepCountB++;
    } else {
        if (stepCountB > 0) stepCountB--;
    }
}


ServoStepper chanAServo(
    CHAN_0_SERVO_OUT_PIN,
    scotchYokeMapPercentage, //linearMapPercentage,
    0, 180,
    CHAN_0_TOTAL_STEPS
);

ServoStepper chanBServo(
    CHAN_1_SERVO_OUT_PIN,
    linearMapPercentage,
    150, 60,
    CHAN_1_TOTAL_STEPS
);


void setup() {
    
    pinMode(CHAN_0_STEP_PIN, INPUT);
    pinMode(CHAN_1_STEP_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(CHAN_0_STEP_PIN), ISR_stepA, RISING);
    attachInterrupt(digitalPinToInterrupt(CHAN_1_STEP_PIN), ISR_stepB, RISING);
    
    CHAN_0_DDR &= ~_BV(CHAN_0_DIR_PIN);   // clear bit → INPUT
    CHAN_1_DDR &= ~_BV(CHAN_1_DIR_PIN);   // clear bit → INPUT
    
    pinMode(CHAN_0_ENDSTOP_PIN, OUTPUT);
    pinMode(CHAN_1_ENDSTOP_PIN, OUTPUT);
    
    chanAServo.begin();
    chanBServo.begin();
}


void loop() {

    noInterrupts();
    int32_t stepsA = stepCountA;
    int32_t stepsB = stepCountB;
    interrupts();
    
    chanAServo.step(stepsA);
    chanBServo.step(stepsB);
    
    digitalWrite(CHAN_0_ENDSTOP_PIN, chanAServo.endstopMinus || chanAServo.endstopPlus);
    digitalWrite(CHAN_1_ENDSTOP_PIN, chanBServo.endstopMinus || chanBServo.endstopPlus);
    delay(20);

}
