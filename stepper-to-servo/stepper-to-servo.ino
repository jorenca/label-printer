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

    // Inverse Scotch yoke: linear position -> crank angle
    // θ in range [-π/2, +π/2]
    float theta = asin(2.0f * percent - 1.0f);

    // Normalize angle to [0.0, 1.0]
    float angleNorm = (theta + (PI / 2.0f)) / PI;

    // Map normalized angle to servo degrees
    return servoMinDeg +
           (int)((servoMaxDeg - servoMinDeg) * angleNorm);
  
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
        float defaultStepPerc = 0.005
    )
        : _pin(servoPin),
          _mappingStrategy(mappingStrategyFn),
          _servoMin(servoMinDeg),
          _servoMax(servoMaxDeg),
          _defaultStepPercent(defaultStepPerc)
    {
        _currentAnglePerc = 0;
    }

    void begin() {
        _servo.attach(_pin);
        step(false, 0);
    }
    
    void step(bool dir) {
        step(dir, _defaultStepPercent);
    }
  
    // Step like a stepper motor
    void step(bool dir, float deltaPercent) {

        if (dir) {
            if (_currentAnglePerc < 1.0) {
                _currentAnglePerc+=deltaPercent;
            }
        } else {
            if (_currentAnglePerc > 0.0) {
                _currentAnglePerc-=deltaPercent;
            }
        }
        
        updateServo();
    }

private:
    Servo _servo;
    uint8_t _pin;

    int _servoMin;
    int _servoMax;
    float _defaultStepPercent;
    float _currentAnglePerc;
    int (*_mappingStrategy)(float, int, int);

    void updateServo() {
      
        _currentAnglePerc = _currentAnglePerc < 0.0 ? 0.0 : _currentAnglePerc;
        _currentAnglePerc = _currentAnglePerc > 1.0 ? 1.0 : _currentAnglePerc;
        
        endstopPlus = _currentAnglePerc > 0.999;
        endstopMinus = _currentAnglePerc < 0.001;

        int currentAngleDeg = (*_mappingStrategy)(
            _currentAnglePerc,
            _servoMin,
            _servoMax
        );
        
        _servo.write(currentAngleDeg);
    }
};

//////////////////////////////////////////////

#define CHAN_0_STEP_PIN 3
#define CHAN_0_DIR_PIN 4
#define CHAN_0_ENDSTOP_PIN 5

#define CHAN_0_SERVO_OUT_PIN 9

#define CHAN_1_STEP_PIN 6
#define CHAN_1_DIR_PIN 7
#define CHAN_1_ENDSTOP_PIN 8

#define CHAN_1_SERVO_OUT_PIN 10


ServoStepper chanAServo(
    CHAN_0_SERVO_OUT_PIN,
    scotchYokeMapPercentage, //linearMapPercentage,
    0, 160,
    0.05
);

ServoStepper chanBServo(
    CHAN_1_SERVO_OUT_PIN,
    scotchYokeMapPercentage, //linearMapPercentage,
    0, 180
);

bool lastStepStateA = false;
bool lastStepStateB = false;

void setup() {
    chanAServo.begin();
    chanBServo.begin();
    
    pinMode(CHAN_0_STEP_PIN, INPUT);
    pinMode(CHAN_0_DIR_PIN, INPUT);
    pinMode(CHAN_1_STEP_PIN, INPUT);
    pinMode(CHAN_1_DIR_PIN, INPUT);

    
    pinMode(CHAN_0_ENDSTOP_PIN, OUTPUT);
    pinMode(CHAN_1_ENDSTOP_PIN, OUTPUT);
}


bool DEBUGmoveDir = true;

void loop() {

        
    // if step rising edge
    bool stepPinState = digitalRead(CHAN_0_STEP_PIN);

    // FIXME FOR TESTING ONLY GEORGI
    chanAServo.step(DEBUGmoveDir);
    if (chanAServo.endstopMinus) { DEBUGmoveDir = true; delay(1000); }
    if (chanAServo.endstopPlus) { DEBUGmoveDir = false; delay(1000); }
    delay(400);
    
    if (lastStepStateA == false && stepPinState == true) {
      lastStepStateA = true;
      chanAServo.step(digitalRead(CHAN_0_DIR_PIN));
    }
    if (!stepPinState) lastStepStateA = false;
    
    // same for channel B
    stepPinState = digitalRead(CHAN_1_STEP_PIN);
    if (lastStepStateB == false && stepPinState == true) {
      lastStepStateB = true;
      chanBServo.step(digitalRead(CHAN_1_DIR_PIN));
    }
    if (!stepPinState) lastStepStateB = false;
      

    
    digitalWrite(CHAN_0_ENDSTOP_PIN, chanAServo.endstopMinus);
    digitalWrite(CHAN_1_ENDSTOP_PIN, chanBServo.endstopMinus);

}
