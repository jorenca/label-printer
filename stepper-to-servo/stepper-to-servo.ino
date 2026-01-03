#include <Arduino.h>
#include <Servo.h>


class ServoStepper {
public:
    // Public endstop indicators
    bool endstopPlus  = false;
    bool endstopMinus = false;

    // Constructor
    ServoStepper(
        uint8_t servoPin,
        int servoMinDeg = 0,
        int servoMaxDeg = 180
    )
        : _pin(servoPin),
          _servoMin(servoMinDeg),
          _servoMax(servoMaxDeg)
    {
        _currentAnglePerc = 0;
    }

    void begin() {
        _servo.attach(_pin);
        step(false, 0);
    }

    // Step like a stepper motor
    void step(bool dir, float deltaPercent = 0.005) {

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
    float _currentAnglePerc;

    void updateServo() {
      
        _currentAnglePerc = _currentAnglePerc < 0.0 ? 0.0 : _currentAnglePerc;
        _currentAnglePerc = _currentAnglePerc > 1.0 ? 1.0 : _currentAnglePerc;
        
        endstopPlus = _currentAnglePerc > 0.999;
        endstopMinus = _currentAnglePerc < 0.001;

        int currentAngleDeg = map(
            _currentAnglePerc * 10000,
            0 * 10000,
            1 * 10000,
            _servoMin,
            _servoMax
        );
        
        _servo.write(currentAngleDeg);
    }
};

//////////////////////////////////////////////

#define MOTORS_ENABLE_PIN 2

#define CHAN_0_STEP_PIN 3
#define CHAN_0_DIR_PIN 4
#define CHAN_0_ENDSTOP_PIN 5

#define CHAN_0_SERVO_OUT_PIN 9

#define CHAN_1_STEP_PIN 6
#define CHAN_1_DIR_PIN 7
#define CHAN_1_ENDSTOP_PIN 8

#define CHAN_1_SERVO_OUT_PIN 10


ServoStepper chanAServo(CHAN_0_SERVO_OUT_PIN, 0, 180);
ServoStepper chanBServo(CHAN_1_SERVO_OUT_PIN, 0, 180);
bool lastStepStateA = false;
bool lastStepStateB = false;

void setup() {
    chanAServo.begin();
    chanBServo.begin();

    pinMode(MOTORS_ENABLE_PIN, INPUT_PULLUP);
    
    pinMode(CHAN_0_STEP_PIN, INPUT);
    pinMode(CHAN_0_DIR_PIN, INPUT);
    pinMode(CHAN_1_STEP_PIN, INPUT);
    pinMode(CHAN_1_DIR_PIN, INPUT);

    
    pinMode(CHAN_0_ENDSTOP_PIN, OUTPUT);
    pinMode(CHAN_1_ENDSTOP_PIN, OUTPUT);
}


void loop() {

    if (digitalRead(MOTORS_ENABLE_PIN)) {
        
        // if step rising edge
        bool stepPinState = digitalRead(CHAN_0_STEP_PIN);
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
      
    }

    
    digitalWrite(CHAN_0_ENDSTOP_PIN, chanAServo.endstopMinus);
    digitalWrite(CHAN_1_ENDSTOP_PIN, chanBServo.endstopMinus);

}
