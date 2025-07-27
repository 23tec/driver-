// Stepper.cpp - Angelo Z. (2025)
// Controller for stepper motors with nunchuk
#include <Arduino.h>
#include <nI2C.h> // Versione 2.1

class WiiNunchuk {
    private:
        CI2C::Handle handle;
        uint8_t data[6];

        void decrypt() {
            for (int i = 0; i < 6; i++) {
                data[i] = (data[i] ^ 0x17) + 0x17;
            }
        }

    public:
        void begin() {
            handle = nI2C->RegisterDevice(0x52, 1, CI2C::Speed::FAST);

            const uint8_t init1[] = { 0xF0, 0x55 };
            const uint8_t init2[] = { 0xFB, 0x00 };

            nI2C->Write(handle, init1, sizeof(init1), 0);
            nI2C->Write(handle, init2, sizeof(init2), 0);
        }

        bool update() {
            const uint8_t start[] = { 0x00 };
            nI2C->Write(handle, start, sizeof(start), 0);

            CI2C::status_t status = nI2C->Read(handle, data, sizeof(data), nullptr);
            if (!status != CI2C::STATUS_OK) return false;

            decrypt();
            return true;
        }

        uint8_t analogX() { return data[0]; }
        uint8_t analogY() { return data[1]; }

        bool zButton() { return !(data[5] & 0x01); }
        bool cButton() { return !(data[5] & 0x02); }

        uint16_t accelX() {
            return (data[2] << 2) | ((data[5] >> 2) & 0x03);
        }

        uint16_t accelY() {
            return (data[3] << 2) | ((data[5] >> 4) & 0x03);
        }

        uint16_t accelZ() {
            return (data[4] << 2) | ((data[5] >> 6) & 0x03);
        }
};


#define STEP_PIN  2
#define DIR_PIN   3
#define ENA_PIN   9

#define ENDSTOP_0 10
#define ENDSTOP_1 11

#define BUTTON_TIMEOUT_MS 1000

const int centerJoy     = 127;
const int deadZone      = 2;

unsigned long velocity = 0;
unsigned long savedAngle = 0;
unsigned long buttonPressTime = 0;
int automatic = 0;

bool buttonPressed = false;
bool endstopTriggered = false;
bool stepperIdle = true;
int stepperDir = 0;
const float alpha = 0.1;
float lowpass = 0;

WiiNunchuk nunchuk;


void setup() {
    nunchuk.begin();
   
    //Serial.begin(115200);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENA_PIN, OUTPUT);
    pinMode(ENDSTOP_0, INPUT_PULLUP);
    pinMode(ENDSTOP_1, INPUT_PULLUP);

    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(ENA_PIN, HIGH);
}


void runStepper(unsigned long us) {
    static unsigned long lastPulseTime = 0;
    static bool pulseActive = false;

    const unsigned long highTime = constrain(us / 50, 2, 10);
    const unsigned long lowTime = us;
    const unsigned long currentTime = micros();

    if (pulseActive && currentTime - lastPulseTime >= highTime) {
        PORTD &= ~(1 << PD2);
        pulseActive = false;
        lastPulseTime = currentTime;
    }
    else if (!pulseActive && currentTime - lastPulseTime >= lowTime) {
        PORTD |= (1 << PD2);
        pulseActive = true;
        lastPulseTime = currentTime;
    }
}


// Abilitazione stepper
void enableStepper(bool enable) {
    if (stepperIdle != enable) return;

    if (enable) 
        PORTB &= ~(1 << PB1);
    else
        PORTB |= (1 << PB1);

    stepperIdle = !enable;

    // Piccolo tempo per lo stepper driver
    delayMicroseconds(100);
}


// Imposta senso di rotazione
void setRotation(int direction) {
    const int threshold = 5;
    const int hysteresis = 10;

    if (abs(direction) < threshold) return;

    if (direction > hysteresis && stepperDir != 1) {
        PORTD &= ~(1 << PD3);
        stepperDir = 1;
    } else if (direction < -hysteresis && stepperDir != -1) {    
        PORTD |= (1 << PD3); 
        stepperDir = -1;
    }
}


// Ripristino
void resetStepper() {
    automatic = 0;
    velocity  = 0;

    endstopTriggered = false;
}


unsigned long smoothSpeed(int input) {
    int delta = abs(input);
    return 100 + 20900 * exp(-0.03 * delta);
}


void handleEndStops() {
    if (!(PINB & (1 << PB2)) || !(PINB & (1 << PB3))) {
        endstopTriggered = true;
        enableStepper(false);
    } 
    else if (endstopTriggered) {
            resetStepper();
    }
}


void handleJoystickInput(int angle) {
    if (automatic && abs(angle) >= deadZone && (millis() - buttonPressTime >= BUTTON_TIMEOUT_MS)) {
        automatic = 0;
        velocity = 0;
        enableStepper(false);
    }
    else if (abs(angle) >= deadZone && !automatic) {
        enableStepper(true);
        setRotation(angle);
        velocity = smoothSpeed(angle);
        savedAngle = angle;
    } 
    else if (automatic) {
        enableStepper(true);   
        velocity = smoothSpeed(savedAngle); 
    } else {
        enableStepper(false);
        velocity = 0;
    }
}


void handleButton(bool buttonState) {
    if (buttonState && !buttonPressed) {
        automatic = !automatic;
        buttonPressTime = millis();
    }

    buttonPressed = buttonState;
}


void loop() {
    nunchuk.update();

    bool buttonState = nunchuk.zButton();
    int rawReading = nunchuk.analogX() - centerJoy;

    lowpass = alpha * rawReading + (1.0 - alpha) * lowpass;
    int joyAngle = (int) lowpass;

    handleEndStops();

    if (!endstopTriggered) { 
        handleJoystickInput(joyAngle);
        handleButton(buttonState);

        if (!stepperIdle) {
            runStepper(velocity);
        }
    }
}
