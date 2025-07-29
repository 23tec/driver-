// Stepper.cpp - Angelo Z. (2025)
// Controller for stepper motor with nunchuck

/*
**Functionality:**
- **Joystick Control**:
  - Tilt left or right to move the motor in the corresponding direction.
  - Greater tilt increases the motor speed proportionally.

- **Z Button (Automatic Mode)**:
  - While tilting press Z to save the current direction and speed.
  - Press Z again to exit automatic mode, retaining the saved speed.
  
- **C Button (Reset)**:
  - Press C to clear saved data for a new recording.
*/
#include <Arduino.h>
#include <limits.h>
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

#define ENDSTOP0_PIN 10
#define ENDSTOP1_PIN 11

const int centerJoy     = 127;
const int deadZone      = 4;

unsigned long velocity  = ULONG_MAX;

bool automatic          = false;
bool memAuto            = false;
bool endstopTriggered   = false;
bool stepperIdle        = true;
int stepperDir          = 0;
int joyangle            = 0;


WiiNunchuk nunchuk;


void setup() {
    nunchuk.begin();
   
    Serial.begin(115200);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENA_PIN, OUTPUT);
    pinMode(ENDSTOP0_PIN, INPUT_PULLUP);
    pinMode(ENDSTOP1_PIN, INPUT_PULLUP);

    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(ENA_PIN, HIGH);
}


// Pulsi allo stepper
void runStepper(unsigned long us) {
    static unsigned long lastPulseTime = 0;
    static bool pulseActive = false;

    const unsigned long highTime = constrain(us / 50, 2, 10);
    const unsigned long lowTime = us;
    const unsigned long currentTime = micros();

    if (us == ULONG_MAX) 
        return;

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
    stepperIdle = !enable;

    if (enable) PORTB &= ~(1 << PB1);
    else        PORTB |= (1 << PB1);

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


// Ripristino finecorsa e stepper
void resetStepper() {
    automatic = false;
    velocity  = ULONG_MAX;
    endstopTriggered = false;
}


// Angolazione della leva del joystick in una curva esponenziale
unsigned long smoothSpeed(int angolo) {
    int delta = abs(angolo);
    if (delta == 0) return ULONG_MAX;
    return 100 + 20900 * exp(-0.03 * delta);
}


// Controllo stato dei finecorsa
void handleEndStops() {
    if (!(PINB & (1 << PB2)) || !(PINB & (1 << PB3))) {
        endstopTriggered = true;
        enableStepper(false);
    } 
    else if (endstopTriggered) {
            resetStepper();
    }
}


// Gestione leva del joystick
void handleJoystickInput(int angle) {
    if (automatic) return;

    if (abs(angle) >= deadZone) {
        enableStepper(true);
        setRotation(angle);
        velocity = smoothSpeed(angle);
    } else {
        velocity = ULONG_MAX;
        enableStepper(false);
    }
}


// Gestione tasti del joystick
void handleButton(uint8_t buttonId, bool buttonState) {
    static bool zPressed    = false;
    static bool cPressed    = false;
    static int autoDirection    = 0;
    static unsigned long autoVelocity = 0;

    if (buttonId == 0) {
        if (buttonState && !zPressed) {
            automatic = !automatic;

            // Nessun automatico con joystick neutrale senza memoria
            if (velocity == ULONG_MAX && !memAuto) {
                automatic = false;
                enableStepper(false);
            } 
            else if (automatic && !memAuto) { // Salva
                memAuto = true;
                autoVelocity = velocity;
                autoDirection = joyangle;
            }
            else if (automatic && memAuto) { // Attiva
                enableStepper(true);
                setRotation(autoDirection);
                velocity = autoVelocity;
            }
        }

        zPressed = buttonState;
    } 
    else if (buttonId == 1) {
        if (buttonState && !cPressed) { 
            memAuto = false;
            automatic = false;
            autoVelocity = 0;
        }

        cPressed = buttonState;
    }
}


// Filtro passa-basso
int lowpass(int rawData) {
    const float alpha = 0.1;
    static float filtered = 0.0;

    filtered = alpha * rawData + (1.0 - alpha) * filtered;
    return (int) filtered;
}


void loop() {

    handleEndStops();

    nunchuk.update();

    if (!endstopTriggered) {
        joyangle = lowpass(nunchuk.analogX() - centerJoy);

        handleJoystickInput(joyangle);
        handleButton(0, nunchuk.zButton());
        handleButton(1, nunchuk.cButton());

        if (!stepperIdle) {
            runStepper(velocity);
        }
    }
}
