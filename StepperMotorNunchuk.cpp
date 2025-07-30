// Stepper.cpp - Angelo Z. (2025)
// Controller for stepper motors with nunchuk 

// Functionality:
// The joystick controls the stepper motor:
//   - Tilt it left or right to move the motor in that direction.
//   - The greater the tilt, the higher the speed.
//
// Pressing the Z button activates automatic mode:
//   - Press Z while the joystick is tilted to save direction and speed.
//   - Press Z again to exit automatic mode, keeping the saved speed.
//
// Pressing the C button clears the saved data:
//   - Use it to disable automatic mode and prepare for a new recording.
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

const int centerJoy       = 127;
const int deadZone        = 4;

unsigned long pulseDelay  = ULONG_MAX;

bool automatic            = false;
bool memAuto              = false;
bool endstopTriggered     = false;
bool stepperIdle          = true;

int stepperDir            = 0;
int stickAngle            = 0;


WiiNunchuk nunchuk;


void setup() {
    nunchuk.begin();
   
    //Serial.begin(115200);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENA_PIN, OUTPUT);
    pinMode(ENDSTOP0_PIN, INPUT_PULLUP);
    pinMode(ENDSTOP1_PIN, INPUT_PULLUP);

    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(ENA_PIN, HIGH);
}


// Controllo inclinazione effettiva del joystick
bool stickTilt() {
    return ((pulseDelay != ULONG_MAX) || (stickAngle != 0));
}


// Segnali per lo stepper
void runStepper(unsigned long usDelay) {
    delayMicroseconds(10);      PORTD &= ~(1 << PD2);
    delayMicroseconds(usDelay); PORTD |= (1 << PD2);
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
void setRotation(int angle) {
    const int threshold = 5;
    const int hysteresis = 5; // 10

    if (abs(angle) < threshold) return;

    if (angle > hysteresis && stepperDir != 1) {
        PORTD &= ~(1 << PD3);
        stepperDir = 1;
    } else if (angle < -hysteresis && stepperDir != -1) {    
        PORTD |= (1 << PD3); 
        stepperDir = -1;
    }
}


// Ripristino finecorsa e stepper
void resetStepper() {
    automatic = false;
    pulseDelay = ULONG_MAX;
    endstopTriggered = false;
}


// Curva esponenziale per ritardo
// unsigned long smoothCurve(int angle) {
//     int delta = abs(angle);
//     if (delta == 0) return ULONG_MAX;
//     return 100 + 20900 * exp(-0.03 * delta);
// }


// unsigned long smoothCurve(int angle) {
//     int delta = abs(angle);
//     if (delta == 0) return ULONG_MAX;
    
//     const int base = 100;
//     const int maxOut = 20900;
//     const float maxDelta = sqrt(127.0);

//     float speedRatio = sqrt(delta) / maxDelta;

//     return base + int((1.0 - speedRatio) * (maxOut - base));
// }


unsigned long smoothCurve(int angle) {
    const float kp              = 0.03;
    const int base              = 100;
    const long maxOut           = 20900;

 
    float maxExp                = exp(-kp * 0);
    float minExp                = exp(-kp * 127);
    float currentExp            = exp(-kp * abs(angle));
    float normalized            = (currentExp - minExp) / (maxExp - minExp);

    return base + int(normalized * (maxOut - base));
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


// Logica joystick
void handleJoystickInput(int angle) {
    //if (automatic) 
        //return;

    if (abs(angle) >= deadZone) {
        enableStepper(true);
        setRotation(angle);
        pulseDelay = smoothCurve(angle);
    } else {
        pulseDelay = ULONG_MAX;
        enableStepper(false);
    }
}


// Gestione pulsanti Z/C
void handleButton(uint8_t buttonId, bool buttonState) {
    static bool zPressed                = false;
    static bool cPressed                = false;
    static int autoDirection            = 0;
    static unsigned long autoDelay      = 0;

    if (buttonId == 0) {
        if (buttonState && !zPressed) {
            automatic = !automatic;

            // Con joystick neutrale senza registro nessun automatico
            // attivo. 
            if (!stickTilt() && !memAuto) {
                automatic = false;
            } 
            else if (automatic && !memAuto) { // Registra
                memAuto = true;
                autoDelay = pulseDelay;
                autoDirection = stickAngle;
            }
            else if (automatic && memAuto) { // Attiva lo stepper
                enableStepper(true);
                setRotation(autoDirection);
                pulseDelay = autoDelay;
            }
        }

        zPressed = buttonState;
    } 
    else if (buttonId == 1) {
        if (buttonState && !cPressed) { 
            memAuto = false;
            automatic = false;
            pulseDelay = ULONG_MAX;
            autoDelay = 0;
        }

        cPressed = buttonState;
    }
}


// Filtro passa-basso
int lowpass(int rawData) {
    const float alpha = 0.08;
    static float filtered = 0.0;

    filtered = alpha * rawData + (1.0 - alpha) * filtered;
    return (int) filtered;
}


void loop() {

    handleEndStops(); 
    
    nunchuk.update();

    if (!endstopTriggered) {
        // Leggere il nunchuk in passo automatico crea microvariazioni nello stepper
        if (!automatic) {
            stickAngle = lowpass(nunchuk.analogX() - centerJoy);
            handleJoystickInput(stickAngle);
        }

        if (!stepperIdle && stickTilt()) 
            runStepper(pulseDelay);

        handleButton(0, nunchuk.zButton());
        handleButton(1, nunchuk.cButton());
    }
}
