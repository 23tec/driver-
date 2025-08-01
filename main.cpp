// StepperMotorJoystick.cpp - Angelo Z. (2025)

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

#define MAX_PULSE_DELAY 20900
#define MIN_PULSE_DELAY 100

const int centerJoy       = 127;
const int deadZone        = 8;

unsigned long pulseDelay  = ULONG_MAX;

unsigned long autoTime    = 0;
bool automatic            = false;
bool memAuto              = false;
bool endstopTriggered     = false;
bool stepperIdle          = true;

int stepperDir            = 0;
int stickAngle            = 0;


WiiNunchuk nunchuk;


unsigned long preloadCurveTable[128];

void initCurveTable() {
    unsigned long smoothCurve(int angle);
    for (int i = 0; i < 128; i++) {
        preloadCurveTable[i] = smoothCurve(i);
    }
}

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

    initCurveTable();
}


// Controllo inclinazione effettiva del joystick
bool stickTilt() {
    return (abs(stickAngle) >= deadZone);
}


// Segnali per lo stepper
void runStepper(unsigned long t) {
    static unsigned long currentDelay = 0;
    static unsigned long lastDelay = 3000; // t
    const float easing = 0.06;

    // Smorza i movimenti bruschi del joystick
    currentDelay = (unsigned long)((1.0 - easing) * lastDelay + easing * t);
    if (currentDelay < MIN_PULSE_DELAY) currentDelay = MIN_PULSE_DELAY;
    lastDelay = currentDelay;

       
    PORTD &= ~(1 << PD2);
    delayMicroseconds(10);   
    
    PORTD |= (1 << PD2);
    delayMicroseconds(currentDelay); 
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
    const int hysteresis = 10;

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


/////////////////////////////////////////////////////////////////////
// Esempi di curve esponenziale 
// Tipico usato nei Videogiochi 2d-3d, Robotica e cinematografica per movimenti 
// fluidi e smorzati
/////////////////////////////////////////////////////////////////////
/*
unsigned long smoothCurve(int angle) {
    int delta = abs(angle);
    if (delta == 0) return ULONG_MAX;
    return 100 + 20900 * exp(-0.03 * delta);
}
*/

/*
unsigned long smoothCurve(int angle) {
    int delta = abs(angle);
    if (delta == 0) return ULONG_MAX;
    
    const int base = 100;
    const int maxOut = 20900;
    const float maxDelta = sqrt(127.0);

    float speedRatio = sqrt(delta) / maxDelta;

    return base + int((1.0 - speedRatio) * (maxOut - base));
}
*/

/*
unsigned long smoothCurve(int angle) {
    const float base    = 100;
    const long maxOut   = 20900;
    const int maxAngle  = 127;

    int delta           = constrain(abs(angle), 0, maxAngle);
    float x             = float(delta) / maxAngle;

    // EaseInOutQuad: accelera al centro, lento ai bordi
    float eased = (x < 0.5) ? (2 * x * x) : (1 - pow(-2 * x + 2, 2) / 2);

    return base + int((1.0 - eased) * (maxOut - base));
}
*/

/* XX
unsigned long smoothCurve(int angle) {
    const int base      = 100;
    const long maxOut   = 20900;
    const int maxAngle  = 127;

    int delta           = constrain(abs(angle), 0, maxAngle);
    float x             = float(delta) / maxAngle;
    float eased         = 1 - pow(1 - x, 3); // easeOutCubic
    
    return base + int((1.0 - eased) * (maxOut - base));
}
*/

unsigned long smoothCurve(int angle) {
    const float kp    = 0.03;
    const int base    = MIN_PULSE_DELAY;
    const long maxOut = MAX_PULSE_DELAY;
    const int delta   = abs(angle);

    if (delta == 0)
        return ULONG_MAX;

    float maxExp         = exp(-kp * 0);
    float minExp         = exp(-kp * 127);
    float currentExp     = exp(-kp * delta);
    float normalized     = (currentExp - minExp) / (maxExp - minExp);

    return base + int(normalized * (maxOut - base));
}
/////////////////////////////////////////////////////////////////////
unsigned long motionProfile(unsigned long t) { 
    const unsigned long accelTime = 4000;
    const long minDelay = 2000;
    const long maxDelay = pulseDelay;

    // A velocita' bassa non serve una accelerazione
    if (maxDelay >= 1250) 
        return maxDelay;

    if (t < accelTime) {
        float ratio = float(t) / accelTime;
        return minDelay - ratio * (minDelay - maxDelay);
    } 

    return maxDelay;
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


// Gestione pulsanti Z/C
void handleButton(int buttonId, bool buttonState) {
    static bool zPressed                = false;
    static bool cPressed                = false;
    static int autoDirection            = 0;
    static unsigned long autoDelay      = 0;

    if (buttonId == 0) {
        if (buttonState && !zPressed) {
            automatic = !automatic;
            if (!stickTilt() && !memAuto) {
                automatic = false;
            }
            else if (automatic && !memAuto) {
                memAuto = true;
                autoDelay = smoothCurve(stickAngle); // pulseDelay;
                autoDirection = stickAngle;
            }
            else if (automatic && memAuto) {
                enableStepper(true);
                setRotation(autoDirection);
                pulseDelay = autoDelay;
            }

            if (automatic) autoTime = millis();
        }

        zPressed = buttonState;
    } 
    else if (buttonId == 1) {
        if (buttonState && !cPressed) { 
            memAuto = false;
            automatic = false;
            pulseDelay = ULONG_MAX;
        }

        cPressed = buttonState;
    }
}


// Filtro passa-basso
int lowpassFilter(int rawData) {
    const float alpha = 0.08;
    static float filtered = 0.0;

    filtered = alpha * rawData + (1.0 - alpha) * filtered;
    return (int) filtered;
}


void loop() {

    handleEndStops(); 
    
    nunchuk.update();

    if (!endstopTriggered) {
        if (automatic) {
            // Parte con una accelerazione
            unsigned long autoPulse = motionProfile(millis() - autoTime);
            runStepper(autoPulse);
        } else {
            stickAngle = nunchuk.analogX() - centerJoy;
        
            // Logica joystick in manuale
            if (stickTilt()) {
                enableStepper(true);
                setRotation(stickAngle);
                // Calcolata al volo, lenta 
                //pulseDelay = smoothCurve(stickAngle);
                // Precalcolata, veloce
                pulseDelay = preloadCurveTable[constrain(abs(stickAngle), 0, 127)];
                runStepper(pulseDelay);
            } else {
                enableStepper(false);
            }
        } 

        handleButton(0, nunchuk.zButton());
        handleButton(1, nunchuk.cButton());
    }
}
