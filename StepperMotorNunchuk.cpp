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

const int centerJoy     = 127;
const int deadZone      = 8;
const int maxSpeed      = 15000;
const int minSpeed      = 200;

int velocity = 0;
int automatic = 0;

bool buttonPressed = false;
bool endstopTriggered = false;
bool stepperIdle = true;
int stepperDir = 0;

WiiNunchuk nunchuk;


void setup() {
    nunchuk.begin();

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

    const unsigned long highTime = 2;
    const unsigned long lowTime = us;

    if (pulseActive && micros() - lastPulseTime >= highTime) {
        PORTD &= ~(1 << PD2);
        pulseActive = false;
        lastPulseTime = micros();
    }
    else if (!pulseActive && micros() - lastPulseTime >= lowTime) {
        PORTD |= (1 << PD2);
        pulseActive = true;
        lastPulseTime = micros();
    }
}


// Abilitazione stepper
void enableStepper(bool enable) {
    if (!stepperIdle == enable)
        return;
        
    if (enable) 
        PORTB &= ~(1 << PB1);
    else
        PORTB |= (1 << PB1);

    stepperIdle = !enable;

    // Piccolo tempo per il driver stepper
    delayMicroseconds(500);
}


// Imposta senso di rotazione
void setRotation(int direction) {
    if (stepperDir == direction) 
        return;
        
    if (direction > 0)
        PORTD &= ~(1 << PD3);
    else    
        PORTD |= (1 << PD3);
        
    stepperDir = direction;     
}


// Ripristino
void resetStepper() {
    automatic = 0;
    velocity  = 0;

    endstopTriggered = false;
}


void loop() {
    const int joyAngle = nunchuk.analogX() - centerJoy;

    //automatic ^= nunchuk.zButton();
    bool buttonState = nunchuk.zButton();

    // Finecorsa innescato blocca stepper
    if (!(PINB & (1 << PB2)) || !(PINB & (1 << PB3))) {
        endstopTriggered = true;
        enableStepper(false);
    } 
    else if (endstopTried) {
            resetStepper();
    }
    else {
        // Con leva premuto verso nord e tasto Z premuto attiva il 
        // automatico
        if (buttonState && !buttonPressed
                && nunchuk.analogY() > 250) {
            automatic = !automatic;
        }

        // Esci da automatico
        if (automatic && abs(joyAngle) > deadZone) {
            automatic = 0;
            velocity  = 0;
        }

        // Con joystick
        if (abs(joyAngle) > deadZone) {
            // Attiva stepper
            enableStepper(true);
            // <- ->
            setRotation(joyAngle);
            // Mappa angolazione in velocita'
            velocity = map(abs(joyAngle), deadZone, centerJoy, maxSpeed, minSpeed);
        }
        else if (automatic) {
            enableStepper(true);
            velocity = minSpeed;
        }
        else {
            // Dissativa stepper per movimento libero
            enableStepper(false);
            velocity = 0;
        }


        if (!stepperIdle) 
            runStepper(velocity);

        buttonPressed = buttonState;

        nunchuk.update();
    }
}
