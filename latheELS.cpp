 // StepperMotorJoystick.cpp - Angelo Z. (2025)

#include <Arduino.h>
#include <limits.h>
#include <nI2C.h> // Versione 2.1


// Cambiato CI2C::Speed::FAST in SLOW perchè sto usando un cavo da 3 metri
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
            handle = nI2C->RegisterDevice(0x52, 1, CI2C::Speed::SLOW); 

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



#define MICROSTEPS_PER_REV 400      // DM542
#define SCREWS_PITCH_MM 2.5         // Asse Z
#define DISTANCE_TO_TRAVEL_MM 10.0  // 1cm

#define STEP_PIN 10     // PB2
#define DIR_PIN 11      // PB3
#define ENA_PIN 7       // PD7
#define ENCODER_A_PIN 2 // PD2 -> Step PB2
#define ENCODER_B_PIN 3 // PD3 -> Dir PB3
#define ENDSTOP0_PIN 8  // PB0
#define ENDSTOP1_PIN 9  // PB1

#define MAX_PULSE_DELAY 20900
#define MIN_PULSE_DELAY 100
#define ENCODER_TIMEOUT_MS  3000

#define JOYSTICK_CENTER 127
#define JOYSTICK_DEADZONE 8

#define STEPS_TO_RELEASE_ENDSTOP (MICROSTEPS_PER_REV / SCREWS_PITCH_MM) * DISTANCE_TO_TRAVEL_MM

volatile unsigned long lastEncoderActivity = 0;
unsigned long stepperPulseDelay  = ULONG_MAX;
volatile bool endstopTriggered   = false;
unsigned long stepperMotionStartTime = 0;
bool automatic = false;
bool memAngle = false;
bool stepperIdle = true;
int stickAngle = 0;
volatile bool encoderTurns = false;
volatile bool stepPending = false;
volatile bool pulseHigh = false;
volatile int encoderSteps = 0;
int allowedDirection = 0;

unsigned long preloadCurveTable[128];

enum StepperDirection {
    DIR_NONE = 0,
    DIR_FORWARD = 1,
    DIR_BACKWARD = -1
};

volatile int stepperDir = DIR_NONE;

WiiNunchuk nunchuk;



void initEasedTable() {
    unsigned long smoothCurve(int angle);
    for (int i = 0; i < 128; i++) {
        preloadCurveTable[i] = smoothCurve(i);
    }
}


bool stickTilt() {
    return (abs(stickAngle) >= JOYSTICK_DEADZONE);
}


void runStepper(unsigned long t) {
    static unsigned long currentDelay = 0;
    static unsigned long lastDelay = 3000;
    const float easing = 0.06;

    if (t == ULONG_MAX) t = 3000;

    // Smorza i movimenti bruschi del joystick
    currentDelay = (unsigned long)((1.0 - easing) * lastDelay + easing * t);
    if (currentDelay < MIN_PULSE_DELAY) currentDelay = MIN_PULSE_DELAY;
    lastDelay = currentDelay;

       
    PORTB &= ~(1 << PB2);
    delayMicroseconds(currentDelay);   
    PORTB |= (1 << PB2);

    //Serial.println("....");
}


void enableStepper(bool enable) {
    if (stepperIdle != enable) return;
    stepperIdle = !enable;

    if (enable) PORTD &= ~(1 << PD7);
    else        PORTD |= (1 << PD7);

    delayMicroseconds(100);
}


void setRotation(int angle) {
    if (angle >= JOYSTICK_DEADZONE) {
        PORTB &= ~(1 << PB3);
        stepperDir = DIR_FORWARD;
    } else if (angle <= -JOYSTICK_DEADZONE) {
        PORTB |= (1 << PB3);
        stepperDir = DIR_BACKWARD;
    } else {
        stepperDir = DIR_NONE;
    }
}


void disableStepper() {
    automatic = false;
    stepperPulseDelay = ULONG_MAX;
    stickAngle = 0;
    enableStepper(false);
}


// Hello, World!
// Encoder Pin 2/3 -> [Arduino] Pin 10/11 -> DM542
void encoderISR() {
    if (automatic || stickTilt()) 
        return;

    int dir = ((PIND >> PD2) & 1) == ((PIND >> PD3) & 1) ? DIR_FORWARD : DIR_BACKWARD;

    if (endstopTriggered && dir != allowedDirection)
        return;

    stepperDir = dir;
    PORTB = (PORTB & ~(1 << PB3)) | (dir == DIR_FORWARD ? 0 : (1 << PB3));

    stepPending = true;
    lastEncoderActivity = millis();
    encoderTurns = true;
}


ISR(TIMER1_COMPA_vect) {
    if (stepPending) {
        if (!pulseHigh) {
            PORTB |= (1 << PB2);
            pulseHigh = true;
        } else {
            PORTB &= ~(1 << PB2);
            stepPending = false;
            pulseHigh = false;
        }
    }
}

/////////////////////////////////////////////////////////////////////
// Esempio di curve esponenziale 
// Tipico usato nei Videogiochi 2d-3d, Robotica e cinematografica per movimenti 
// fluidi e smorzati
/////////////////////////////////////////////////////////////////////
unsigned long smoothCurve(int angle) {
    const float kp    = 0.08;
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


unsigned long motionProfile(unsigned long t) { 
    const unsigned long accelTime = 2000;
    const long minDelay = 800;
    const long maxDelay = stepperPulseDelay;

    // A velocita' bassa non serve una accelerazione
    if (maxDelay >= 1250) 
        return maxDelay;

    unsigned long motion = millis() - t;
    if (motion < accelTime) {
        float ratio = float(motion) / accelTime;
        //float eased = pow(ratio, 0.2); // 0.3
        float eased = (1.0 - cos(ratio * PI)) / 2.0; // S-curve
        return minDelay - eased * (minDelay - maxDelay);
    } 

    return maxDelay;
}


void handleEndstops() {
    if (endstopTriggered) {
        // Basta 1 cm per liberare l'asse
        if (encoderSteps >= STEPS_TO_RELEASE_ENDSTOP) {
            endstopTriggered = false;
            encoderSteps = 0;
            allowedDirection = DIR_NONE;
        } 
       
        return;
    } 

    bool endstop0 = !(PINB & (1 << PB0)); 
    bool endstop1 = !(PINB & (1 << PB1)); 

    if (endstop0 || endstop1) {
        disableStepper();
        endstopTriggered = true;
        encoderSteps = 0;

        if (endstop0) allowedDirection = DIR_BACKWARD; 
        if (endstop1) allowedDirection = DIR_FORWARD; 
    }

    // Serial.print(endstop0);
    // Serial.print(" ");
    // Serial.print(endstop1);
    // Serial.print(" ");
    // Serial.println(endstopTriggered);
}


void handleButton(uint8_t buttonId, bool buttonState) {
    static bool zPressed                = false;
    static bool cPressed                = false;
    static int autoDirection            = 0;
    static unsigned long autoDelay      = 0;
    static unsigned long buttonHoldTime = 0;

    if (buttonId == 0) { // Z button
        if (buttonState && !zPressed) {
            buttonHoldTime = millis();

            if (automatic) {
                enableStepper(false);
                stepperMotionStartTime = 0;
                automatic = false;
            }
        }

        if (buttonState && zPressed) {
            unsigned long pressDuration = millis() - buttonHoldTime;

            if (!memAngle && stickTilt()) {
                autoDelay = smoothCurve(abs(stickAngle));
                autoDirection = stickAngle;
                memAngle = true;
                automatic = true; 
            }

            if (!automatic && memAngle && pressDuration >= 1000) {
                setRotation(autoDirection);
                enableStepper(true);
                stepperPulseDelay = autoDelay;
                automatic = true;
                stepperMotionStartTime = millis();
            }
        }
    } 

    if (buttonId == 1) { // C button
        if (buttonState == !cPressed) {
            memAngle = false;
            automatic = false;
            stepperPulseDelay = ULONG_MAX;
        }
    }
 
    zPressed = buttonState;
    cPressed = buttonState;
}


void setup() {
    nunchuk.begin();
   
    //Serial.begin(115200);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENA_PIN, OUTPUT);
    pinMode(ENDSTOP0_PIN, INPUT_PULLUP); 
    pinMode(ENDSTOP1_PIN, INPUT_PULLUP);
    pinMode(ENCODER_A_PIN, INPUT);
    pinMode(ENCODER_B_PIN, INPUT);


    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(ENA_PIN, HIGH);

    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);

    OCR1A = 100;
    TIMSK1 |= (1 << OCIE1A);

    initEasedTable();

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);

    delay(2000);
}


void loop() {

    handleEndstops(); 
    
    nunchuk.update();

    if (!endstopTriggered) {
        if (automatic) {
            unsigned long autoPulseDelay = motionProfile(stepperMotionStartTime);
            runStepper(autoPulseDelay);
        } else {
            stickAngle = nunchuk.analogX() - JOYSTICK_CENTER;

            // Logica joystick
            if (stickTilt()) {
                encoderTurns = false; // Annulla encoder in esecuzione
                setRotation(stickAngle);
                enableStepper(true);
                // Calcolata al volo, lenta 
                //pulseDelay = smoothCurve(stickAngle);
                // Precalcolata, veloce
                stepperPulseDelay = preloadCurveTable[constrain(abs(stickAngle), 0, 127)];
                runStepper(stepperPulseDelay);
            } else {
                // Qui il joystick e' in posizione neutrale, encoder gira? non spegnere lo stepper
                if (!encoderTurns && !stepPending) 
                    enableStepper(false);
                stepperPulseDelay = ULONG_MAX;
            }
        } 

        handleButton(0, nunchuk.zButton());
        handleButton(1, nunchuk.cButton());
    }


    if (lastEncoderActivity) {
        if (!stepPending && 
            !automatic   &&
            !stickTilt() && (millis() - lastEncoderActivity >= ENCODER_TIMEOUT_MS)) {
            enableStepper(false);
            encoderTurns = false;
            lastEncoderActivity = 0;
        }
    }
}
