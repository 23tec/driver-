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


#define ENCODER_PPR 1000
#define MICROSTEPS_PER_REV 400      // DM542
#define LEADSCREW_PITCH_MM 2.5      // Asse Z
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

#define STEPS_TO_RELEASE_ENDSTOP (MICROSTEPS_PER_REV / LEADSCREW_PITCH_MM) * DISTANCE_TO_TRAVEL_MM

volatile unsigned long lastEncoderActivity = 0;
unsigned long stepperPulseDelay  = ULONG_MAX;
volatile bool endstopTriggered   = false;
unsigned long stepperMotionStartTime = 0;
bool autofeedActived = false;
bool stepperIdle = true;
int stickAngle = 0;
volatile bool stepPending = false;
volatile bool pulseHigh = false;
volatile int encoderSteps = 0;
volatile int buttonsQueryDelay = 0;

int allowedDirection = 0;

unsigned long preloadCurveTable[128];

struct Button {
    bool isPressed; // Appena premuto
    bool isHold;    // Tenere premuto
    bool isClicked; // Click breve
    bool isHeld;    // Pressione lunga
    bool mask;      // Stato 
    unsigned long pressTime;  // Tempo pressione
};

Button zButton, cButton;


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


void runStepper(unsigned long pulseWidth) {
    static unsigned long currentDelay = 0;
    static unsigned long lastDelay = 3000;
    const float easing = 0.06;

    if (pulseWidth == ULONG_MAX) pulseWidth = 3000;

    // Smorza i movimenti bruschi del joystick
    currentDelay = (unsigned long)((1.0 - easing) * lastDelay + easing * pulseWidth);
    if (currentDelay < MIN_PULSE_DELAY) currentDelay = MIN_PULSE_DELAY;
    lastDelay = currentDelay;

       
    PORTB &= ~(1 << PB2);
    delayMicroseconds(currentDelay);   
    PORTB |= (1 << PB2);

    //Serial.println(".");
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
        //stepperDir = DIR_NONE;
    }
}


void shutdown() {
    enableStepper(false);
    autofeedActived = false;
    stepperPulseDelay = ULONG_MAX;
    stepperMotionStartTime = 0;
    stepPending = false;
    stickAngle = 0;
}


// Hello, World!
// Encoder Pin 2/3 -> [Arduino] Pin 10/11 -> DM542
void encoderISR() {
    if (autofeedActived || stickTilt()) 
        return;

    int dir = ((PIND >> PD2) & 1) == ((PIND >> PD3) & 1) ? DIR_FORWARD : DIR_BACKWARD;

    if (endstopTriggered && dir != allowedDirection) return;
    if (endstopTriggered) encoderSteps++;

    stepperDir = dir;
    PORTB = (PORTB & ~(1 << PB3)) | (dir == DIR_FORWARD ? 0 : (1 << PB3));

    enableStepper(true);
    lastEncoderActivity = millis();
    stepPending = true;
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


unsigned long motionProfile(unsigned long startTime) { 
    const unsigned long accelTime = 2000;
    const long minDelay = 800;
    const long maxDelay = stepperPulseDelay;

    // A bassa velocita' non serve una accelerazione
    if (maxDelay >= 1300) return maxDelay;

    unsigned long timelapse = millis() - startTime;
    if (timelapse < accelTime) {
        float ratio = float(timelapse) / accelTime;
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
        shutdown();
        endstopTriggered = true;
        encoderSteps = 0;

        allowedDirection = endstop0 ? DIR_BACKWARD : DIR_FORWARD; 
    }

    // Serial.print(endstop0);
    // Serial.print(" ");
    // Serial.print(endstop1);
    // Serial.print(" ");
    // Serial.println(endstopTriggered);
}


void buttonEvent(Button &btn, bool state) {
    btn.isPressed  =  state && !btn.mask;
    btn.mask       =  state;

    if (btn.isPressed) {
        btn.pressTime = millis();
        btn.isClicked = false;
        btn.isHeld = false;
        btn.isHold = false;
    }

    unsigned long duration = millis() - btn.pressTime;

    if (state) {
        btn.isHeld = (duration >= 1000);
        btn.isHold = !btn.isHeld && (duration >= 500);
    } else {
        btn.isClicked = (duration < 200);
        btn.isHold = false;
        btn.isHeld = false;
        btn.pressTime = 0;
    }
}


void configure_buttons() {

    zButton = {
        .isPressed = false,
        .isHold = false,
        .isClicked = false,
        .isHeld = false,
        .mask = nunchuk.zButton(),
        .pressTime = 0
    };

    cButton = {
        .isPressed = false,
        .isHold = false,
        .isClicked = false,
        .isHeld = false,
        .mask = nunchuk.zButton(),
        .pressTime = 0
    };
}


void handleButtons() {
    static int           direction  = 0;
    static unsigned long feedrate   = 0;
    static bool          saved      = false;
    static bool          inProgress = false;

    // Save autofeed
    if (zButton.isPressed && !saved) {
        if (!stickTilt())
            return;
        feedrate = stepperPulseDelay;
        direction = stickAngle;
        autofeedActived = true;
        inProgress = true;
        saved = true;
    } 
    // Stop autofeed
    else if (zButton.isPressed && autofeedActived && inProgress) {
        enableStepper(false);
        autofeedActived = false;
        inProgress = false;
    }

    // Start autofeed
    if (saved && !inProgress && !autofeedActived) {
        if (zButton.isHeld) {
            setRotation(direction);
            enableStepper(true);
            stepperPulseDelay = feedrate;
            stepperMotionStartTime = millis();
            inProgress = true;
            autofeedActived = true;
        }
    }

    // Clear autofeed
    if (cButton.isPressed) {
        enableStepper(false);
        stepperPulseDelay = ULONG_MAX;
        autofeedActived = false;
        inProgress = false;
        saved = false;
    }
}


ISR(TIMER2_COMPA_vect) {
    buttonsQueryDelay++;

    if (buttonsQueryDelay >= 20) { // 20ms
        buttonEvent(zButton, nunchuk.zButton());
        buttonEvent(cButton, nunchuk.cButton());
        handleButtons();
        buttonsQueryDelay = 0;
    }
}


void stopEncoderActivity() {
    lastEncoderActivity = 0;
    stepPending = false;
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

    // Timer1: Stepper motor e encoder
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    OCR1A = 100;
    TIMSK1 |= (1 << OCIE1A);

    // Timer2: Tasti joystick ~1ms
    configure_buttons();
    TCCR2A = 0;
    TCCR2B = 0;
    TCCR2A |= (1 << WGM21);
    TCCR2B |= (1 << CS22);
    OCR2A = 249;
    TIMSK2 |= (1 << OCIE2A);

    sei();

    initEasedTable();

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);

    delay(2000);
}


void loop() {

    handleEndstops(); 
    
    if (endstopTriggered) return;
    
    nunchuk.update();


    if (!autofeedActived) {        
        // Logica joystick 
        stickAngle = nunchuk.analogX() - JOYSTICK_CENTER;
        
        if (stickTilt()) {
            stopEncoderActivity(); // Annulla encoder
            setRotation(stickAngle);
            enableStepper(true);
            // Calcolata al volo, lenta 
            //pulseDelay = smoothCurve(stickAngle);
            // Precalcolata, veloce
            stepperPulseDelay = preloadCurveTable[constrain(abs(stickAngle), 0, 127)];
            runStepper(stepperPulseDelay);
        } else {
            // In neutro
            if (lastEncoderActivity == 0) {
                enableStepper(false);
                stepperPulseDelay = ULONG_MAX;
            }
        }
    } else {
        // Autofeed enable
        runStepper(motionProfile(stepperMotionStartTime));
    }


    
    if (lastEncoderActivity) {
        if (!stepPending      && 
            !autofeedActived  &&
            !stickTilt()      && (millis() - lastEncoderActivity >= ENCODER_TIMEOUT_MS)) {
            enableStepper(false);
            stopEncoderActivity();
        }
    }
}
