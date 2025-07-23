// Stepper.cpp - Angelo Z. (2025)
// Controller for stepper motors with use of nunchuk.
// All the code implements non-blocking functions.
#include <nI2C.h>

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

      const uint8_t init1[] = {0xF0, 0x55};
      const uint8_t init2[] = {0xFB, 0x00};

      nI2C->Write(handle, init1, sizeof(init1), 0);
      nI2C->Write(handle, init2, sizeof(init2), 0);
    }

    bool update() {
      const uint8_t start[] = {0x00};
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

const int centerJoy   = 127;
const int deadZone    = 8;
const int maxSpeed    = 5000;
const int minSpeed    = 100;

int velocity  = 0;
int automatic = 0;

bool buttonPressed = false;
bool endstopTriggered = false;

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


// Attiva stepper
void enableStepper(bool enable) {
    if (enable) 
      PORTB |= (1 << PB1);
    else
      PORTB &= ~(1 << PB1);
}


// Imposta senso di rotazione
void setRotation(int direction) {
    if (direction > 0) 
        PORTD |= (1 << PD3);
    else
        PORTD &= ~(1 << PD3);
}

// Ripristino 
void resetStepper() {
    automatic = 0;
    velocità  = 0;
        
    endstopTriggered = false;
}


void loop() {
    const int joyAngle = nunchuk.analogX() - centerJoy;
  
    //automatic ^= nunchuk.zButton;
    bool buttonState = nunchuk.zButton();

    // Finecorsa innescato blocca tutto
    if (!(PIN & (1 << PB2)) || !(PIN & (1 << PB3))) {
        endstopTriggered = true;
    } else {
        if (endstopTriggered)
            resetStepper();
    }
    
    if (!endstopTriggered) {
        // Con leva verso nord e tasto Z premuto attiva il 
        // automatico
        if (buttonState && !buttonPressed
                && nunchuk.analogY() >= 250) {
            automatic = !automatic;
        }
  
        // Con leva X in azione esci da automatico
        if (automatic && abs(joyAngle) > deadZone)
            automatic = 0;
    
        // Con uso joystick
        if (abs(joyAngle) > deadZone) {
            // Attiva stepper
            enableStepper(true);
            // <- ->
            setRotation(joyAngle);
            // Angolazione in velocità 
            velocity = map(abs(joyAngle), deadZone, centerJoy, maxSpeed, minSpeed);
        } 
        else if (automatic) {
            enableStepper(true);
            velocity = minSpeed;
        } 
        else {
            // Disattiva stepper per movimento libero
            enableStepper(false);
            velocity = 0;
        }

      
        runStepper(velocity);
  
        buttonPressed = buttonState;
  
        nunchuk.update();
    }
}
