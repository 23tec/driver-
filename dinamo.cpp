// PWM Driver Motore con tachimetrica a dinamo e pid
// Configurabile via seriale. 

// Motor.c (c) Angelo Z. 2025
#include <OneButton.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include <SparkFun_External_EEPROM.h>
#include <Wire.h>

#define ARRAY_len(x)    (sizeof(x) / sizeof(x[0]))

#define READ_config       1
#define WRITE_config      0

#define TACHTIMEOUT     100 
#define DYNAMO_pin       A1
#define POT_pin          A0 
#define OPTO_pin          2
#define TRIAC_pin         8
#define POT_RANGE       255
#define SERIAL_interval  40
#define MAXDATA_len      30
#define INVERTED_SCALE(x) (POT_RANGE - (x)) // AC mode

typedef enum { Idle, Running, Acceleration, Deceleration, 
               ConstantSpeed } Motor;

OneButton buttonStart(8);
OneButton buttonStop(9);

// Buffer seriale
char receive_buf[MAXDATA_len] = { 0 };
unsigned long t1 = 0;

float    rampAccel          = 0.1;
float    rampDecel          = 0.5;

int      maxSpeed           = 0;
int      motorSpinThreshold = 0;

volatile boolean zero       = false;
volatile int speedlevel     = 0, 
                      i     = 0;

boolean  motorEnabled       = false,
         motorAccel         = false,
         alarm              = false;
Motor    motorFlag          = Idle;

double   kp = 1.0,
         ki = 0.0,
         kd = 0.0,
       
         input = 0.0, output = 0.0, setPoint = 0.0;

int      showPid = 0;

PID pid(&input, &output, &setPoint, kp, ki, kd, P_ON_M, DIRECT);

ExternalEEPROM eeprom;

// Messaggistica 
struct SerialMsg {
  const char *text;
  bool sent;
} messages[] = {
  { "Motore gira", false }, { "Motore fermo", false },
  { "In accelerazione...", false }, { "In decelerazione...", false },
  { "In velocità...", false } 
};

// Chiudere la seriale prima di riavviare l'avr
// altrimenti abbiamo dati spazzatura 
void resetAVR() {
  if (Serial) Serial.end(); delay(100); 
  asm volatile ("jmp 0");
}


// Invia messaggi solo una volta
void sendMessage(const int x) {
  if (Serial) {
      if (!messages[x].sent) {
        Serial.println(messages[x].text);
        messages[x].sent = true;
      }
  }
}


// Resetta messaggi inviati
void resetSent() {
  for (int i = 0; i < ARRAY_len(messages); i++) {
    messages[i].sent = false;
  }
}


// Svuota buffer seriale
void clearData() {
    memset(receive_buf, 0, MAXDATA_len);
}


// Ricevi dati dalla seriale
void serial_recv_r() {
    int ii = 0;

    while ( Serial.available() > 0 ) {
        if ( ii >= MAXDATA_len ) 
            break;
        receive_buf[ ii++ ] = Serial.read();
    }

    if ( ii > 0 ) { ATCommand( receive_buf ); }
}

// Macro
#define ee(x, ...) \
    do { \
        if (x) \
            eeprom.put(__VA_ARGS__); \
        else \
            eeprom.get(__VA_ARGS__); \
    } while (0)

// Scrive e legge i dati sulla EEPROM
void storage(int rw) {
    if (!eeprom.isConnected(0x50)) {
        Serial.println("EEPROM: error!");
        return;
    }

    ee(rw, 0, kp);
    delay(7);
    ee(rw, 4, ki);
    delay(7);
    ee(rw, 8, kd);
    delay(7);
    ee(rw, 12, RAMP_accel);
    delay(7);
    ee(rw, 16, RAMP_decel);
    delay(7);
    ee(rw, 18, maxSpeed);
    delay(7);
    ee(rw, 20, motorSpinThreshold);

    beep();
}

char *removeLn(char *str) {
  size_t n = strcspn(str, "\n");
  if (n) *(str + n) = '\0';
  return str;
}

boolean isNum(const char *str) {
  while ( *str ) {
    if ( !isdigit( *str ) ) return false;
    str++;
  }

  return true;
}


// Parser per commandi seriali
boolean AT_valid(char *prompt) {
  char tokenBuffer[MAXDATA_len] = { 0 }, *n = NULL;
  float values[3]  = { 0 };
  boolean accepted = false;
  int i = 0;

  // Test if serial connection response
  if (strcmp( prompt, "at" ) == 0) 
  {
      // My FTDI has DTR(Data terminal ready) and CTS(Clear to send) pin.
      // I can connect one of these to the digital pin as input signal.
      // DTR is High serial is ready. 
      // CTS is High ready to receive data.

      if (digitalRead(ftdi_cts_pin) == HIGH) accepted = true;
  }

  // AT+PID=0,0,0
  if (strncmp(prompt, "at+pid=", 7) == 0) {
      strcpy(tokenBuffer, prompt + 7);

      n = strtok(tokenBuffer, ",");

      while(n) {
          if (i < 3 && isNum(n)) {
              values[i] = atof(n);
              i++;
          }
          n = strtok(NULL, ",");
      }

      if (i == 3)
      {
          kp = values[0];
          ki = values[1];
          kd = values[2];

          accepted = true;
      }
  }

  // Kp, Ki, Kd
  const char *v = prompt + 6;
  if (isNum(v)) {
      if (strncmp(prompt, "at+kp=", 6) == 0) {
          kp = atof(v); accepted = true;
      }
      if (strncmp(prompt, "at+ki=", 6) == 0) {
          ki = atof(v); accepted = true;
      }
      if (strncmp(prompt, "at+kd=", 6) == 0) {
          kd = atof(v); accepted = true;
      }
  }

  // Ramp
  v = prompt + 9;
  if (isNum(v)) {
    if (strncmp(prompt, "at+accel=", 9) == 0) {
      rampAccel = atof(v); accepted = true;
    }
    if (strncmp(prompt, "at+decel=", 9) == 0) {
      rampDecel = atof(v); accepted = true;
    }
  }

  // Speed 0-255
  v = prompt + 12;
  if (isNum(v)) {
    int pwm = atoi(v);
    if (strncmp(prompt, "at+maxspeed=", 12) == 0) {
      if (pwm >= 0 || pwm <= POT_RANGE) {
        maxSpeed = pwm;
        accepted = true;
      }
    }
  }

  // Motor Spin threshold
  v = prompt + 13;
  if (isNum(v)) {
    int range = atoi(v);
    if (strncmp(prompt, "at+motorspin=", 13) == 0) {
      if (range >= 0 || range <= maxSpeed) {
        motorSpinThreshold = range;
        accepted = true;
      }
    }
  }

  // Reboot avr
  if (strcmp(prompt, "at+reset") == 0) {
    Serial.println("OK");
    resetAVR();
  }

  // Write to eeprom
  if (strcmp(prompt, "at+write") == 0) {
      storage(WRITE_config);
      accepted = true;
  }
  
  // Show pid
  if (strncmp(prompt, "at+pidstat=", 11) == 0) {
      v = prompt + 11;
      if (isNum(v)) {
          showPid = atoi(v);
          accepted = true;
      }
  }

  // Help
  if (strcmp(prompt, "at+help")) 
      AT_help();

   // Queries
  if (strcmp(prompt, "at+kp?") == 0) Serial.println(kp); 
  if (strcmp(prompt, "at+ki?") == 0) Serial.println(ki);
  if (strcmp(prompt, "at+kd?") == 0) Serial.println(kd);
  if (strcmp(prompt, "at+accel?") == 0) Serial.println(RAMP_accel);
  if (strcmp(prompt, "at+decel?") == 0) Serial.println(RAMP_decel);
  if (strcmp(prompt, "at+maxspeed?") == 0) Serial.println(maxSpeed);
  if (strcmp(prompt, "at+motorspin?") == 0) Serial.println(motorSpinThreshold); 
  
  return accepted;
}


// Commandi seriali 
void ATCommand(char *prompt) {
    char *cmd = removeLn(prompt);

    if (AT_valid(cmd)) {
      // Command accepted
      Serial.println("OK");
    }

    // Clear for the next reception
    clearData(); 
}


void ac_on() {
    zero = true;
    i = 0;
    // Spegni triac
    PORTB &= ~(1 << 0);
    speedlevel = INVERTED_SCALE(output);
}


void setSpeed() {
    if (!zero) return;
    
    if (i >= speedlevel) {
        // Accendi triac
        PORTB |= (1 << 0);
        zero = false;
        i = 0;
    } else i++;
}


void startMotor() {
    if (motorEnabled) return;
    
    motorEnabled = motorAccel = true;
    alarm = false;
    beep();
}


void stopMotor() {
    motorEnabled = motorAccel = false;
    resetSent();
}


void buttons_listening() {
    buttonStart.tick();
    buttonStop.tick();
}


void setup () {
    Serial.begin( 115200 );
    
    pinMode( OPTO_pin, INPUT );
    pinMode( TRIAC_pin, OUTPUT );
    
    pid.SetSampleTime( 10 );
    pid.SetOutputLimits( 0, 255 );
    
    buttonStart.attachClick( startMotor );
    buttonStop.attachClick( stopMotor );
    
    Timer1.initialize( 40 );
    Timer1.attachInterrupt( setSpeed );
    
    attachInterrupt( digitalPinToInterrupt( OPTO_pin, ac_on, RISING ) );
    
    Wire.begin();
    Wire.setClock( 400000 );
    eeprom.setMemoryType( 256 );
    eeprom.begin( 0x50 );
  
    // Load settings
    storage(READ_config);
    
    beep();
}


// Letture dalla tachimetrica 
int readDynamo() {
    static unsigned int readings[numReadings];
    static unsigned int readIndex;
    static unsigned int total;
    
    total = total - readings[readIndex];  
    readings[readIndex] = map(DYNAMO_pin, 0, 1023, 0, 255);
    total = total + readings[readIndex];
    readIndex = readIndex + 1; 

    if (readIndex >= numReadings) 
    {
        readIndex = 0;
    }
  
    return (total / numReadings);
}


void loop() {
    static unsigned long lastTime = 0;
    int speedPot = map(POT_pin, 0, 1023, 0, maxSpeed);
    int tachPWM  = readDynamo();
    
    input = tachPWM;
    
    
    /*
        Controllo se la tachimetrica funzioni
    */
    if ( millis() - lastTime >= TACHTIMEOUT ) {
        
        // Soglia della tachimetrica
        if ( motorEnabled && speedPot >= motorSpinThreshold )
                          && tachPWM == 0 ) {
            
            // Alert
            pid.SetMode( MANUAL );
            stopMotor();
            alarm = true;
            
            Serial.println("Errore lettura tachimetrica");
        } 
        
        lastTime = millis();           
    } 
    
    // Nessun alarme
    if (motorEnabled && !alarm) {
        pid.SetMode(AUTOMATIC);
        
        // In accelerazione 
        if (motorAccel) {
            setpoint = min(speedPot, setPoint + rampAccel);
            
            // Motore inizia girare
            if (setPoint >= motorSpinThreshold) 
                motorFlag = Running;
                
            // Fine rampa
            if (setPoint >= speedPot) {
                motorAccel = false;
                motorFlag = ConstantSpeed;
            } else {
                motorFlag = Acceleration;
            }
        } else { 
            // Pid mantiene i giri del motore stabile 
            setPoint = speedPot;
        }
    } else {
        // In decelerazione 
        setPoint = max(0, setPoint - rampDecel);
        motorFlag = Deceleration;
        
        // Motore fermo
        if (setPoint == 0 && tachPWM == 0) {
            pid.SetMode(MANUAL);
            motorFlag = Idle;
        }
    }
    
    pd.SetTunings(kp, ki, kd);
    pid.Compute();
    

    // Seriale in ascolto
    if (Serial) {
        if (millis() - t1 >= SERIAL_interval) {
          serial_recv_r();
          
          // Visualizza reazione del pid in tempo reale
          if (showPid > 0) { 
              Serial.print("Pot: ");
              Serial.print(speedPot);
              Serial.print(" Sensor: ");
              Serial.println(tachPWM);

              showPid--;
          }
            
          t1 = millis();  
        }
    }
  
    buttons_listening();
    
    sendMessage(motorFlag);
}


    