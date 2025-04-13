// Serial USB Terminal settings: send LF. Baudrate=115200
// Regolatore di tensione da 6.2kW con sistema di feedback a closed loop.

/* Command list. at+
                 pid=kp,ki,kd
                 kp= 
                 ki= 
                 kd= 
                 pidstat=
                 kp? 
                 ki? 
                 kd?
                 at 
                 wr 
                 reset
                 accel=
                 decel=
                 maxspeed%=
                 motorspin=
                 cutoff=
                 timeout=

  Atari_3v - Successivo di atari_2v con una aggiunta di timeout    

  Da fare:
            Motore Tapis da 3.5HP 180 dc 5000 rpm. MaxPotRange=199.41, MaxSpeed=78.2% for Max_DC_OUTPUT=180
            
            In AT_Valid() in at+maxspeed% at+motorspoin 
            MaxSpeed < motorSpinThreshold then MaxSpeed = motorSpinThreshold
            float analogSpinThreshold = voltInAnalog(motorSpinThreshold);

            Controllo ausiliari di potenza
*/
#include <Wire.h>
#include <SparkFun_External_EEPROM.h>
#include <OneButton.h>
#include <TimerOne.h>
#include <PID_v1.h>

#include "Pinout.h"

#define ARRAY_len(x)        (sizeof(x) / sizeof(x[0]))
#define SERIAL_interval     40 // Delay for Rx e Tx
#define PIDSTAT_delay       40 
#define MAXDATA_len         30 // Serial buffer
#define SENSOR_READINGS     50 // Delay(5)x20 100ms 0.1s
#define AC_main          233.0 // AC Main
#define AVR_VRef          4.92 // Vcc
#define DIV_VRef          5.20 // Divider
#define POT_RANGE        255.0 // Potenziometer spectrum
#define INVERTED_SCALE(x) (POT_RANGE - (x)) // AC mode

double RAMP_accel = 5.0; // Acceleration
double RAMP_decel = 0.1; // Deceleration


volatile bool zc = false;
volatile bool inRush = false; 
volatile float i = 0;
volatile float speedlevel = POT_RANGE;
float pot = 0.0;
int auxiliary_ok = 0;
bool motorEnable = false;
typedef enum { IsSpinning, NotSpinning } Motor;
volatile Motor motorFlag = NotSpinning;
double maxSpeed = 1023.0; // 100% = 230 Volts
double motorSpinThreshold = 0.30; // Volts
bool autostart = false;
int dir = 0; // CW CCW
char receive_buf[MAXDATA_len] = { 0 };
int iterate_pidstat = 0;
uint32_t t1 = 0;
float sensorOutput = 0.0;
double cutoff = 0.20; // Volts 

 // It's double because all settings are in floating point numbers
 // I have to cast it in unsigned long later.
double motorTimeout = 1.0; 
unsigned long motorOff = 100;
bool motorIdle = false;

struct SerialMsg {
  const char *text;
  bool sent;
} messages[] = {
  { "Motore fermo", false }, { "Motore gira", false },
  { "Fine accelerazione", false }, { "Fine decelerazione", false }
};

double kp = 1.90, ki = 2.10, kd = 0.05;
double input = 0.0, setpoint = 0.0, output = 0.0;

// Setup da salvare sulla eeprom esterna.
double *settings[] = {
  &kp, &ki, &kd, &RAMP_accel, &RAMP_decel, &maxSpeed,
  &motorSpinThreshold, &cutoff, &motorTimeout
};

ExternalEEPROM eeprom;

PID pid(&input, &output, &setpoint, kp, ki, kd, P_ON_M, DIRECT);


OneButton buttonStop(stop_pin);
OneButton buttonStart(start_pin);


/*
=====================
 void sendMessage
=====================
*/
void sendMessage(const int x) 
{
  if (Serial) {
      if (!messages[x].sent) {
        Serial.println(messages[x].text);
        messages[x].sent = true;
      }
  }
}

/*
=====================
 void resetSent
=====================
*/
void resetSent()
{
  for (size_t i = 0; i < ARRAY_len(messages); i++) {
    messages[i].sent = false;
  }
}

/*
=====================
 void beep
=====================
*/
void beep()
{ 
  tone(buz_pin, 4000, 200); 
}

/*
=====================
 void resetAVR
=====================
*/
void resetAVR()
{
  // Close serial if connected before resetting the avr
  // otherwise we receive garbage data.
  if (Serial) Serial.end(); delay(100); 
  asm volatile ("jmp 0");
}

/*
=====================
 int scanDelay
=====================
*/
int scanDelay(unsigned long timeout)
{
  static unsigned long start = millis();
  if (millis() - start >= timeout) {
    start = millis();
    return 0;
  }
  return 1;
}

/*
=====================
 void setup
=====================
*/
void setup() 
{

  configure_board();

  buttonStart.attachClick(startMotor);
  buttonStop.attachClick(stopMotor);

  // Firing angle calculation :: 50Hz-> 10ms (1/2 Cycle)
  // (10000us - 10us) / 128 = 75 (Approx)
  long us = (long) ((1000000.0 / getFrequency()) / POT_RANGE) + 1;
  Timer1.initialize(us);
  Timer1.attachInterrupt(setSpeed);

  attachInterrupt(digitalPinToInterrupt(opto_pin), ac_load, RISING);

  pid.SetSampleTime(10); 

  // In my case 24LC256, 400khz, total of 32768 byte free space, 512 pages,
  // and 1 page 64 byte writable approximately in 5ms.
  Wire.begin();
  Wire.setClock(400000);
  eeprom.setMemoryType(256);
  eeprom.begin(0x50);
  
  Serial.begin(115200);
  // Plx-Daq
  // Serial.begin(9600);
  // Serial.println("CLEARDATA");
  // Serial.println("LABEL,Time,Setpoint,Input");

  loadSettings();

  motorDir();

  beep();

  t1 = millis();
}

/*
=====================
 void ac_load 
=====================
*/
void ac_load()
{
  zc = true;
  i = 0;
  
  digitalWrite(triac_pin, LOW);
  speedlevel = INVERTED_SCALE(output);
}

/*
=====================
 void setSpeed 
=====================
*/
void setSpeed()
{
  if (zc == false) return;
  if (i >= speedlevel) {
      digitalWrite(triac_pin, HIGH);
      zc = false;
      i = 0; 
  } else i++;
}

/*
=====================
 double getFrequency 
=====================
*/
// PulseIn block port during the measurements. Use it only in setup.
double getFrequency()
{
  unsigned long hi, lo;
  double freq, period;

  hi = pulseIn(opto_pin, HIGH);
  lo = pulseIn(opto_pin, LOW);

  period = hi + lo;

  if (period == 0)
    freq = 0;
  else
    freq = 1000000.0 / period;

  return freq;
}

/*
=====================
 float mapf
=====================
*/
float mapf(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
=====================
 void start 
=====================
*/
void startMotor()
{
   if (motorEnable || autostart) return;

   motorEnable = inRush = true;
   motorIdle = false;
   beep();
}

/*
=====================
 void stop 
=====================
*/
void stopMotor()
{ 
  motorEnable = inRush = false;
  resetSent();
}

/*
=====================
 void buttons_listening
=====================
*/
void buttons_listening()
{
  buttonStop.tick();
  buttonStart.tick();
}

/*
=====================
 float potVoltage
=====================
*/
float potVoltage()
{
  return analogRead(pot_pin) * AVR_VRef / 1023.0;
}

/*
=====================
 float triacAC_voltage 
=====================
*/
// Get AC output from triac.
float triacAC_voltage()
{
   return 0;
}

/*
=====================
 float mainAC_voltage 
=====================
*/
float mainAC_voltage() 
{ 
  return 0; 
}

/*
=====================
 float voltInAnalog
=====================
*/
float voltInAnalog(float volts)
{
  return (volts / AVR_VRef) * 1023.0;
}

/*
=====================
 float timeInAnalog
=====================
*/
float timeInAnalog(float x)
{
  return mapf(x, 0.0, POT_RANGE, 0.0, 1023.0);
}

/*
=====================
 float analogInTime
=====================
*/
float analogInTime(float x)
{
  return mapf(x, 0.0, 1023.0, 0.0, POT_RANGE);
}

/*
=====================
 float analogInVoltage
=====================
*/
float analogInVolt(float scaleFactor)
{
  return (scaleFactor * AVR_VRef) / 1023.0;
}

/*
=====================
 float readSensor
=====================
*/
// Partitore di tensione collegato al Triac.
float readSensor()
{
  static float voltages[SENSOR_READINGS] = { 0 };
  static float peakVoltage = 0.0;
  static float total = 0;
  static int readIndex = 0;

  // Legge la tensione in uscita dal sensore. Max DIV_VRef.
  if (scanDelay(5) == 0) {
    float reading = analogInVolt(analogRead(div_pin));


    total = total - voltages[readIndex];
    voltages[readIndex] = reading;
    total = total + reading;

    readIndex = (readIndex + 1) % SENSOR_READINGS;

    peakVoltage = total / SENSOR_READINGS;
  }

  return peakVoltage;
}

/*
=====================
 float powerlv
=====================
*/
float powerlv(float x)
{
  sensorOutput = readSensor();

  if ( motorEnable && sensorOutput >= motorSpinThreshold ) {
    motorFlag = IsSpinning;
    sendMessage(1);
  }

  if ( x >= maxSpeed ) x = maxSpeed;

  return analogInTime( x );
}

/*
=====================
 void motorDir
=====================
*/
#define MAX_DEBOUNCE 100

void motorDir()
{
  static bool waitMotor = false;
  static int selPosition[MAX_DEBOUNCE] = { 0 };
  static int countDebouncing = 0;
  bool isFlipped = false;

  selPosition[countDebouncing] = digitalRead(selector_pin);

  // Anti-debounce
  if (countDebouncing >= MAX_DEBOUNCE - 1) {
    for (int i = 0; i < MAX_DEBOUNCE; i++) {
      if (selPosition[i] != dir) {
        isFlipped = true; 
      } else {
        isFlipped = false; 
        break;
      }
    }
  }

  // int selPosition = digitalRead(selector_pin);
  // if (dir != selPosition) {
  //   isFlipped = true;
  //   dir = !dir;
  // }

  countDebouncing = (countDebouncing + 1) % MAX_DEBOUNCE;

  // Change direction
  if (isFlipped) {
    dir = !dir;
    if (!waitMotor && motorFlag == IsSpinning) {
      waitMotor = autostart = true;
      stopMotor();
    } else {
      digitalWrite(rel1_pin, dir);
    }
  }

  // Wait motor to stop
  if (autostart) {
    if (waitMotor && motorFlag == NotSpinning) {
      digitalWrite(rel1_pin, dir);
      waitMotor = autostart = false;
      startMotor();
    }
  }
}

/*
=====================
 void serial_recv_r 
=====================
*/
// Receiver 
void serial_recv_r()
{
  int ii = 0;

  while (Serial.available() > 0) {
    if (ii >= MAXDATA_len) 
      break;
    receive_buf[ii++] = Serial.read();
  }

  if (ii > 0) { ATCommand(receive_buf); }
}

/*
=====================
 void clearData
=====================
*/
void clearData()
{
  memset(receive_buf, 0, MAXDATA_len);
}

/*
=====================
 char *removeLn
=====================
*/
char *removeLn(char *str)
{
  size_t n = strcspn(str, "\n");

  if (n) *(str + n) = '\0';
  return str;
}

/*
=====================
 char *lowerCase
=====================
*/
char *lowerCase(char *str)
{
  for (int i = 0; str[i] != '\0'; i++)
    str[i] = tolower(str[i]);
  return str;
}

/*
=====================
 char *trim
=====================
*/
char *trim(char *str)
{
  char *end, *start = str;
  
  while (isspace(*start)) start++;
  end = start + strlen(start) - 1;

  while(end > start && isspace(*end)) 
    end--;
  
  end[1] = '\0';
  memmove(str, start, end - start + 2);

  return str;
}

/*
=====================
 bool isNum
=====================
*/
bool isNum(const char *str)
{
  int dotseen = 0;

  if (*str == '\0') return false;

  while (*str) {
    if (*str == '.') {
      if (dotseen) return false;
      dotseen = 1;
    } else if (!isdigit(*str)) {
      return false;
    }
    str++;
  }

  return true;
}

/*
=====================
 bool strcmpl
=====================
*/
bool strcmpl(const char *str1, const char *str2)
{
  if (strncmp(str1, str2, strlen(str2)) == 0 && strlen(str1) == strlen(str2))
    return true;
  return false;
}

/*
=====================
 void PID_stat
=====================
*/
void PID_stat()
{
  
  static unsigned long last_ms = 0;

  if ( iterate_pidstat <= 0 )
    return;

  if ( millis() - last_ms >= PIDSTAT_delay ) {
    Serial.print("Pot: ");
    Serial.print(analogInVolt(timeInAnalog(pot)));
    Serial.print(" Set: ");
    Serial.print(analogInVolt(timeInAnalog(setpoint)));
    Serial.print(" Sensor: ");
    Serial.println(analogInVolt(timeInAnalog(input)));

    iterate_pidstat--;

    last_ms = millis();
  }
}

/*
=====================
 bool AT_valid
=====================
*/
bool AT_valid(char *prompt)
{
  char tokenBuffer[MAXDATA_len] = { 0 }, *n = NULL;
  float values[3]  = { 0 };
  bool accepted = false;
  int i = 0;

  // Test if serial connection works.
  if (strcmpl(prompt, "at")) 
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
  if (isNum(v))
  {
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

  // Ramps.
  v = prompt + 9;
  if (isNum(v)) {
    if (strncmp(prompt, "at+accel=", 9) == 0) {
      RAMP_accel = atof(v); accepted = true;
    }

    if (strncmp(prompt, "at+decel=", 9) == 0) {
      RAMP_decel = atof(v); accepted = true;
    }
  }

  // Speed in percentage.
  v = prompt + 13;
  if (isNum(v)) {
    int perc = atof(v);
    // From (0% to 100%) percentage in (0 to POT_RANGE) 
    if (strncmp(prompt, "at+maxspeed%=", 13) == 0) {
      if (perc >= 0.0 || perc <= 100.0) {
        maxSpeed = timeInAnalog((POT_RANGE * perc) / 100.0);
        accepted = true;
      }
    }
  }

  // Motor Spin threshold
  v = prompt + 13;
  if (isNum(v)) {
    float voltage = atof(v);
    if (strncmp(prompt, "at+motorspin=", 13) == 0) {
      if (voltage >= 0.0 || voltage <= DIV_VRef) {
        motorSpinThreshold = voltage;
        accepted = true;
      }
    }
  }

  // Motor stop threshold
  v = prompt + 10;
  if (isNum(v)) {
    float voltage = atof(v);
    if (strncmp(prompt, "at+cutoff=", 10) == 0) {
      if (voltage >= 0.0 || voltage <= DIV_VRef) {
        cutoff = voltage;
        accepted = true;
      }
    }
  }

  // Motor Timeout 
  v = prompt + 11;
  if (isNum(v)) {
    if (strncmp(prompt, "at+timeout=", 11) == 0) {
      motorTimeout = max(1.0, atof(v));
      accepted = true;
    }
  }

  // Reboot avr.
  if (strcmpl(prompt, "at+reset")) {
    Serial.println("OK");
    resetAVR();
  }

  // Write to eeprom.
  if (strcmpl(prompt, "at+wr")) {
      saveSettings();
      accepted = true;
  }
  
  // Help
  if (strcmpl(prompt, "at+help")) {
    AT_help();
  }

   // Queries
  if (strcmpl(prompt, "at+kp?")) Serial.println(kp); 
  if (strcmpl(prompt, "at+ki?")) Serial.println(ki);
  if (strcmpl(prompt, "at+kd?")) Serial.println(kd);
  if (strcmpl(prompt, "at+accel?")) Serial.println(RAMP_accel);
  if (strcmpl(prompt, "at+decel?")) Serial.println(RAMP_decel);

  if (strcmpl(prompt, "at+cutoff?")) { 
    Serial.print(cutoff); Serial.println(" volts"); 
  }
  if (strcmpl(prompt, "at+maxspeed?")) { 
    Serial.print((analogInTime(maxSpeed) * 100.0) / POT_RANGE, 0); Serial.println("%"); 
  }
  if (strcmpl(prompt, "at+motorspin?")) { 
    Serial.print(motorSpinThreshold); Serial.println(" volts");
  }
  if (strcmpl(prompt, "at+timeout?")) 
    Serial.println((unsigned long) motorTimeout);


  if (strncmp(prompt, "at+pidstat=", 11) == 0)
  {
      v = prompt + 11;
      
      if (isNum(v)) {
          iterate_pidstat = atoi(v);
          accepted = true;
      }
  }

  return accepted;
}

/*
=====================
 void ATCommand
=====================
*/
void ATCommand(char *prompt)
{
  char *str = trim(removeLn(lowerCase(prompt)));

  if (AT_valid(str)) {
      // Real time tuning.
      pid.SetTunings(kp, ki, kd);
      // Done.
      Serial.println("OK");
  }

  // Clear for the next reception.
  clearData();
}

/*
=====================
 void AT_help
=====================
*/
void AT_help()
{
  const char *comandList[] = {
    "at+kp=", "at+ki=", "at+kd=", "at+pid=kp,ki,kd", "at+pidstat=",
    "at+accel=", "at+decel=", "at+reset", "at+wr", "at",
    "at+cutoff=", "at+maxspeed%=", "at+motorspin=", "at+timeout="
  };

  for (size_t i = 0; i < ARRAY_len(comandList); i++) {
    Serial.print(comandList[i]);
    Serial.print(" ");
  }

  Serial.println("use ? for query settings");
}

/*
=====================
 void saveSettings
=====================
*/
// Abbiamo 1kb di spazio libero. Se tutti i dati sono
// del tipo float 4 byte, abbiamo
// 1024/4=256 indirizzi.
// ADDR 0 Kp
// ADDR 4 Ki
// ADDR 8 Kd
void saveSettings()
{
  uint32_t addr = 0;

  if (!eeprom.isConnected(0x50)) {
    Serial.println("EEPROM: error writing!\r\n");
    return;
  }

  for (size_t i = 0; i < ARRAY_len(settings); i++) {
    eeprom.put(addr, *settings[i]);
    addr += sizeof(float);
    delay(7);
  }

  beep();
}

/*
=====================
 void loadSettings
=====================
*/
void loadSettings()
{
  if (!eeprom.isConnected(0x50) || eeprom.isBusy(0x50)) {
    if (Serial) Serial.println("EEPROM: error reading!");
    return;
  }

  for (size_t i = 0; i < ARRAY_len(settings); i++) {
    eeprom.get(i * sizeof(float), *settings[i]);
  }
}

/*
=====================
 void plx_daq
=====================
*/
void plx_daq()
{
  static unsigned long wait = millis();

  if (millis() - wait >= SERIAL_interval) {
    Serial.print("DATA,TIME,");
    Serial.print(setpoint, 1);
    Serial.print(",");
    Serial.println(input, 1);
    wait = millis();
  }
}

/*
=====================
 void loop
=====================
*/
void loop() 
{  
  // Potenziometer 0<=>POT_RANGE
  pot   = powerlv(analogRead(pot_pin)); 
  // Scale 1:1 0<=>Potenziometer, regulated by pid
  input = analogInTime(voltInAnalog(sensorOutput));


  if (motorEnable) {
    // Turn pid on
    pid.SetMode(AUTOMATIC);

    if (inRush) {
        // Acceleration
        setpoint = min(pot, setpoint + RAMP_accel);
        if (setpoint >= pot) inRush = false;
    } else {
        // Pid wants to be always updated      
        setpoint = pot;
    }
  } else {
      // Deceleration
      setpoint = max(0.0, setpoint - RAMP_decel);

      // At a certain threshold, I shut everything down
      if (sensorOutput <= cutoff && motorIdle == false) {   
          motorOff = millis();
          motorIdle = true;
      } 

      if (motorIdle) {
          // Additional end time
          if (millis() - motorOff >= (unsigned long) motorTimeout) {
              motorFlag = NotSpinning;
              sendMessage(0);
          }

          speedlevel     = POT_RANGE;
          input          = 0.0;
          setpoint       = 0.0;
          output         = 0.0;

          // Pid off
          pid.SetMode(MANUAL);
      }
  }

  pid.SetTunings(kp, ki, kd);
  pid.Compute();


  if (Serial) {
    if (millis() - t1 >= SERIAL_interval) {
      serial_recv_r();
      t1  = millis();
    }
  }
  
  PID_stat();

  //plx_daq();

  motorDir();

  buttons_listening();  
}

/* eof */

