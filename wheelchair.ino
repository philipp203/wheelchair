#include <PID_v1.h>

// --- Pin Definitions ---
const int POT_PIN = A0;      
const int M1_PWM = 5;        
const int M1_ENC_A = 2;       // Interrupt Pin
const int M2_PWM = 6;        
const int M2_ENC_A = 3;       // Interrupt Pin

// --- Constants ---
const float EDGES_PER_REV = 1350.0;       // 90 pulses * 2 (CHANGE interrupt)
const unsigned long PID_SAMPLE_TIME = 50; // PID updates at 20 Hz (every 50ms)
const float MAX_RPM = 92.0;             // 4mph with 10-inch wheel
const unsigned long TIMEOUT_US = 200000; // 0.2 seconds timeout (If stopped, RPM = 0)

// --- Constraints ---
const float MIN_ACTUAL_MPH = 0.2;        // Physical speed threshold required to activate motors
const int MAX_PWM_LIMIT = 235;          // 70% of 255 (255 * 0.70 = 178.5)

// --- Filter Configuration ---
const int POT_FILTER_SAMPLES = 20;       // Smooth potentiometer over 20 samples

// --- PID Tuning Parameters ---
double Kp = 5, Ki = 10, Kd = 0.0001;

// --- PID Variables ---
double Setpoint, Input1, Output1;
double Input2, Output2;

PID myPID1(&Input1, &Output1, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- Interrupt Accumulators (Volatiles) ---
volatile unsigned long m1_last_time = 0;
volatile unsigned long m1_total_duration = 0;
volatile unsigned int  m1_edge_count = 0;

volatile unsigned long m2_last_time = 0;
volatile unsigned long m2_total_duration = 0;
volatile unsigned int  m2_edge_count = 0;

unsigned long previousMillis = 0;

// --- Potentiometer Filter Variables ---
int pot_buffer[POT_FILTER_SAMPLES];
int pot_buffer_index = 0;
long pot_total_sum = 0;

// --- Interrupt Service Routines (ISR) ---
void measureMotor1() {
  unsigned long now = micros();
  if (m1_last_time > 0) {
    m1_total_duration += (now - m1_last_time);
    m1_edge_count++;
  }
  m1_last_time = now;
}

void measureMotor2() {
  unsigned long now = micros();
  if (m2_last_time > 0) {
    m2_total_duration += (now - m2_last_time);
    m2_edge_count++;
  }
  m2_last_time = now;
}

// --- 3-Point Median Filter Helper ---
double median3(double a, double b, double c) {
  if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
  if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
  return c;
}

void setup() {
  Serial.begin(115200);

  pinMode(POT_PIN, INPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M1_ENC_A, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), measureMotor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), measureMotor2, CHANGE);

  myPID1.SetMode(AUTOMATIC);
  myPID1.SetSampleTime(PID_SAMPLE_TIME);
  myPID1.SetOutputLimits(0, MAX_PWM_LIMIT);

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetSampleTime(PID_SAMPLE_TIME);
  myPID2.SetOutputLimits(0, MAX_PWM_LIMIT);
}

void loop() {
  // 1. Read Potentiometer and map to Target RPM
  int rawPotValue = analogRead(POT_PIN);
 
  pot_total_sum -= pot_buffer[pot_buffer_index];
  pot_buffer[pot_buffer_index] = rawPotValue;
  pot_total_sum += rawPotValue;
 
  pot_buffer_index = (pot_buffer_index + 1) % POT_FILTER_SAMPLES;
 
  int potValue = pot_total_sum / POT_FILTER_SAMPLES;

  long targetRPMScaled = map(potValue, 0, 1023, 0, (int)(MAX_RPM * 100));
  Setpoint = (double)targetRPMScaled / 100.0;

  unsigned long currentMillis = millis();
  unsigned long currentMicros = micros();
 
  // 2. Process updates every 50ms (20 Hz)
  if (currentMillis - previousMillis >= PID_SAMPLE_TIME) {
    previousMillis = currentMillis;

    // --- Safe Copies of Interrupt Data ---
    noInterrupts();
   
    unsigned long t_duration1 = m1_total_duration;
    unsigned int  count1      = m1_edge_count;
    unsigned long lastTime1   = m1_last_time;
    m1_total_duration = 0;
    m1_edge_count = 0;

    unsigned long t_duration2 = m2_total_duration;
    unsigned int  count2      = m2_edge_count;
    unsigned long lastTime2   = m2_last_time;
    m2_total_duration = 0;
    m2_edge_count = 0;
   
    interrupts();

    double i2 = 0, i1 = 0;

    // --- Motor 1 Average Speed Processing ---
    if (currentMicros - lastTime1 > TIMEOUT_US || lastTime1 == 0) {
      i1 = 0; 
    } else if (count1 > 0) {
      float avg_edge_duration = (float)t_duration1 / count1;
      i1 = 60000000.0 / (avg_edge_duration * EDGES_PER_REV);
    }

    // --- Motor 2 Average Speed Processing ---
    if (currentMicros - lastTime2 > TIMEOUT_US || lastTime2 == 0) {
      i2 = 0;
    } else if (count2 > 0) {
      float avg_edge_duration = (float)t_duration2 / count2;
      i2 = 60000000.0 / (avg_edge_duration * EDGES_PER_REV);
    }

    // --- 3-Point Median Filter Buffers ---
    static double m1_history[3] = {0, 0, 0};
    static double m2_history[3] = {0, 0, 0};

    // Shift history values
    m1_history[0] = m1_history[1];
    m1_history[1] = m1_history[2];
    m1_history[2] = i1;

    m2_history[0] = m2_history[1];
    m2_history[1] = m2_history[2];
    m2_history[2] = i2;

    // Apply median filter to inputs
    Input1 = median3(m1_history[0], m1_history[1], m1_history[2]);
    Input2 = median3(m2_history[0], m2_history[1], m2_history[2]);

    // Convert Actual RPMs into actual MPH
    float currentMPH1 = (Input1 * 31.416 * 60.0) / 63360.0;
    float currentMPH2 = (Input2 * 31.416 * 60.0) / 63360.0;

    // --- Linked Safety Check ---
    if (currentMPH1 < MIN_ACTUAL_MPH || currentMPH2 < MIN_ACTUAL_MPH) {
      Output1 = 0;
      Output2 = 0;
     
      if (myPID1.GetMode() == AUTOMATIC) {
        myPID1.SetMode(MANUAL);
        myPID2.SetMode(MANUAL);
        myPID1.SetMode(AUTOMATIC);
        myPID2.SetMode(AUTOMATIC);
      }
    } else {
      myPID1.Compute();
      myPID2.Compute();
    }

    // 4. Actuate Motors
    analogWrite(M1_PWM, Output1);
    analogWrite(M2_PWM, Output2);

    // 5. Telemetry
    float targetMPH = (Setpoint * 31.416 * 60.0) / 63360.0;
    Serial.print("Target_MPH:"); Serial.print(targetMPH, 2);
    Serial.print(",M1_MPH:"); Serial.print(currentMPH1, 2);
    Serial.print(",M2_MPH:"); Serial.print(currentMPH2, 2);
    Serial.print(",M1_PWM:"); Serial.print((float)Output1/17);
    Serial.print(",M2_PWM:"); Serial.println((float)Output2/17);
  }
}