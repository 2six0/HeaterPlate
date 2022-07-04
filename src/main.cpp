#include <thermistor.h>
#include <Arduino.h>
#define ssr 5

thermistor therm1(A0, 0); // Analog Pin which is connected to the 3950 temperature sensor, and 0 represents TEMP_SENSOR_0 (see configuration.h for more information).

unsigned long millis_now = 0;
unsigned long millis_before, millis_before_2;
unsigned long millis_before_3;

float kalmanFilterData;
float Xt, XtUpdate, Xtprev;
float Pt, PtUpdate, Ptprev;
float kt, R, Q;

float temp;
float pwm_value = 255;
float temp_setpoint = 0;
float seconds = 0;
int runMode = 0;

float MIN_PID_VALUE = 0;
float MAX_PID_VALUE = 180; // Max PID value. You can change this.
float Kp = 2;              // Mine was 2
float Ki = 0.0025;         // Mine was 0.0025
float Kd = 9;              // Mine was 9
float PID_Output = 0;
float PID_P, PID_I, PID_D;
float PID_ERROR, PREV_ERROR;

void count_seconds();
void PID_controller();

void setup()
{
  // put your setup code here, to run once:
  pinMode(ssr, OUTPUT);
  Serial.begin(9600); // initialize port serial at 9600 Bauds.
  R = 10;             // 10
  Q = 0.1;            // 0,2
  Ptprev = 1;
}

void loop()
{
  count_seconds();

  if(millis() - millis_before_3 > 20){
  temp = therm1.analog2temp(); // read temperature
  XtUpdate = Xtprev;
  PtUpdate = Ptprev + Q;
  kt = PtUpdate / (PtUpdate + R);
  Xt = XtUpdate + (kt * (temp - XtUpdate));
  Pt = (1 - kt) * PtUpdate;
  Xtprev = Xt;
  Ptprev = Pt;
  kalmanFilterData = Xt;
  millis_before_3 = millis();
  }

  PID_controller();
  Serial.print(seconds);
  Serial.print(", ");
  Serial.print(kalmanFilterData);
  Serial.print(", ");
  Serial.print(PID_Output);
  Serial.print(", ");
  Serial.println(pwm_value);
}

void count_seconds()
{
  millis_now = millis();
  if (millis_now - millis_before_2 > 1000)
  {
    millis_before_2 = millis_now;
    seconds++;
    // Serial.println(seconds);
  }
}

void PID_controller()
{
  millis_now = millis();
  if (millis_now - millis_before > 50)
  {

    if (kalmanFilterData < 140)
    {
      temp_setpoint = seconds * 1.666; // step 1: setpoint = seconds * 1.666 per derajat celcius
    }
    // Calculate PID
    PID_ERROR = temp_setpoint - kalmanFilterData;
    PID_P = Kp * PID_ERROR;
    PID_I = PID_I + (Ki * PID_ERROR);
    PID_D = Kd * (PID_ERROR - PREV_ERROR);
    PID_Output = PID_P + PID_I + PID_D;
    // Define maximun PID values
    if (PID_Output > MAX_PID_VALUE)
    {
      PID_Output = MAX_PID_VALUE;
    }
    else if (PID_Output < MIN_PID_VALUE)
    {
      PID_Output = MIN_PID_VALUE;
    }
    pwm_value = 255 - PID_Output;
    PREV_ERROR = PID_ERROR;

    analogWrite(ssr, 0);

    millis_before = millis_now; // Update millis_before
  }
}