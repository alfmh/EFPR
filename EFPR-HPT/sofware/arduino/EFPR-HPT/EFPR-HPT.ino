/*
****************************************************************************************************************
*** Project: EFPR (Electronics Fuel Pressure Regulator)              Date:                                   
*** Firmware version: v1.3                                                                                   ***
*** Description: Design EFPR for an EFI (Electronic Fuel Injection) used to control a his fuel pump.         ***
*** The purpose of this project is to replace the carburetor of a small combustion engine with an EFI sytem. ***
*** This serves as a propulsion system for a UAV (Unmanned Areal Vehicle).                                   ***
***                                                                  Company: https://www.loweheiser.com     ***
***                                                                  Author: Alfonso Muñoz Hormigo           *** 
****************************************************************************************************************
*/
  #include <AutoPID.h>
/*
*** FUEL PUMP ***
    Magnetic coupled gear pump (seal-less):
    HP-Tech DC Fuel Pump ZP25M14F https://www.hptech.at/english/products/dc-pump/
    Specs:
          - Nominal Voltage: 6 [V]
          - Nominal Pressure: 3.5 [bar]
          - Flow Rate: 190 [ml/min] = 11.4 [l/s]

*** FUEL PRESSURE SENSOR ***
    Analog fuel pressure sensor:
    MPX5700DP  -> Upper limit = 7 [bar] https://www.farnell.com/datasheets/2291495.pdf
    Specs:
          - Sensitivity of the sensor = 6.4 [mV/KPa] at VS = 5 [V]   

*** MICROCONTROLLER ***
    Seeeduino XIAO I/O works at 3.3 [V] https://wiki.seeedstudio.com/Seeeduino-XIAO/
    VS = 5 [V]
*/

// *****************************************
// *** I/O MICROCONTROLLER CONFIGURATION ***
// *****************************************
#define V_PS_MEASURE_IN A5
#define ECU_ENABLE_FP_IN A7
#define VBAT_SENSE_IN A8
#define FP_PWM_OUT A1

// ******************************
// *** PARAMETERS DEFINITIONS ***
// ******************************
// MICROCONTROLLER
#define ADC_MAX 4095                     // 12 bits
#define VS_uC 3300.0f                    // [mV]
#define VBAT_VOLTAGE_DIVIDER 4.94f       // [] -> (10.2k / 50.4k) = 1 / 4.94
#define FP_ENABLE_THRESHOLD 2000 
#define PWM_FREQUENCY 100000.0f          // [Hz]
#define VS_DOCUMENTATION 5000.0f         // [mV]
#define PS_SENSIVITY_DOCUMENTATION 6.4f  // [mV/KPa]
#define PRESSURE_SETPOINT 2.5f           // [bar]
#define PS_OFFSET 0.31f                  // [bar]
#define PS_VOLTAGE_DIVIDER 0.6f          // [] -> 3.3k / 5.5k = 0.6

// FUEL PUMP
#define VFP_MAX 5.0f    // [V]
#define OUTPUT_MIN 0
#define OUTPUT_MAX 1024 // [] -> 10 bits range 

// PID CONTROL
#define NO_BANG_BANG -1000.0f
#define KP 1.0f  //1.7
#define KI 0.0f  //1.2
#define KD 0.0f

// *****************
// *** VARIABLES ***
// *****************
double outputMin = (double) OUTPUT_MIN;
double outputMax = 0.2f * OUTPUT_MAX;
double setPoint = PRESSURE_SETPONT;
double pressure, pressureFiltered, output, error;
float sensivity, pressureBar, vBat, offset;
uint16_t pwmOut, fpEnable;
bool ledState = HIGH;
float printSetPoint;

// ***************
// *** OBJECTS ***
// ***************
AutoPID myPID(&pressure, &setPoint, &output, outputMin, outputMax, KP, KI, KD);

// ********************
// *** MAIN PROGRAM ***
// ********************
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  analogReadResolution(12);
  
  // I/0 Setup
  pinMode(ECU_ENABLE_FP_IN, INPUT);
  analogWrite(ECU_ENABLE_FP_IN, ADC_MAX);
  pinMode(V_PS_MEASURE_IN, INPUT);
  analogWrite(V_PS_MEASURE_IN, 0);
  pinMode(VBAT_SENSE_IN, INPUT);
  analogWrite(VBAT_SENSE_IN, 0);
  pinMode(FP_PWM_OUT, OUTPUT);
  digitalWrite(FP_PWM_OUT, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  //setPoint = (double)((PRESSURE_SETPOINT * PS_SENSIVITY_DOCUMENTATION * (float)(ADC_MAX)*PS_VOLTAGE_DIVIDER * 100.0f) / VS_uC);
  offset = (double)((PS_OFFSET * PS_SENSIVITY_DOCUMENTATION * (float)(ADC_MAX)*PS_VOLTAGE_DIVIDER * 100.0f) / VS_uC);

  // Remove the bang bang control
  myPID.setOutputRange(outputMin, outputMax);
  myPID.setBangBang (NO_BANG_BANG, NO_BANG_BANG);
  //set PID update init interval to 0.1ms
  myPID.setTimeStep(0.001);
  pwm(FP_PWM_OUT,PWM_FREQUENCY,(uint16_t)(output));
}

void loop() {
  // put your main code here, to run repeatedly:
  vBat = ((float)(analogRead(VBAT_SENSE_IN)) / (float)(ADC_MAX)) * VS_uC * VBAT_VOLTAGE_DIVIDER;

  pressure = analogRead(V_PS_MEASURE_IN) - offset;
  pressureBar = pressure * VS_uC / (ADC_MAX * PS_SENSIVITY_DOCUMENTATION * PS_VOLTAGE_DIVIDER * 100.0);
  
  outputMax = (double)((VFP_MAX / (vBat / 1000.0f)) * (float)(OUTPUT_MAX));
  myPID.setOutputRange(outputMin, outputMax);
 
  fpEnable = analogRead(ECU_ENABLE_FP_IN);
  if (fpEnable <= FP_ENABLE_THRESHOLD) {
    //Serial.printf("Dentro\r\n");
    myPID.run();
    printSetPoint = PRESSURE_SETPOINT;
  } else {
    //Serial.printf("Fuera\r\n");
    output = outputMax;
    printSetPoint = 0.0f;
    myPID.reset();
  }
  
  if (output < OUTPUT_MIN)
    output = OUTPUT_MIN;
  else if (output > outputMax)
    output = outputMax;

  pwm(FP_PWM_OUT,PWM_FREQUENCY,(uint16_t)(output));

  ledState = ((pressureBar >= (PRESSURE_SETPOINT - 0.1f)) && (pressureBar < (PRESSURE_SETPOINT + 0.1f))) ? LOW : HIGH;
  digitalWrite(LED_BUILTIN, ledState);
  //Serial.printf("Pressure: %.2f [bar] | Error: %.2f \r\n", pressureBar, output);
  Serial.printf("%0.2f|%0.2f\r\n", printSetPoint, pressureBar);

}