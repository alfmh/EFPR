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
#include <TimerTCC0.h>
#include <TimerTC3.h>

/*
   MICROCONTROLLER
*/
//PINS
#define V_PS_MEASURE_IN A5
#define ECU_ENABLE_FP_IN A7
#define VBAT_SENSE_IN A8
#define FP_PWM_OUT A1
//PARAMETERS
#define ADC_MAX 4095        // 12 bits
#define VS_uC 3300.0f              // [mV]
#define VBAT_VOLTAGE_DIVIDER 4.94f // [] -> (10.2k / 50.4k) = 1 / 4.94
#define FP_ENABLE_THRESHOLD 2000 
#define PWM_FREQUENCY 120000.0f    // [Hz]

/*
*** FUEL PUMP ***
    Magnetic coupled gear pump (seal-less):
    HP-Tech DC Fuel Pump ZP25M14F https://www.hptech.at/english/products/dc-pump/
    Specs:
          - Nominal Voltage: 6 [V]
          - Nominal Pressure: 3.5 [bar]
          - Flow Rate: 190 [ml/min] = 11.4 [l/s]
*/
/*  
*** FUEL PRESSURE SENSOR ***
    Analog fuel pressure sensor:
    MPX5700DP  -> Upper limit = 7 [bar] https://www.farnell.com/datasheets/2291495.pdf
    Specs:
          - Sensitivity of the sensor = 6.4 [mV/KPa] at VS = 5 [V]   
*/
/*
*** MICROCONTROLLER ***
    Seeeduino XIAO I/O works at 3.3 [V] https://wiki.seeedstudio.com/Seeeduino-XIAO/
    VS = 5 [V]
*/

// *** PARAMETERS DEFINITIONS ***
// PRESSURE SENSOR
#define VS_DOCUMENTATION 5000.0f         // [mV]
#define PS_SENSIVITY_DOCUMENTATION 6.4f  // [mV/KPa]
#define PRESSURE_SETPOINT 2.5f           // [bar]
#define PS_OFFSET 0.31f                  // [bar]
#define PS_VOLTAGE_DIVIDER 0.6f          // [] -> 3.3k / 5.5k = 0.6

// FUEL PUMP
#define VFP_MAX 6.0f  // [V]
#define OUTPUT_MIN 0
#define OUTPUT_MAX 1024 // [] -> 10 bits range 

// PID CONTROL
#define NO_BANG_BANG -1000.0f
#define KP 5.0f  //1.7
#define KI 1.5f  //1.2
#define KD 0.0f

// *** VARIABLES ***
double outputMin = (double) OUTPUT_MIN;
double outputMax = 0.2f * OUTPUT_MAX;
double pressure, setPoint, output, error;
float sensivity, pressureBar, vBat, offset;
uint16_t pwmOut, fpEnable;
bool ledState = HIGH;

AutoPID myPID(&pressure, &setPoint, &output, outputMin, outputMax, KP, KI, KD);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  analogReadResolution(12);
  
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

  setPoint = (double)((PRESSURE_SETPOINT * PS_SENSIVITY_DOCUMENTATION * (float)(ADC_MAX)*PS_VOLTAGE_DIVIDER * 100.0f) / VS_uC);
  offset = (double)((PS_OFFSET * PS_SENSIVITY_DOCUMENTATION * (float)(ADC_MAX)*PS_VOLTAGE_DIVIDER * 100.0f) / VS_uC);

  //if pressure is more than 0.5 [bar] below or above setpoint, OUTPUT will be set to min or max respectively
  myPID.setOutputRange(outputMin, outputMax);
  myPID.setBangBang (NO_BANG_BANG, NO_BANG_BANG);
  //set PID update interval to 0.1ms
  myPID.setTimeStep(0.001);
  pwm(FP_PWM_OUT,PWM_FREQUENCY,(uint16_t)(output));
/*
  // PWM 12KHZ
  // Set GCLK 4 to 48 MHz // REG_GCLK_GENDIV
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(2) |  // Divide the 48MHz clock source by divisor 2: 48MHz/2=24MHz
                    GCLK_GENDIV_ID(4);    // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Configure GCLK // REG_GCLK_GENCTLL
  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);         // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Feed GCLK4 to TCC0 and TCC1 // REG_GCLK_CLKCTRL
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |        // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;  // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
  
  //Use "Normal PWM" 
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  // Wait for bus synchronization
  while (TCC0->SYNCBUSY.bit.WAVE) {};
  // Use "Normal PWM" 
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
  // Wait for bus synchronization
  while (TCC1->SYNCBUSY.bit.WAVE) {};
  // Configure PA04 (A1) to be output. 
  PORT->Group[PORTA].DIRSET.reg = PORT_PA04;
  PORT->Group[PORTA].OUTCLR.reg = PORT_PA04;

  // Enable the peripheral multiplexer for the pins.
  PORT->Group[0].PINCFG[4].reg |= PORT_PINCFG_PMUXEN;

  PORT->Group[PORTA].PMUX[2].reg = PORT_PMUX_PMUXE_E;

  // Dual slope PWM operation // REG_TCC0_WAVE
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |       // Reverse the output polarity on all TCC0 outputs
                   TCC_WAVE_WAVEGEN_DSBOTH;  // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE)
    ;  // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  //Set per frecuency // REG_TCC0_PER // this determines the frequency of the PWM operation:
  REG_TCC0_PER = 1000;  // Set the frequency of the PWM on TCC0 to 12kHz
  while (TCC0->SYNCBUSY.bit.PER)
    ;  // Wait for synchronization

  // Set the PWM signal 20% dc // REG_TCC0_CC2
  REG_TCC0_CC0 = 400;  // TCC0 CC3 - on A2
  while (TCC0->SYNCBUSY.bit.CC0)
    ;  // Wait for synchronization

  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs // REG_TCC0_CTRLA
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |  // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;           // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE)
    ;  // Wait for synchronization
  */
}

void loop() {
  // put your main code here, to run repeatedly:
  vBat = ((float)(analogRead(VBAT_SENSE_IN)) / (float)(ADC_MAX)) * VS_uC * VBAT_VOLTAGE_DIVIDER;
  //outputMax = (double)((VFP_MAX / (vBat / 1000.0f)) * (float)(REG_TCC0_PER));
  outputMax = (double)((VFP_MAX / (vBat / 1000.0f)) * (float)(OUTPUT_MAX));
  myPID.setOutputRange(outputMin, outputMax);
  pressure = analogRead(V_PS_MEASURE_IN) - offset;
  pressureBar = pressure * VS_uC / (ADC_MAX * PS_SENSIVITY_DOCUMENTATION * PS_VOLTAGE_DIVIDER * 100.0);
  
  fpEnable = analogRead(ECU_ENABLE_FP_IN);
  if (fpEnable <= FP_ENABLE_THRESHOLD) {
    //Serial.printf("Dentro\r\n");
    myPID.run();
  } else {
    //Serial.printf("Fuera\r\n");
    output = outputMax;
    myPID.reset();
  }
  
  if (output < OUTPUT_MIN)
    output = OUTPUT_MIN;
  else if (output > outputMax)
    output = outputMax;

  /*REG_TCC0_CC0 = pwmOut;
  while (TCC0->SYNCBUSY.bit.CC0)
    ;  // Wait for synchronization
  */
 pwm(FP_PWM_OUT,PWM_FREQUENCY,(uint16_t)(output));

  ledState = ((pressureBar >= (PRESSURE_SETPOINT - 0.1f)) && (pressureBar < (PRESSURE_SETPOINT + 0.1f))) ? LOW : HIGH;
  digitalWrite(LED_BUILTIN, ledState);

  Serial.printf("Pressure: %.2f [bar] | Error: %.2f \r\n", pressureBar, output);
}