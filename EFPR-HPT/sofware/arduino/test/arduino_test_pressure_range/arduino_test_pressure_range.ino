#include <FIR.h>

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
#define ADC_MAX 4095                // 12 bits
#define VS_uC 3300.0f               // [mV]
#define VBAT_VOLTAGE_DIVIDER 4.94f  // [] -> (10.2k / 50.4k) = 1 / 4.94
#define FP_ENABLE_THRESHOLD 2000
#define PWM_FREQUENCY 100000.0f          // [Hz]
#define VS_DOCUMENTATION 5000.0f         // [mV]
#define PS_SENSIVITY_DOCUMENTATION 6.4f  // [mV/KPa]
#define PS_OFFSET 0.31f                  // [bar]
#define PS_VOLTAGE_DIVIDER 0.6f          // [] -> 3.3k / 5.5k = 0.6

// FUEL PUMP
#define VFP 5.0f  // [V]
#define OUTPUT_MIN 0
#define OUTPUT_MAX 1024  // [] -> 10 bits range

// FIR Low Pass Filter
/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 1000 Hz

* 0 Hz - 30 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 3.862326127235951 dB

* 50 Hz - 500 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.65454680997551 dB

*/
#define FIR_LP_COEFFS_NUMBER 57

// *****************
// *** VARIABLES ***
// *****************
static double offset, vBat;
double pressure[FIR_LP_COEFFS_NUMBER], pressureFiltered, pressureBar;
uint16_t output, fpEnable;
static double fir_lp_coeffs[FIR_LP_COEFFS_NUMBER] = {
  -0.006625935301879643,
  -0.0046263466113658025,
  -0.005985511353086233,
  -0.007380821787402303,
  -0.00873036796448379,
  -0.00993950700883047,
  -0.010896287757939917,
  -0.011482281236332718,
  -0.011587810884810515,
  -0.011101547216457086,
  -0.009910366222694235,
  -0.007953295634674642,
  -0.005157471110900287,
  -0.0015077591607003994,
  0.002991486627176334,
  0.008296553290521154,
  0.014325031893215599,
  0.020957137106996516,
  0.028040562259329046,
  0.03539202153105998,
  0.042803468083504215,
  0.05005837859922911,
  0.05692676448045463,
  0.06318998799854678,
  0.06863884571993356,
  0.07308751616149889,
  0.0763828896849263,
  0.07840786342744005,
  0.07909108371401496,
  0.07840786342744005,
  0.0763828896849263,
  0.07308751616149889,
  0.06863884571993356,
  0.06318998799854678,
  0.05692676448045463,
  0.05005837859922911,
  0.042803468083504215,
  0.03539202153105998,
  0.028040562259329046,
  0.020957137106996516,
  0.014325031893215599,
  0.008296553290521154,
  0.002991486627176334,
  -0.0015077591607003994,
  -0.005157471110900287,
  -0.007953295634674642,
  -0.009910366222694235,
  -0.011101547216457086,
  -0.011587810884810515,
  -0.011482281236332718,
  -0.010896287757939917,
  -0.00993950700883047,
  -0.00873036796448379,
  -0.007380821787402303,
  -0.005985511353086233,
  -0.0046263466113658025,
  -0.006625935301879643
};

// ***************
// *** OBJECTS ***
// ***************
FIR<double, FIR_LP_COEFFS_NUMBER> fir_lp;

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

  offset = (PS_OFFSET * PS_SENSIVITY_DOCUMENTATION * (float)(ADC_MAX)*PS_VOLTAGE_DIVIDER * 100.0f) / VS_uC;

  // FIR Low Pass filter
  fir_lp.setFilterCoeffs(fir_lp_coeffs);

  pwm(FP_PWM_OUT, PWM_FREQUENCY, (uint16_t)(output));
}

void loop() {
  // put your main code here, to run repeatedly:
  fpEnable = analogRead(ECU_ENABLE_FP_IN);
  if (fpEnable <= FP_ENABLE_THRESHOLD) {
    //Serial.printf("Dentro\r\n");
    vBat = ((float)(analogRead(VBAT_SENSE_IN)) / (float)(ADC_MAX)) * VS_uC * VBAT_VOLTAGE_DIVIDER;
    output = (uint16_t)((double)((VFP / (vBat / 1000.0f)) * (float)(OUTPUT_MAX)));

    for (uint8_t i = 0; i < FIR_LP_COEFFS_NUMBER; i++) {
      pressure[i] = analogRead(V_PS_MEASURE_IN) - offset;
    }
    pressureFiltered = fir_lp.processReading(pressure[0]);
    pressureBar = pressureFiltered * VS_uC / (ADC_MAX * PS_SENSIVITY_DOCUMENTATION * PS_VOLTAGE_DIVIDER * 100.0);

  } else {
    //Serial.printf("Fuera\r\n");
    output = OUTPUT_MIN;
  }

  pwm(FP_PWM_OUT, PWM_FREQUENCY, output);
  Serial.printf("Output: %d |Vout: %0.2fV | Pressure: %0.2fbar\r\n", output, VFP, pressureBar);
}
