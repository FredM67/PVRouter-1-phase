/* February 2014
 * Tool to capture the raw V and I samples generated by the Atmega 328P processor
 * during one or more mains cycles.  The data is displayed on the Serial Monitor.
 *
 * Voltage samples are displayed as 'v'
 * Current samples via CT1 are displayed as '1'
 *
 * The display is more compact if not every set of samples is shown.  This aspect
 * can be changed at the second of the two lines of code which contain a '%' character.
 *
 * February 2021
 * In the original version, data samples were obtained using the analogRead() function.  Now,
 * they are obtained by the ADC being controlled by a hardware timer with a periodicity of 125 us,
 * hence a full set of 1 x V and 2 x I samples takes 375 us.  The same scheme for collecting
 * data samples is found in many of my Mk2 PV Router sketches.
 *
 * When used with an output stage that has zero-crossing detection, the signal at port D4 can
 * be used to activate a load for just a single half main cycle.  The behaviour of the output signal
 * from CT1 can then be studied in detail.
 *
 * The stream of raw data samples from any floating CT will always be distorted because the CT acts as
 * a High Pass Filter.  This effect is only noticeable when the current that is being measured changes,
 * such as when an electrical load is turned on or off.  This sketch includes additional software which
 * compensates for this effect.  Similar compensation software has been introduced to the varous
 * "fasterControl" sketches that now exist.
 *
 *      Robin Emley
 *      www.Mk2PVrouter.co.uk
 *      June 2021
 */

#include <Arduino.h>
#include <TimerOne.h>

// definition of enumerated types
enum polarities
{
  NEGATIVE,
  POSITIVE
};
enum loadStates
{
  LOAD_ON,
  LOAD_OFF
}; // the external trigger device is active low

#define ADC_TIMER_PERIOD 125 // uS (determines the sampling rate / amount of idle time)
#define MAINS_CYCLES_PER_SECOND 50

const byte outputForTrigger = 4; // active low

byte sensor_V = 0;
byte sensor_I1 = 1;
byte sensor_I2 = 3;

long cycleCount = 0;
int samplesRecorded = 0;
const int DCoffsetI1_nominal = 511; // nominal mid-point value of ADC @ x1 scale

long DCoffset_V_long; // <--- for LPF
long DCoffset_V_min;  // <--- for LPF
long DCoffset_V_max;  // <--- for LPF

// extra items for an LPF to improve the processing of data samples from CT1
long lpf_long = 512; // new LPF, for ofsetting the behaviour of CT1 as a HPF
//
// The next two constants determine the profile of the LPF.
// They are matched to the physical behaviour of the YHDC SCT-013-000 CT
// and the CT1 samples being 375 us apart
//
const float lpf_gain = 8; // <- setting this to 0 disables this extra processing
// const float lpf_gain = 0;  // <- setting this to 0 disables this extra processing
const float alpha = 0.002; //

// for interaction between the main processor and the ISRs
volatile boolean dataReady = false;
volatile int sample_I2;
volatile int sample_I1;
volatile int sample_V;

enum polarities polarityOfMostRecentVsample;
enum polarities polarityOfLastVsample;
boolean beyondStartUpPhase = false;

int lastSample_V;            // stored value from the previous loop (HP filter is for voltage samples only)
float lastFiltered_V;        //  voltage values after HP-filtering to remove the DC offset
byte polarityOfLastSample_V; // for zero-crossing detection

boolean recordingNow;
boolean recordingComplete;
byte cycleNumberBeingRecorded;
byte noOfCyclesToBeRecorded;

unsigned long recordingMayStartAt;
boolean firstLoop = true;
int settlingDelay = 5; // <<---  settling time (seconds) for HPF

char blankLine[82];
char newLine[82];
int storedSample_V[170];
int storedSample_I1_raw[170];
int storedSample_I1[170];
// int storedSample_I2[100];

void setup()
{

  pinMode(outputForTrigger, OUTPUT);
  digitalWrite(outputForTrigger, LOAD_OFF);

  Serial.begin(9600);
  Serial.println();
  Serial.println("-------------------------------------");
  Serial.println("Sketch ID:      RST_375us_dev.ino");
  //
  Serial.print("alpha = ");
  Serial.println(alpha, 4);
  Serial.print("lpf_gain = ");
  Serial.println(lpf_gain, 1);
  Serial.println();

  // initialise each character of the display line
  blankLine[0] = '|';
  blankLine[80] = '|';

  for (int i = 1; i < 80; i++)
  {
    blankLine[i] = ' ';
  }
  blankLine[40] = '.';

  // Define operating limits for the LP filter which identifies DC offset in the voltage
  // sample stream.  By limiting the output range, the filter always should start up
  // correctly.
  DCoffset_V_long = 512L * 256;              // nominal mid-point value of ADC @ x256 scale
  DCoffset_V_min = (long)(512L - 100) * 256; // mid-point of ADC minus a working margin
  DCoffset_V_max = (long)(512L + 100) * 256; // mid-point of ADC plus a working margin

  // Set up the ADC to be triggered by a hardware timer of fixed duration
  ADCSRA = (1 << ADPS0) + (1 << ADPS1) + (1 << ADPS2); // Set the ADC's clock to system clock / 128
  ADCSRA |= (1 << ADEN);                               // Enable ADC

  Timer1.initialize(ADC_TIMER_PERIOD); // set Timer1 interval
  Timer1.attachInterrupt(timerIsr);    // declare timerIsr() as interrupt service routine

  Serial.print(">>free RAM = ");
  Serial.println(freeRam()); // a useful value to keep an eye on
}

void timerIsr(void)
{
  static unsigned char sample_index = 0;
  static int sample_I2_raw;
  static int sample_I1_raw;

  switch (sample_index)
  {
  case 0:
    sample_V = ADC;           // store the ADC value (this one is for Voltage)
    ADMUX = 0x40 + sensor_I1; // set up the next conversion, which is for current at CT1
    ADCSRA |= (1 << ADSC);    // start the ADC
    sample_index++;           // increment the control flag
    sample_I1 = sample_I1_raw;
    sample_I2 = sample_I2_raw;
    dataReady = true; // all three ADC values can now be processed
    break;
  case 1:
    sample_I1_raw = ADC;      // store the ADC value (this one is for current at CT1)
    ADMUX = 0x40 + sensor_I2; // set up the next conversion, which is for current at CT2
    ADCSRA |= (1 << ADSC);    // start the ADC
    sample_index++;           // increment the control flag
    break;
  case 2:
    sample_I2_raw = ADC;     // store the ADC value (this one is for current at CT2)
    ADMUX = 0x40 + sensor_V; // set up the next conversion, which is for Voltage
    ADCSRA |= (1 << ADSC);   // start the ADC
    sample_index = 0;        // reset the control flag
    break;
  default:
    sample_index = 0; // to prevent lockup (should never get here)
  }
}

void loop()
{
  if (dataReady) // flag is set after every set of ADC conversions
  {
    dataReady = false;      // reset the flag
    allGeneralProcessing(); // executed once for each set of V&I samples
  }
}

/*  Allow the system to run for several seconds so that the filtered
 *  voltage waveform can settle down.  This info is needed for determining
 *  the start of each new mains cycle.  During this period, a countdown
 *  is displayed.
 *
 *  After the settling period has expired, raw samples taken during one or
 *  more complete mains cycles are stored in an array.  The capacity of the
 *  array needs to be sufficient for the number of sample pairs that may
 *  appear.
 *
 *  At the start of the following cycle, the data collected during the
 *  previous mains cycle(s) is sent to the Serial window.
 */
void allGeneralProcessing() // each iteration is for one set of data samples
{
  static long cumVdeltasThisCycle_long; // for the LPF which determines DC offset (voltage)
  static byte oneSecondTimer = 0;
  static byte fiveSecondTimer = 0;
  static int sampleSetsDuringThisHalfMainsCycle;
  //
  if (firstLoop)
  {
    unsigned long timeNow = millis();
    Serial.print("millis() now = ");
    Serial.println(timeNow);

    recordingMayStartAt = timeNow + (settlingDelay * 1000);
    Serial.print("recordingMayStartAt ");
    Serial.println(recordingMayStartAt);

    recordingNow = false;
    firstLoop = false;
    recordingComplete = false;
    noOfCyclesToBeRecorded = 3; // more array space may be needed if this value is >1 !!!
    cycleNumberBeingRecorded = 0;
    samplesRecorded = 0;
  }

  // remove DC offset from the raw voltage sample by subtracting the accurate value
  // as determined by a LP filter.
  long sample_VminusDC_long = ((long)sample_V << 8) - DCoffset_V_long;

  // determine the polarity of the latest voltage sample
  if (sample_VminusDC_long > 0)
  {
    polarityOfMostRecentVsample = POSITIVE;
  }
  else
  {
    polarityOfMostRecentVsample = NEGATIVE;
  }

  if (polarityOfMostRecentVsample == POSITIVE)
  {
    if (polarityOfLastVsample != POSITIVE)
    {
      // This is the start of a new mains cycle
      cycleCount++;
      sampleSetsDuringThisHalfMainsCycle = 0;

      if (recordingNow == true)
      {
        if (cycleNumberBeingRecorded >= noOfCyclesToBeRecorded)
        {
          Serial.print("No of cycles recorded = ");
          Serial.println(cycleNumberBeingRecorded);
          dispatch_recorded_raw_data();
          dispatch_recorded_data();
        }
        else
        {
          cycleNumberBeingRecorded++;
        }
      }

      else if ((cycleCount % MAINS_CYCLES_PER_SECOND) == 1)
      {
        unsigned long timeNow = millis();
        if (timeNow > recordingMayStartAt)
        {
          recordingNow = true;
          cycleNumberBeingRecorded++;
        }
        else
        {
          Serial.println((int)(recordingMayStartAt - timeNow) / 1000);
        }
      }
    } // end of specific processing for first +ve Vsample in each mains cycle

    // still processing samples where the voltage is POSITIVE ...
    // check to see whether the trigger device can now be reliably armed
    if ((sampleSetsDuringThisHalfMainsCycle == 3) && (cycleNumberBeingRecorded == 1))
    {
      digitalWrite(outputForTrigger, LOAD_ON); // triac will fire at the next ZC point
    }
  }    // end of specific processing of +ve cycles
  else // the polatity of this sample is negative
  {
    if (polarityOfLastVsample != NEGATIVE)
    {
      sampleSetsDuringThisHalfMainsCycle = 0;

      long previousOffset = DCoffset_V_long;
      DCoffset_V_long = previousOffset + (cumVdeltasThisCycle_long >> 12);
      cumVdeltasThisCycle_long = 0;

      if (DCoffset_V_long < DCoffset_V_min)
      {
        DCoffset_V_long = DCoffset_V_min;
      }
      else if (DCoffset_V_long > DCoffset_V_max)
      {
        DCoffset_V_long = DCoffset_V_max;
      }

    } // end of processing that is specific to the first Vsample in each -ve half cycle
    // still processing samples where the voltage is NEGATIVE ...
    // check to see whether the trigger device can now be reliably armed
    if ((sampleSetsDuringThisHalfMainsCycle == 3) && (cycleNumberBeingRecorded == 1))
    {
      digitalWrite(outputForTrigger, LOAD_OFF); // triac will release at the next ZC point
    }
  } // end of processing that is specific to samples where the voltage is negative
    //
  // processing for EVERY set of samples
  //
  // extra filtering to offset the HPF effect of CT1
  //
  // subtract the nominal DC offset so the data stream is based around zero, as is required
  // for the LPF, and left-shift for integer maths use.

  if (recordingNow)
    storedSample_I1_raw[samplesRecorded] = sample_I1;

  long sampleI1minusDC_long = ((long)(sample_I1 - DCoffsetI1_nominal)) << 8;

  long last_lpf_long = lpf_long;
  lpf_long = last_lpf_long + alpha * (sampleI1minusDC_long - last_lpf_long);
  sampleI1minusDC_long += (lpf_gain * lpf_long);

  sample_I1 = (sampleI1minusDC_long >> 8) + DCoffsetI1_nominal;
  //
  if (recordingNow == true)
  {
    storedSample_V[samplesRecorded] = sample_V;
    storedSample_I1[samplesRecorded] = sample_I1;
    //    storedSample_I2[samplesRecorded] = sample_I2;
    ++samplesRecorded;
  }

  sampleSetsDuringThisHalfMainsCycle++;
  cumVdeltasThisCycle_long += sample_VminusDC_long;    // for use with LP filter
  polarityOfLastVsample = polarityOfMostRecentVsample; // for identification of half cycle boundaries
} // end of allGeneralProcessing()

void dispatch_recorded_raw_data()
{
  // display raw samples via the Serial Monitor
  // ------------------------------------------

  Serial.println("Raw data:");

  for (uint16_t index = 0; index < samplesRecorded; ++index)
  {
    Serial.print("V: ");
    Serial.print(storedSample_V[index]);
    Serial.print(" - I1 raw: ");
    Serial.print(storedSample_I1_raw[index]);
    Serial.print(" - I1: ");
    Serial.println(storedSample_I1[index]);
  }
}

void dispatch_recorded_data()
{
  // display raw samples via the Serial Monitor
  // ------------------------------------------

  Serial.print("cycleCount ");
  Serial.print(cycleCount);
  Serial.print(",  samplesRecorded ");
  Serial.println(samplesRecorded);

  int V, I1;
  int min_V = 1023, min_I1 = 1023;
  int max_V = 0, max_I1 = 0;

  for (int index = 0; index < samplesRecorded; index++)
  {
    strcpy(newLine, blankLine);
    V = storedSample_V[index];
    I1 = storedSample_I1[index];
    //    I2 = storedSample_I2[index];

    if (V < min_V)
    {
      min_V = V;
    }
    if (V > max_V)
    {
      max_V = V;
    }
    if (I1 < min_I1)
    {
      min_I1 = I1;
    }
    if (I1 > max_I1)
    {
      max_I1 = I1;
    }
    //    if (I2 < min_I2){min_I2 = I2;}
    //    if (I2 > max_I2){max_I2 = I2;}

    newLine[map(V, 0, 1023, 0, 80)] = 'v';
    //    newLine[map(I1, 0, 1023, 0, 80)] = '1';

    int halfRange = 200;
    int lowerLimit = 512 - halfRange;
    int upperLimit = 512 + halfRange;
    if ((I1 > lowerLimit) && (I1 < upperLimit))
    {
      newLine[map(I1, lowerLimit, upperLimit, 0, 80)] = '1'; // <-- raw sample scale
    }

    //    newLine[map(I2, 0, 1023, 0, 80)] = '2';

    if ((index % 2) == 0) // change this to "% 1" for full resolution
    {
      Serial.println(newLine);
    }
  }

  Serial.print("min_V ");
  Serial.print(min_V);
  Serial.print(",  max_V ");
  Serial.println(max_V);
  Serial.print("min_I1 ");
  Serial.print(min_I1);
  Serial.print(",  max_I1 ");
  Serial.println(max_I1);

  Serial.println();

  // despatch raw samples via the Serial Monitor
  // -------------------------------------------
  /*
  Serial.println("Raw data from stored cycle: <Vsample>, <I1sample>, <I2sample>[cr]");
  Serial.print(samplesRecorded);
  Serial.println(", <<< No of sample sets");

  for (int index = 0; index < samplesRecorded; index++)
  {
    Serial.print (storedSample_V[index]);
    Serial.print(", ");
    Serial.println (storedSample_I1[index]);
//    Serial.print(", ");
//    Serial.println (storedSample_I2[index]);
  }
  */
  recordingNow = false;
  firstLoop = true;
  pause();
}

void pause()
{
  byte done = false;
  byte dummyByte;

  while (done != true)
  {
    if (Serial.available() > 0)
    {
      dummyByte = Serial.read(); // to 'consume' the incoming byte
      if (dummyByte == 'g')
        done++;
    }
  }
}

int freeRam()
{
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}