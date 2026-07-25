//====================================================
// Heart Rate Detector - ESP32
//====================================================

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// ---------- LCD ----------
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------- Pins ----------
const int SENSOR_PIN = 35;
const int LED_PIN = 13;

// ---------- Sampling ----------
const unsigned long SAMPLE_PERIOD = 5;   // 5ms = 200 Hz

// ---------- Serial Plotter display rate ----------
// Detection still runs on every 5ms sample above - this only decides how
// often we SEND a line to Serial. At 200Hz, whatever fixed number of
// points your plotter displays covers a tiny sliver of real time, too
// short to see a full heartbeat. Printing every Nth sample instead spreads
// the same point-count over more real time without touching detection.
// e.g. PRINT_DECIMATION = 4 -> 50Hz print rate -> 4x wider visible window.
const int PRINT_DECIMATION = 10;
int printCounter = 0;

// ---------- Signal validity (clipping / motion / finger off) ----------
// Your amp chain has gain 10 * 10 = 100. A real pulse should sit somewhere
// comfortably inside the ADC range, not slammed against a rail. Motion
// artifacts and finger lift-off tend to push the output toward 0V or VCC.
// TUNE THESE using the serial plotter: watch the raw voltage with your
// finger still, then moving, and set these just outside the "still" range.
const float RAIL_LOW = 0.1;    // volts
const float RAIL_HIGH = 1.5;   // volts

// ---------- Filter ----------
float filtered = 0.0;
float baseline = 0.0;

const float alphaSignal = 0.20;
const float alphaBaseline = 0.005;

// ---------- Detection ----------
const float THRESHOLD = 0.05;      // volts above baseline
// Fixed floor: the absolute minimum time between beats we'll ever accept
// (hard caps max detectable rate; 250ms -> 240 bpm ceiling).
const unsigned long MIN_REFRACTORY = 250; // ms
const unsigned long MAX_REFRACTORY = 500; // ms

// Adaptive refractory: recalculated after every confirmed beat as roughly
// half the current average beat interval. A fixed 300ms window doesn't
// scale with your actual heart rate, so a secondary bump (e.g. a dicrotic
// notch exaggerated by waveform clipping) landing ~350-450ms after a real
// peak at a resting heart rate can sneak past it and get double-counted.
// Scaling the lockout to the current rate closes that gap.
unsigned long refractoryPeriod = 300; // ms, current value in use

// ---------- Beat-to-beat amplitude plausibility ----------
// Real pulses have fairly consistent peak height beat-to-beat. Motion
// artifacts usually don't. Reject peaks whose height is way outside the
// recent normal range instead of trusting anything that merely clears
// THRESHOLD.
const float MIN_AMPLITUDE_RATIO = 0.5;  // reject if peakHeight < 0.5x avg
const float MAX_AMPLITUDE_RATIO = 2.0;  // reject if peakHeight > 2.0x avg

enum State
{
  WAITING,
  TRACKING_PEAK,
  REFRACTORY_STATE
};

State state = WAITING;

float previousFiltered = 0.0;
float peakValue = 0.0;

// ---------- Timing ----------
unsigned long lastSampleTime = 0;
unsigned long lastBeatTime = 0;
unsigned long refractoryStart = 0;
unsigned long ledOffTime = 0;

const unsigned long SIGNAL_TIMEOUT = 5000; // ms

// ---------- BPM ----------
float bpm = 0;

const int NUM_INTERVALS = 10;
unsigned long intervals[NUM_INTERVALS];
int intervalIndex = 0;
int intervalCount = 0;

// ---------- Amplitude history (for artifact rejection) ----------
const int NUM_AMPLITUDES = 10;
float amplitudes[NUM_AMPLITUDES];
int amplitudeIndex = 0;
int amplitudeCount = 0;

void setup()
{
  lcd.init();
  lcd.backlight();
  
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  filtered = analogReadMilliVolts(SENSOR_PIN) / 1000.0;
  baseline = filtered;

  for (int i = 0; i < NUM_INTERVALS; i++)
    intervals[i] = 0;

  for (int i = 0; i < NUM_AMPLITUDES; i++)
    amplitudes[i] = 0;
}

void loop()
{
  unsigned long now = millis();

  //---------------- LED OFF ----------------
  if (now > ledOffTime)
    digitalWrite(LED_PIN, LOW);

  //---------------- SAMPLE ----------------
  if (now - lastSampleTime >= SAMPLE_PERIOD)
  {
    lastSampleTime = now;

    float voltage = analogReadMilliVolts(SENSOR_PIN) / 1000.0;

    //-------------------------------------------------
    // SIGNAL VALIDITY CHECK
    //-------------------------------------------------
    if (voltage <= RAIL_LOW || voltage >= RAIL_HIGH)
    {
      // Clipped / saturated - almost always motion or finger off the
      // sensor. Don't touch the filter or baseline with this sample
      // (that would corrupt them), and reset the detector so we don't
      // get stuck mid-peak waiting for a waveform that will never
      // legitimately resolve.
      state = WAITING;

      printCounter++;
      if (printCounter >= PRINT_DECIMATION)
      {
        printCounter = 0;

        Serial.print("Filtered:");
        Serial.print(filtered);
        Serial.print(",Threshold:");
        Serial.print(baseline + THRESHOLD);

        // To fix y axis scale
        // Serial.print(",y_low:");
        // Serial.print(RAIL_LOW);
        // Serial.print(",y_high:");
        // Serial.println(RAIL_HIGH);

        // BPM not plotted
        Serial.println(">>> BPM: 0");
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("BPM: 0.00");
      }

      return;
    }

    // Low-pass filter
    filtered = alphaSignal * voltage +
               (1.0 - alphaSignal) * filtered;

    // Slowly moving baseline
    baseline = alphaBaseline * filtered +
               (1.0 - alphaBaseline) * baseline;

    float threshold = baseline + THRESHOLD;

    //-------------------------------------------------
    // STATE MACHINE
    //-------------------------------------------------

    switch (state)
    {

      //-------------------------------------------------
      case WAITING:

        if (filtered > threshold)
        {
          peakValue = filtered;
          state = TRACKING_PEAK;
        }

        break;

      //-------------------------------------------------
      case TRACKING_PEAK:

        // Keep highest point
        if (filtered > peakValue)
        {
          peakValue = filtered;
        }

        // Peak reached when signal starts decreasing
        if (filtered < previousFiltered)
        {
          float peakHeight = peakValue - baseline;

          // Too small to be a real heartbeat peak - likely noise
          if (peakHeight < THRESHOLD)
          {
              state = WAITING;
              break;
          }

          //---------------------------------------------
          // Amplitude plausibility check
          //---------------------------------------------
          bool amplitudeOK = true;

          if (amplitudeCount > 0)
          {
            float ampSum = 0;
            for (int i = 0; i < amplitudeCount; i++)
              ampSum += amplitudes[i];

            float avgAmplitude = ampSum / amplitudeCount;

            float minAmp = avgAmplitude * MIN_AMPLITUDE_RATIO;
            float maxAmp = avgAmplitude * MAX_AMPLITUDE_RATIO;

            if (peakHeight < minAmp || peakHeight > maxAmp)
              amplitudeOK = false;
          }

          if (!amplitudeOK)
          {
            // Very likely a motion artifact, not a real pulse.
            // Lock out re-detection of this same distorted waveform by
            // entering refractory, but do NOT touch lastBeatTime, the
            // LED, or either averaging buffer - this beat is simply
            // discarded, not "dampened".
            refractoryStart = now;
            state = REFRACTORY_STATE;
            break;
          }

          //---------------------------------------------
          // Genuine beat - timing / BPM update
          //---------------------------------------------
          if (lastBeatTime != 0)
          {
            unsigned long interval = now - lastBeatTime;

            if (intervalCount > 0)
            {
                unsigned long sum = 0;

                for (int i = 0; i < intervalCount; i++)
                    sum += intervals[i];

                float avgInterval = (float)sum / intervalCount;

                float minInterval = avgInterval * 0.7;
                float maxInterval = avgInterval * 1.3;

                // Secondary guard: clamp any remaining timing oddities
                // (e.g. a genuine early/late beat) instead of rejecting
                // outright, now that amplitude-based artifacts are
                // already filtered out above.
                if (interval < minInterval)
                  interval = (unsigned long)minInterval;
                else if (interval > maxInterval)
                  interval = (unsigned long)maxInterval;
            }

            if (interval > MIN_REFRACTORY)
            {
              intervals[intervalIndex] = interval;

              intervalIndex++;

              if (intervalIndex >= NUM_INTERVALS)
                intervalIndex = 0;

              if (intervalCount < NUM_INTERVALS)
                intervalCount++;

              unsigned long sum = 0;

              for (int i = 0; i < intervalCount; i++)
                sum += intervals[i];

              float avgInterval = (float)sum / intervalCount;

              bpm = 60000.0 / avgInterval;

              // Update the adaptive refractory window for the next beat,
              // based on the rate we're seeing right now.
              float target = avgInterval * 0.7;

              if (target < MIN_REFRACTORY)
                target = MIN_REFRACTORY;
              else if (target > MAX_REFRACTORY)
                target = MAX_REFRACTORY;

              refractoryPeriod = (unsigned long)target;
            }
          }

          // Record amplitude only for beats we trusted enough to use
          amplitudes[amplitudeIndex] = peakHeight;

          amplitudeIndex++;

          if (amplitudeIndex >= NUM_AMPLITUDES)
            amplitudeIndex = 0;

          if (amplitudeCount < NUM_AMPLITUDES)
            amplitudeCount++;

          lastBeatTime = now;

          digitalWrite(LED_PIN, HIGH);
          ledOffTime = now + 40;

          refractoryStart = now;
          state = REFRACTORY_STATE;
        }

        // Signal fell below threshold before peaking
        else if (filtered < threshold)
        {
          state = WAITING;
        }

        break;

      //-------------------------------------------------
      case REFRACTORY_STATE:

        if (now - refractoryStart >= refractoryPeriod)
        {
          state = WAITING;
        }

        break;
    }

    previousFiltered = filtered;

    // set bpm to 0 if no beat is detected
    if (lastBeatTime != 0 && (now - lastBeatTime) > SIGNAL_TIMEOUT) // 
    {
      bpm = 0;
      intervalCount = 0;
      intervalIndex = 0;
      amplitudeCount = 0;
      amplitudeIndex = 0;
      lastBeatTime = 0;
      refractoryPeriod = 300;
    }

    //---------------- Serial Plotter ----------------

    printCounter++;
    if (printCounter >= PRINT_DECIMATION)
    {
      printCounter = 0;

      Serial.print("Filtered:");
      Serial.print(filtered);
      Serial.print(",Threshold:");
      Serial.print(threshold);

      // To fix y axis scale
      // Serial.print(",y_low:");
      // Serial.print(RAIL_LOW);
      // Serial.print(",y_high:");
      // Serial.println(RAIL_HIGH);

      // BPM not plotted
      Serial.print(">>> BPM: ");
      Serial.println(bpm);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("BPM: ");
      lcd.print(bpm);
    }
  }
}



