// Audio Spectrum Display modified by Nate Meyer to work on a LED matrix utilizing FastLED

// Original code is by Tony DiCola (tony@tonydicola.com)
// Original is part of the guide at http://learn.adafruit.com/fft-fun-with-fourier-transforms/


////HARDWARE & SOFTWARE///
//ARDUINO VERSION: 1.8.9
//FASTLED VERSION: 3.10
//LEDS: WS2812B
//MICROCONTROLLER: TEENSY 4.0
//MICROPHONE: NOYITO MAX9814 Electret MIC Microphone

#include <FastLED.h>
#include <LEDMatrix.h>
#include <arm_math.h>
#define ARM_MATH_CM4



const uint8_t kMatrixWidth = 23; 
const uint8_t kMatrixHeight = 23;

////////////////////////////////////Needed for the teensy 4.0/////////////////////////////////////////////


#define NUM_LEDS_PER_STRIP (kMatrixWidth * kMatrixHeight) // used for the teensy 4.0
#define NUM_STRIPS 1
#define DATA_PIN 7 // Or whatever pin you want that is included in those 3 sets
#define LED_TYPE    WS2812B


/////////////////////////////////////////////////////////////////////////////////////////////////////////

#define BRIGHTNESS  255 //overall brigthtness
#define MATRIX_TYPE    HORIZONTAL_ZIGZAG_MATRIX  // See top of LEDMatrix.h for matrix wiring types
cLEDMatrix<kMatrixWidth, kMatrixHeight, MATRIX_TYPE> leds;

#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
#define LAST_VISIBLE_LED (kMatrixWidth * kMatrixHeight)

////////////////////////////////////////////////////////////////////////////////
// CONIFIGURATION
// These values can be changed to alter the behavior of the spectrum display.
////////////////////////////////////////////////////////////////////////////////
int SAMPLE_RATE_HZ = 22050;            // Sample rate of the audio in hertz. Higher the sample rate the faster it updates
float SPECTRUM_MIN_DB = 30.0;          // Audio intensity (in decibels) that maps to low LED brightness.
float SPECTRUM_MAX_DB = 60.0;          // Audio intensity (in decibels) that maps to high LED brightness.
//30.0db  and 60.0db seem to have the best results
int LEDS_ENABLED = 1;                  // Control if the LED's should display the spectrum or not.  1 is true, 0 is false.
int huemap = 0;                        // used to convert number of leds to hue
const int FFT_SIZE = 256;              // Size of the FFT.  Realistically can only be at most 256
                                        // without running out of memory for buffers and other state.
int intensitymap = 0;                  //used to map sound intensity to leds
const int AUDIO_INPUT_PIN = 17;        // Input ADC pin for audio data.
const int ANALOG_READ_RESOLUTION = 10; // Bits of resolution for the ADC.
const int ANALOG_READ_AVERAGING = 16;  // Number of samples to average with each ADC reading.
// any other changes to the program.
const int MAX_CHARS = 65;              // Max size of the input command buffer

////////////////////////////////////////////////////////////////////////////////
// INTERNAL STATE
// These shouldn't be modified unless you know what you're doing.
////////////////////////////////////////////////////////////////////////////////

IntervalTimer samplingTimer;
float samples[FFT_SIZE * 2];
float magnitudes[FFT_SIZE];
int sampleCounter = 0;
char commandBuffer[MAX_CHARS];
float frequencyWindow[NUM_LEDS + 1];
float hues[NUM_LEDS];

void setup() {
  // Set up serial port.
  Serial.begin(38400);

  // Set up ADC and audio input.
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);
  FastLED.setBrightness( BRIGHTNESS );
  FastLED.addLeds<NUM_STRIPS, WS2812B, DATA_PIN, GRB>(leds[0], leds.Size());
  
  // Clear the input command buffer
  memset(commandBuffer, 0, sizeof(commandBuffer));

  // Initialize spectrum display
  spectrumSetup();

  // Begin sampling audio
  samplingBegin();
}

void loop() {
  // Calculate FFT if a full sample is available.
  if (samplingIsDone()) {
    // Run FFT on sample data.
    arm_cfft_radix4_instance_f32 fft_inst;
    arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
    arm_cfft_radix4_f32(&fft_inst, samples);
    // Calculate magnitude of complex numbers output by the FFT.
    arm_cmplx_mag_f32(samples, magnitudes, FFT_SIZE);

    if (LEDS_ENABLED == 1)
    {
    
      spectrumLoop();
    }

    // Restart audio sampling.
    samplingBegin();
  }

  // Parse any pending commands.
  parserLoop();
}


////////////////////////////////////////////////////////////////////////////////
// UTILITY FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

// Compute the average magnitude of a target frequency window vs. all other frequencies.
void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean) {
  *windowMean = 0;
  *otherMean = 0;
  // Notice the first magnitude bin is skipped because it represents the
  // average power of the signal.
  for (int i = 1; i < FFT_SIZE / 2; ++i) {
    if (i >= lowBin && i <= highBin) {
      *windowMean += magnitudes[i];
    }
    else {
      *otherMean += magnitudes[i];
    }
  }
  *windowMean /= (highBin - lowBin) + 1;
  *otherMean /= (FFT_SIZE / 2 - (highBin - lowBin));
}

// Convert a frequency to the appropriate FFT bin it will fall within.
int frequencyToBin(float frequency) {
  float binFrequency = float(SAMPLE_RATE_HZ) / float(FFT_SIZE);
  return int(frequency / binFrequency);
}


////////////////////////////////////////////////////////////////////////////////
// SPECTRUM DISPLAY FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

void spectrumSetup() {
  // Set the frequency window values by evenly dividing the possible frequency
  // spectrum across the number of neo pixels.
  float windowSize = (SAMPLE_RATE_HZ / 2.0) / float(23);
  for (int i = 0; i < kMatrixWidth + 1; ++i) {
    frequencyWindow[i] = i * windowSize;
  }
  // Evenly spread hues across all pixels.
  for (int i = 0; i < kMatrixWidth; ++i) {
    hues[i] = 360.0 * (float(i) / float(kMatrixWidth - 1));
  }
}

void spectrumLoop() {

  // Update each LED based on the intensity of the audio
  // in the associated frequency window.
  float intensity, otherMean;
  FastLED.clear();
  for (int i = 0; i < kMatrixWidth; ++i) {
    windowMean(magnitudes,
               frequencyToBin(frequencyWindow[i]),
               frequencyToBin(frequencyWindow[i + 1]),
               &intensity,
               &otherMean);
    // Convert intensity to decibels.
    intensity = 20.0 * log10(intensity);
    // Scale the intensity and clamp between 0 and 1.0.
    intensity -= SPECTRUM_MIN_DB;
    intensity = intensity < 0.0 ? 0.0 : intensity;
    intensity /= (SPECTRUM_MAX_DB - SPECTRUM_MIN_DB);
    intensity = intensity > 1.0 ? 1.0 : intensity;

    intensitymap = map(intensity, 0, 1, 0, 23);
    huemap = map(i, 0, kMatrixHeight, 0, 255);

    for (int led = 0; led < intensitymap; led++) {

      leds.DrawLine(i, 0, i, led, CHSV(huemap, 255, 255));

    }
  }
  FastLED.show(); // display this frame
}

////////////////////////////////////////////////////////////////////////////////
// SAMPLING FUNCTIONS
////////////////////////////////////////////////////////////////////////////////

void samplingCallback() {
  // Read from the ADC and store the sample data
  samples[sampleCounter] = (float32_t)analogRead(AUDIO_INPUT_PIN);
  // Complex FFT functions require a coefficient for the imaginary part of the input.
  // Since we only have real data, set this coefficient to zero.
  samples[sampleCounter + 1] = 0.0;
  // Update sample buffer position and stop after the buffer is filled
  sampleCounter += 2;
  if (sampleCounter >= FFT_SIZE * 2) {
    samplingTimer.end();
  }
}

void samplingBegin() {
  // Reset sample buffer position and start callback at necessary rate.
  sampleCounter = 0;
  samplingTimer.begin(samplingCallback, 1000000 / SAMPLE_RATE_HZ);
}

boolean samplingIsDone() {
  return sampleCounter >= FFT_SIZE * 2;
}


////////////////////////////////////////////////////////////////////////////////
// COMMAND PARSING FUNCTIONS
// These functions allow parsing simple commands input on the serial port.
// Commands allow reading and writing variables that control the device.
//
// All commands must end with a semicolon character.
//
// Example commands are:
// GET SAMPLE_RATE_HZ;
// - Get the sample rate of the device.
// SET SAMPLE_RATE_HZ 400;
// - Set the sample rate of the device to 400 hertz.
//
////////////////////////////////////////////////////////////////////////////////

void parserLoop() {
  // Process any incoming characters from the serial port
  while (Serial.available() > 0) {
    char c = Serial.read();
    // Add any characters that aren't the end of a command (semicolon) to the input buffer.
    if (c != ';') {
      c = toupper(c);
      strncat(commandBuffer, &c, 1);
    }
    else
    {
      // Parse the command because an end of command token was encountered.
      parseCommand(commandBuffer);
      // Clear the input buffer
      memset(commandBuffer, 0, sizeof(commandBuffer));
    }
  }
}

// Macro used in parseCommand function to simplify parsing get and set commands for a variable
#define GET_AND_SET(variableName) \
  else if (strcmp(command, "GET " #variableName) == 0) { \
    Serial.println(variableName); \
  } \
  else if (strstr(command, "SET " #variableName " ") != NULL) { \
    variableName = (typeof(variableName)) atof(command+(sizeof("SET " #variableName " ")-1)); \
  }

void parseCommand(char* command) {
  if (strcmp(command, "GET MAGNITUDES") == 0) {
    for (int i = 0; i < FFT_SIZE; ++i) {
      Serial.println(magnitudes[i]);
    }
  }
  else if (strcmp(command, "GET SAMPLES") == 0) {
    for (int i = 0; i < FFT_SIZE * 2; i += 2) {
      Serial.println(samples[i]);
    }
  }
  else if (strcmp(command, "GET FFT_SIZE") == 0) {
    Serial.println(FFT_SIZE);
  }
  GET_AND_SET(SAMPLE_RATE_HZ)
  GET_AND_SET(LEDS_ENABLED)
  GET_AND_SET(SPECTRUM_MIN_DB)
  GET_AND_SET(SPECTRUM_MAX_DB)

  // Update spectrum display values if sample rate was changed.
  if (strstr(command, "SET SAMPLE_RATE_HZ ") != NULL) {
    spectrumSetup();
  }

  // Turn off the LEDs if the state changed.
  if (LEDS_ENABLED == 0) {
    FastLED.clear();
    FastLED.show(); // display this frame

  }
}
