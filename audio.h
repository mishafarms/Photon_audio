#ifndef __AUDIO_H__
#define __AUDIO_H__

/* DEFINE STATEMENTS */

#define ADC1_DR  (uint32_t) 0x4001204c

#define FFT_SIZE 256

#define SAMPLE_RATE_HZ 20000          // Sample rate of the audio in hertz.
#define SPECTRUM_MIN_DB 30.0          // Audio intensity (in decibels) that maps to low LED brightness.
#define SPECTRUM_MAX_DB 60.0          // Audio intensity (in decibels) that maps to high LED brightness.

#define NEO_PIXEL_PIN 3               // Output pin for neo pixels.
#define NEO_PIXEL_COUNT 25            // Number of neo pixels.  You should be able to increase this without
                                      // any other changes to the program.

#define BUFFERSIZE (2 * FFT_SIZE )    // we are kinda doing a double buffer scheme, we get 2 interrupts half and full 

void windowMean(float* magnitudes, int lowBin, int highBin, float* windowMean, float* otherMean);
int frequencyToBin(float frequency);
uint32_t pixelHSVtoRGBColor(float hue, float saturation, float value);
void spectrumSetup();
void spectrumLoop();

extern float frequencyWindow[];
extern float hues[];
extern float magnitudes[FFT_SIZE];

#endif
