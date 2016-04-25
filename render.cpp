#include <BeagleRT.h>
#include <Utilities.h>
#include <rtdk.h>
#include <math.h>
#include <NE10.h>
#include <stdio.h>
#include <iostream>
#include <WriteFile.h>
#include "drums.h"
#include <limits>
#include <complex>
#include <algorithm>

const int NUMBER_OF_FEATURES = 3;

// set FFT size.
const int FFT_SIZE = 128;

// create variables for FFT.
ne10_fft_cpx_float32_t* timeDomainIn;
ne10_fft_cpx_float32_t* frequencyDomain;
ne10_fft_cfg_float32_t cfg;

// set external variables.
extern float *gDrumSampleBuffers[NUMBER_OF_DRUMS];
extern int gDrumSampleBufferLengths[NUMBER_OF_DRUMS];

// create variables.
int gInputWritePointer;
int gOutputReadPointerA, gOutputReadPointerB;
float* gMagSpectrum;
bool gPlayDrumA, gPlayDrumB;
float gHanning[FFT_SIZE];
bool gPrevStatusA = false;
bool gPrevStatusB = false;
bool gRecordA, gRecordB;
bool isEngaged = false;
float gSimA, gSimB;
float totEnergy;

// create feature matrices that will contain the feature vectors.
std::vector< std::vector<float> > gRecordedFeaturesA(NUMBER_OF_FEATURES);
std::vector< std::vector<float> > gRecordedFeaturesB(NUMBER_OF_FEATURES);
std::vector< std::vector<float> > gIncomingFeatures(NUMBER_OF_FEATURES);

bool setup(BeagleRTContext *context, void *userData){

	// allocate memory for the time domain and frequency domain signals.
	timeDomainIn = (ne10_fft_cpx_float32_t*) NE10_MALLOC (FFT_SIZE * sizeof (ne10_fft_cpx_float32_t));
	frequencyDomain = (ne10_fft_cpx_float32_t*) NE10_MALLOC (FFT_SIZE * sizeof (ne10_fft_cpx_float32_t));
	cfg = ne10_fft_alloc_c2c_float32 (FFT_SIZE);

	// instantiate read/write pointers.
	gInputWritePointer = 0;
	gOutputReadPointerA = -1;
	gOutputReadPointerB = -1;

	// instantiate the energy.
	totEnergy = 0;
	gPlayDrumA = false;
	gPlayDrumB = false;

	// initialise input pins.
	for(unsigned int n=0; n<context->digitalFrames; n++){
		pinModeFrame(context, 0, P9_12, INPUT);
		pinModeFrame(context, 0, P9_14, INPUT);
    }

    // instatiate the Hann window array.
	for (unsigned int i = 0; i < FFT_SIZE; i++){
		gHanning[i] = 0.5 - 0.5 * cos(2 * M_PI * i / (FFT_SIZE - 1));	
	}

	// instantiate magnitude spectrum array.
	gMagSpectrum = new float[FFT_SIZE/2];

	// set default values to similarity variables.
	gSimA = 100;
	gSimB = 100;

	return true;
}

float centroid(float* ampSpectrum) {
	float numerator = 0;
	float denominator = 0;
	for (unsigned int k = 0; k < FFT_SIZE/2; k++) {
		numerator += ((float)k * fabs(ampSpectrum[k]));
		denominator += ampSpectrum[k];
	}
	return (numerator/denominator)/(FFT_SIZE/2);
}

float rolloff(float* ampSpectrum){
	float nyqBin = 44100.0/(2.0*((FFT_SIZE/2.0)-1));
	float ec = 0;
	for (int i = 0; i < (FFT_SIZE/2); i++) {
		ec += ampSpectrum[i];
	}

	float threshold = 0.99 * ec;
	int n = (FFT_SIZE/2) - 1;
	while (ec > threshold && n >= 0) {
		ec -= ampSpectrum[n];
		--n;
	}

	return ((n + 1) * nyqBin)/ 22050.0;
}

float flatness(float* ampSpectrum){
	float numerator = 0;
	float denominator = 0;
	for (int i = 0; i < FFT_SIZE/2; i++) {
		numerator += log(ampSpectrum[i]);
		denominator += ampSpectrum[i];
	}

	return exp(numerator / (FFT_SIZE/2))*(FFT_SIZE/2) / denominator;
}

//http://stackoverflow.com/questions/8424170/1d-linear-convolution-in-ansi-c-code
float xcorr(std::vector<float> v1, std::vector<float> v2){
	float out = 0;
	// reverse input signal, thus transforming the convolution to cross-correlation.
	std::reverse(v2.begin(), v2.end());

	// instantiate output vector.
	int outputSize = v1.size()+v2.size()-1;
	std::vector<float> y(outputSize);

	for (int n = 0; n < outputSize; n++){
		int kmin, kmax;

		y.at(n) = 0;

		// setting upper and lower limits.
		kmin = (n >= v2.size() - 1) ? n - (v2.size() - 1) : 0;
		kmax = (n < v1.size() - 1) ? n : v1.size() - 1;

		// compute cross correlation.
		for (int k = kmin; k <= kmax; k++){
			y.at(n) += v1.at(k) * v2.at(n - k);
		}
		out+=fabs(y.at(n));
	}
	return out;
}

// accepts two feature matrices and computes the similarity between them.
float getSimilarityMeasure(std::vector< std::vector<float> > v1, std::vector< std::vector<float> > v2){
	float similarity = 0;
	for(unsigned int i=0; i<NUMBER_OF_FEATURES; i++){
		similarity += xcorr(v1.at(i), v2.at(i));
	}

	return similarity/(float)NUMBER_OF_FEATURES;
}


void render(BeagleRTContext *context, void *userData){

	// detect when a button is pressed and begin recording if so.
	for(unsigned int n=0; n<context->digitalFrames; n++){
		int statusA=digitalReadFrame(context, 0, P9_12);
		int statusB=digitalReadFrame(context, 0, P9_14);
		
		if(statusA && !gPrevStatusA) gRecordA = !gRecordA;
		if(statusB && !gPrevStatusB) gRecordB = !gRecordB;

		// store current status as the previous for the next loop.
		gPrevStatusA = statusA;
		gPrevStatusB = statusB;
	}

	float out = 0;

	for(unsigned int n = 0; n < context->audioFrames; n++) {

		// store input in time domain array bigger than the frame size in order to be able to compute FFT on a larger number of samples.
		timeDomainIn[gInputWritePointer].r = (ne10_float32_t)(context->audioIn[n*context->audioChannels]);
		timeDomainIn[gInputWritePointer].i = 0;

		// if we have enough data, compute FFT.
		if(++gInputWritePointer >= FFT_SIZE){

			// before the FFT - apply Hann window.
			for (int i = 0; i < FFT_SIZE; i++){
				timeDomainIn[i].r *= gHanning[i];
			}

			// compute FFT.
			ne10_fft_c2c_1d_float32_neon(frequencyDomain, timeDomainIn, cfg->twiddles, cfg->factors, FFT_SIZE, 0);

			for(unsigned int bin=0; bin<FFT_SIZE/2; bin++){
				// compute magnitude spectrum.
				float mag = powf(frequencyDomain[bin].r,2)+powf(frequencyDomain[bin].i,2);
				gMagSpectrum[bin] = sqrtf(mag);

				// compute energy of spectrum. The energy is used to find for onset detection.
				totEnergy += mag;
			}
			
			totEnergy /= (float)FFT_SIZE/2;

			// Setting a threshold for the energy. If it passes this threshold, the analysis begins.
			if(totEnergy > 5.0){
				isEngaged = true;

				// compute features: Spectral Flatness, Centroid and rolloff.
				float flat = flatness(gMagSpectrum);
				float cent = centroid(gMagSpectrum);
				float roll = rolloff(gMagSpectrum);

				// Check the state of the system.
				if(gRecordA){
					// store features in corresponding matrix.
					gRecordedFeaturesA.at(0).push_back(flat);
					gRecordedFeaturesA.at(1).push_back(cent);
					gRecordedFeaturesA.at(2).push_back(roll);
				}
				else if(gRecordB){
					// store features in corresponding matrix.
					gRecordedFeaturesB.at(0).push_back(flat);
					gRecordedFeaturesB.at(1).push_back(cent);
					gRecordedFeaturesB.at(2).push_back(roll);	
				}
				else{
					// store features in corresponding matrix.
					gIncomingFeatures.at(0).push_back(flat);
					gIncomingFeatures.at(1).push_back(cent);
					gIncomingFeatures.at(2).push_back(roll);
				}
			}
			else{
				if(isEngaged){

					// Check similarity of incoming signal to the first recorded signal A.
					if(gIncomingFeatures.at(0).size()!=0 && gRecordedFeaturesA.at(0).size()!=0){
						gSimA = getSimilarityMeasure(gRecordedFeaturesA, gIncomingFeatures);
						printf("similarity to A: %f\n", gSimA);
					}
					// Check similarity of incoming signal to the second recorded signal B.
					if(gIncomingFeatures.at(0).size()!=0 && gRecordedFeaturesB.at(0).size()!=0){
						gSimB = getSimilarityMeasure(gRecordedFeaturesB, gIncomingFeatures);
						printf("similarity to B: %f\n", gSimB);
					}

					// play the correct sample according to the distance.
					if(gSimA != 100 || gSimB != 100){
						if(gSimA <= gSimB){
							gPlayDrumA = true;
						}
						else{
							gPlayDrumB = true;
						}
					}
				}

				// disengage.
				isEngaged = false;
			}

			// reset the input pointer.
			gInputWritePointer = 0;
		}

		// if the first drum should be played, increment its pointer and get the sample.
		if(gPlayDrumA){
			gOutputReadPointerA++;
			out = gDrumSampleBuffers[0][gOutputReadPointerA];
		}
		// if the second drum should be played, increment its pointer and get the sample.
		if(gPlayDrumB){
			gOutputReadPointerB++;
			out = gDrumSampleBuffers[1][gOutputReadPointerB];
		}

		// if the drum sound reached its end, reset the pointer and the boolean and clear the incoming features matrix.
		if(gOutputReadPointerA >= gDrumSampleBufferLengths[0]){
			gPlayDrumA = false;
			gOutputReadPointerA = -1;

			for(unsigned int i=0; i<NUMBER_OF_FEATURES; i++){
				gIncomingFeatures.at(i).clear();
			}
		}
		// if the drum sound reached its end, reset the pointer and the boolean and clear the incoming features matrix.
		if(gOutputReadPointerB >= gDrumSampleBufferLengths[1]){
			gPlayDrumB = false;
			gOutputReadPointerB = -1;

			for(unsigned int i=0; i<NUMBER_OF_FEATURES; i++){
				gIncomingFeatures.at(i).clear();
			}
		}

		// store sample in output.
		for(unsigned int ch=0; ch<context->audioChannels; ch++)
			context->audioOut[n*context->audioChannels+ch] = out;
	}
}

void cleanup(BeagleRTContext *context, void *userData){
	// release memory.
	NE10_FREE(timeDomainIn);
	NE10_FREE(frequencyDomain);
	NE10_FREE(cfg);
}
