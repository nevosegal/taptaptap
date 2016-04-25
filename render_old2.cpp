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

 
ne10_fft_cpx_float32_t* timeDomainIn;
ne10_fft_cpx_float32_t* frequencyDomain;
ne10_fft_cfg_float32_t cfg;

extern float *gDrumSampleBuffers[NUMBER_OF_DRUMS];
extern int gDrumSampleBufferLengths[NUMBER_OF_DRUMS];

const int NUMBER_OF_FEATURES = 3;


const int gFFTSize = 128;
int gInputWritePointer;
int gOutputReadPointerA, gOutputReadPointerB;
float* gMagSpectrum;
bool gPlayDrumA, gPlayDrumB;
float gHanning[gFFTSize];
bool gPrevStatusA = false;
bool gPrevStatusB = false;
bool gRecordA, gRecordB;
bool isEngaged = false;
float gSimA, gSimB;

std::vector< std::vector<float> > gRecordedFeaturesA(NUMBER_OF_FEATURES);
std::vector< std::vector<float> > gRecordedFeaturesB(NUMBER_OF_FEATURES);
std::vector< std::vector<float> > gRecordedFeaturesC(NUMBER_OF_FEATURES);
std::vector< std::vector<float> > gIncomingFeatures(NUMBER_OF_FEATURES);

std::vector<float> gRecordedMeanA(NUMBER_OF_FEATURES);
std::vector<float> gRecordedMeanB(NUMBER_OF_FEATURES);
std::vector<float> gRecordedMeanC(NUMBER_OF_FEATURES);
std::vector<float> gIncomingMean(NUMBER_OF_FEATURES);

float totEnergy;
float dtw_matrix[gFFTSize+1][gFFTSize+1];


bool setup(BeagleRTContext *context, void *userData){
	timeDomainIn = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	frequencyDomain = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	cfg = ne10_fft_alloc_c2c_float32 (gFFTSize);

	gInputWritePointer = 0;
	gOutputReadPointerA = -1;
	gOutputReadPointerB = -1;
	totEnergy = 0;
	gPlayDrumA = false;
	gPlayDrumB = false;

	for(unsigned int n=0; n<context->digitalFrames; n++){
		pinModeFrame(context, 0, P9_12, INPUT);
		pinModeFrame(context, 0, P9_14, INPUT);
    }

	for (unsigned int i = 0; i < gFFTSize; i++){
		gHanning[i] = 0.5 - 0.5 * cos(2 * M_PI * i / (gFFTSize - 1));	
	}

	gMagSpectrum = new float[gFFTSize/2];
	gSimA = 100;
	gSimB = 100;
	return true;
}

float centroid(float* ampSpectrum) {
	float numerator = 0;
	float denominator = 0;
	for (unsigned int k = 0; k < gFFTSize/2; k++) {
		numerator += ((float)k * fabs(ampSpectrum[k]));
		denominator += ampSpectrum[k];
	}
	return (numerator/denominator)/(gFFTSize/2);
}

float rolloff(float* ampSpectrum){
	float nyqBin = 44100.0/(2.0*((gFFTSize/2.0)-1));
	float ec = 0;
	for (int i = 0; i < (gFFTSize/2); i++) {
		ec += ampSpectrum[i];
	}

	float threshold = 0.99 * ec;
	int n = (gFFTSize/2) - 1;
	while (ec > threshold && n >= 0) {
		ec -= ampSpectrum[n];
		--n;
	}

	return ((n + 1) * nyqBin)/ 22050.0;
}

float flatness(float* ampSpectrum){
	float numerator = 0;
	float denominator = 0;
	for (int i = 0; i < gFFTSize/2; i++) {
		numerator += log(ampSpectrum[i]);
		denominator += ampSpectrum[i];
	}

	return exp(numerator / (gFFTSize/2))*(gFFTSize/2) / denominator;
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

float getSimilarityMeasure(std::vector< std::vector<float> > v1, std::vector< std::vector<float> > v2){
	float similarity = 0;
	for(unsigned int i=1; i<NUMBER_OF_FEATURES; i++){
		similarity += xcorr(v1.at(i), v2.at(i));
	}

	return similarity/(float)NUMBER_OF_FEATURES;
}


void render(BeagleRTContext *context, void *userData){
	for(unsigned int n=0; n<context->digitalFrames; n++){
		int statusA=digitalReadFrame(context, 0, P9_12);
		int statusB=digitalReadFrame(context, 0, P9_14);
		
		if(statusA && !gPrevStatusA) gRecordA = !gRecordA;
		if(statusB && !gPrevStatusB) gRecordB = !gRecordB;

		gPrevStatusA = statusA;
		gPrevStatusB = statusB;
	}

	float out = 0;

	for(unsigned int n = 0; n < context->audioFrames; n++) {

		// store input in time domain array bigger than the frame size in order to be able to compute FFT on a larger number of samples.
		timeDomainIn[gInputWritePointer].r = (ne10_float32_t)(context->audioIn[n*context->audioChannels]);
		timeDomainIn[gInputWritePointer].i = 0;

		// if we have enough data, compute FFT.
		if(++gInputWritePointer >= gFFTSize){

			// before the FFT - apply Hann window.
			for (int i = 0; i < gFFTSize; i++){
				timeDomainIn[i].r *= gHanning[i];
			}

			// compute FFT.
			ne10_fft_c2c_1d_float32_neon(frequencyDomain, timeDomainIn, cfg->twiddles, cfg->factors, gFFTSize, 0);

			for(unsigned int bin=0; bin<gFFTSize/2; bin++){
				// compute magnitude spectrum.
				float mag = powf(frequencyDomain[bin].r,2)+powf(frequencyDomain[bin].i,2);
				gMagSpectrum[bin] = sqrtf(mag);

				// compute energy of spectrum. The energy is used to find for onset detection.
				totEnergy += mag;
			}
			
			totEnergy /= (float)gFFTSize/2;

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

					//apply xcorrelation here.
					if(gIncomingFeatures.at(0).size()!=0 && gRecordedFeaturesA.at(0).size()!=0){
					// 	float d = 0;
					// 	for (int i = 0; i < NUMBER_OF_FEATURES; i++){
					// 		for (int j = 0; j < gRecordedFeaturesA.at(i).size(); j++){
					// 			gRecordedMeanA.at(i) += gRecordedFeaturesA.at(i).at(j);
					// 		}
					// 		gRecordedMeanA.at(i) /= gRecordedFeaturesA.at(i).size();
					// 	}

					// 	for (int i = 0; i < NUMBER_OF_FEATURES; i++){
					// 		for (int j = 0; j < gIncomingFeatures.at(i).size(); j++){
					// 			gIncomingMean.at(i) = gIncomingFeatures.at(i).at(j);
					// 		}
					// 		gIncomingMean.at(i) /= gIncomingFeatures.at(i).size();
					// 		d += fabs(gIncomingMean.at(i) - gRecordedMeanA.at(i));
					// 	}
					// 	d /= (float)NUMBER_OF_FEATURES;
					// 	gSimA = d;
					// }
						gSimA = getSimilarityMeasure(gRecordedFeaturesA, gIncomingFeatures);
						printf("similarity to A: %f\n", gSimA);
					}
					if(gIncomingFeatures.at(0).size()!=0 && gRecordedFeaturesB.at(0).size()!=0){
					// 	float d = 0;
					// 	for (int i = 0; i < NUMBER_OF_FEATURES; i++){
					// 		for (int j = 0; j < gRecordedFeaturesB.at(i).size(); j++){
					// 			gRecordedMeanB.at(i) += gRecordedFeaturesB.at(i).at(j);
					// 		}
					// 		gRecordedMeanB.at(i) /= gRecordedFeaturesB.at(i).size();
					// 	}

					// 	for (int i = 0; i < NUMBER_OF_FEATURES; i++){
					// 		for (int j = 0; j < gIncomingFeatures.at(i).size(); j++){
					// 			gIncomingMean.at(i) = gIncomingFeatures.at(i).at(j);
					// 		}
					// 		gIncomingMean.at(i) /= gIncomingFeatures.at(i).size();

					// 		d += fabs(gIncomingMean.at(i) - gRecordedMeanB.at(i));
					// 	}
					// 	d /= (float)NUMBER_OF_FEATURES;
						gSimB = getSimilarityMeasure(gRecordedFeaturesB, gIncomingFeatures);
						printf("similarity to B: %f\n", gSimB);
					}
					if(gSimA != 100 || gSimB != 100){
						if(gSimA <= gSimB){
							gPlayDrumA = true;
						}
						else{
							gPlayDrumB = true;
						}
					}
				}
				isEngaged = false;
			}
			gInputWritePointer = 0;
		}

		if(gPlayDrumA){
			gOutputReadPointerA++;
			out = gDrumSampleBuffers[0][gOutputReadPointerA];
		}
		if(gPlayDrumB){
			gOutputReadPointerB++;
			out = gDrumSampleBuffers[1][gOutputReadPointerB];
		}
		if(gOutputReadPointerA >= gDrumSampleBufferLengths[0]){
			gPlayDrumA = false;
			gOutputReadPointerA = -1;

			for(unsigned int i=0; i<NUMBER_OF_FEATURES; i++){
				gIncomingFeatures.at(i).clear();
			}
		}
		if(gOutputReadPointerB >= gDrumSampleBufferLengths[1]){
			gPlayDrumB = false;
			gOutputReadPointerB = -1;

			for(unsigned int i=0; i<NUMBER_OF_FEATURES; i++){
				gIncomingFeatures.at(i).clear();
			}
		}

		for(unsigned int ch=0; ch<context->audioChannels; ch++)
			context->audioOut[n*context->audioChannels+ch] = out;
	}
}

void cleanup(BeagleRTContext *context, void *userData){
	NE10_FREE(timeDomainIn);
	NE10_FREE(frequencyDomain);
	NE10_FREE(cfg);
}
