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


const int gFFTSize = 128;
int gInputWritePointer;
int gOutputReadPointer;
float* gMagSpectrum;
bool gPlayDrum;
float gHanning[gFFTSize];
bool gPrevStatus = false;
bool gRecord;

std::vector<float> gFlatness;
std::vector<float> gFlatness2;

float totEnergy;
float dtw_matrix[gFFTSize+1][gFFTSize+1];


bool setup(BeagleRTContext *context, void *userData){
	timeDomainIn = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	frequencyDomain = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	cfg = ne10_fft_alloc_c2c_float32 (gFFTSize);

	gInputWritePointer = 0;
	gOutputReadPointer = -1;
	totEnergy = 0;
	gPlayDrum = false;

	for(unsigned int n=0; n<context->digitalFrames; n++){
		pinModeFrame(context, 0, P9_12, INPUT);
    }

	for (unsigned int i = 0; i < gFFTSize; i++){
		gHanning[i] = 0.5 - 0.5 * cos(2 * M_PI * i / (gFFTSize - 1));	
	}

	gMagSpectrum = new float[gFFTSize/2];
	return true;
}

float rms(float* samples, int numSamples){
	float rms = 0;
	for(unsigned int i=0; i<numSamples; i++){
		rms += samples[i]*samples[i];
	}
	rms /= numSamples;
	rms = sqrt(rms);

	return rms;
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

float findMin(float* minVals){
	float min = std::numeric_limits<float>::infinity();
	
	for(unsigned int i=0; i<3; i++){
		if(minVals[i] < min){
			min = minVals[i];
		}
	}
	
	return min;
}


float dtw(std::vector<float> v1, std::vector<float> v2){
	float cost = 0;
	std::vector<float> temp(v2.size()+1, 0.0);
	std::vector< std::vector<float> > dtw_matrix(v1.size()+1, temp);
	
	for(unsigned int i=0; i<v1.size(); i++){
		dtw_matrix.at(i).at(0) = std::numeric_limits<float>::infinity();
	}

	for(unsigned int i=0; i<v2.size(); i++){
		dtw_matrix.at(0).at(i) = std::numeric_limits<float>::infinity();
	}

	dtw_matrix.at(0).at(0) = 0.0;


	for(unsigned int i=0; i<v1.size(); i++){
		for(unsigned int j=0; j<v2.size(); j++){
			cost = fabs(v1.at(i)-v2.at(j));
			float min[3] = {dtw_matrix.at(i).at(j+1), dtw_matrix.at(i+1).at(j), dtw_matrix.at(i).at(j)};
			dtw_matrix.at(i+1).at(j+1) = cost + findMin(min);
		}
	}

	return dtw_matrix.at(v1.size()).at(v2.size());
}


//http://stackoverflow.com/questions/8424170/1d-linear-convolution-in-ansi-c-code
float xcorr(std::vector<float> v1, std::vector<float> v2){
	float out = 0;
	std::reverse(v2.begin(), v2.end());
	int outputSize = v1.size()+v2.size()-1;
	std::vector<float> y(outputSize);

	for (int n = 0; n < outputSize; n++){
		int kmin, kmax;

		y.at(n) = 0;

		kmin = (n >= v2.size() - 1) ? n - (v2.size() - 1) : 0;
		kmax = (n < v1.size() - 1) ? n : v1.size() - 1;

		for (int k = kmin; k <= kmax; k++){
			y.at(n) += v1.at(k) * v2.at(n - k);
		}
		out+=fabs(y.at(n));
	}
	return out;
}


void render(BeagleRTContext *context, void *userData){

	for(unsigned int n=0; n<context->digitalFrames; n++){
		int status=digitalReadFrame(context, 0, P9_12);
		
		if(status && !gPrevStatus) gRecord = !gRecord;

		gPrevStatus = status;
	}

	float out = 0;

	for(unsigned int n = 0; n < context->audioFrames; n++) {

		timeDomainIn[gInputWritePointer].r = (ne10_float32_t)(context->audioIn[n*context->audioChannels]);
		timeDomainIn[gInputWritePointer].i = 0;

		if(++gInputWritePointer >= gFFTSize){

			// apply hann window.
			for (int i = 0; i < gFFTSize; i++){
				timeDomainIn[i].r *= gHanning[i];
			}

			// compute FFT.
			ne10_fft_c2c_1d_float32_neon(frequencyDomain, timeDomainIn, cfg->twiddles, cfg->factors, gFFTSize, 0);


			for(unsigned int bin=0; bin<gFFTSize/2; bin++){
				// compute magnitude spectrum.
				float mag = powf(frequencyDomain[bin].r,2)+powf(frequencyDomain[bin].i,2);
				gMagSpectrum[bin] = sqrtf(mag);
				totEnergy += mag;
			}
			

			totEnergy /= (float)gFFTSize/2;

			if(totEnergy > 2.0){
				// compute features.
				float flat = flatness(gMagSpectrum);
				float cent = centroid(gMagSpectrum);
				float roll = rolloff(gMagSpectrum);
				if(gRecord){
					gFlatness.push_back(flat);
				}
				else{
					gFlatness2.push_back(flat);
					gPlayDrum = true;
				}
			}
			gInputWritePointer = 0;
		}

		if(gPlayDrum){
			gOutputReadPointer++;
			out = gDrumSampleBuffers[0][gOutputReadPointer];
		}

		if(gOutputReadPointer >= gDrumSampleBufferLengths[0]){
			gPlayDrum = false;
			gOutputReadPointer = -1;
			printf("%f\n", xcorr(gFlatness, gFlatness2));
			gFlatness2.clear();
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
