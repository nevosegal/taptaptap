#include <BeagleRT.h>
#include <Utilities.h>
#include <rtdk.h>
#include <math.h>
#include <NE10.h>
#include <stdio.h>
#include <WriteFile.h>
#include "drums.h"

 
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

float weights[gFFTSize/2];
float totEnergy;

bool setup(BeagleRTContext *context, void *userData){
	timeDomainIn = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	frequencyDomain = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	cfg = ne10_fft_alloc_c2c_float32 (gFFTSize);

	gInputWritePointer = 0;
	gOutputReadPointer = -1;
	totEnergy = 0;
	gPlayDrum = false;

	for(unsigned int i=0; i<gFFTSize/2; i++){
		weights[i] = pow(i,2);
		// rt_printf("%d\n", weights[i]);
	}

	for (unsigned int i = 0; i < gFFTSize; i++){
		gHanning[i] = 0.5 - 0.5 * cos(2 * M_PI * i / (gFFTSize - 1));	
	}

	gMagSpectrum = new float[gFFTSize];
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
	return numerator / denominator;
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

	return (n + 1) * nyqBin;
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


void render(BeagleRTContext *context, void *userData){

	float out = 0;
	for(unsigned int n = 0; n < context->audioFrames; n++) {

		timeDomainIn[gInputWritePointer].r = (ne10_float32_t)(context->audioIn[n*context->audioChannels]);
		timeDomainIn[gInputWritePointer].i = 0;

		if(++gInputWritePointer >= gFFTSize){

			ne10_fft_c2c_1d_float32_neon(frequencyDomain, timeDomainIn, cfg->twiddles, cfg->factors, gFFTSize, 0);

			for(unsigned int bin=0; bin<gFFTSize; bin++){
				float mag = powf(frequencyDomain[bin].r,2)+powf(frequencyDomain[bin].i,2);
				gMagSpectrum[bin] = sqrtf(mag);
				totEnergy += mag;
			}
			
			totEnergy /= (float)gFFTSize;

			if(totEnergy > 5.0){	
				gPlayDrum = true;
				float cent = flatness(gMagSpectrum);
				printf("%f\n", cent);
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
			rt_printf("*************************\n");
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
