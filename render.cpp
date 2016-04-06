#include <BeagleRT.h>
#include <Utilities.h>
#include <rtdk.h>
#include <math.h>
#include <NE10.h>
#include <WriteFile.h>
#include "drums.h"

 
ne10_fft_cpx_float32_t* timeDomainIn;
ne10_fft_cpx_float32_t* frequencyDomain;
ne10_fft_cfg_float32_t cfg;

extern float *gDrumSampleBuffers[NUMBER_OF_DRUMS];
extern int gDrumSampleBufferLengths[NUMBER_OF_DRUMS];


const int gFFTSize = 64;
int gReadPointer;

float weights[gFFTSize];
float totEnergy;

bool setup(BeagleRTContext *context, void *userData)
{
	timeDomainIn = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	frequencyDomain = (ne10_fft_cpx_float32_t*) NE10_MALLOC (gFFTSize * sizeof (ne10_fft_cpx_float32_t));
	cfg = ne10_fft_alloc_c2c_float32 (gFFTSize);

	gReadPointer = 0;
	totEnergy = 0;

	for(unsigned int i=0; i<gFFTSize; i++){
		weights[i] = pow(i,2);
	}

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
	for (unsigned int k = 0; k < gFFTSize; k++) {
		numerator += k * abs(ampSpectrum[k]);
		denominator += ampSpectrum[k];
	}

	return numerator / denominator;
}


void render(BeagleRTContext *context, void *userData)
{
	for(unsigned int n = 0; n < context->audioFrames; n++) {

		timeDomainIn[gReadPointer].r = (ne10_float32_t)(context->audioIn[n*context->audioChannels]);
		timeDomainIn[gReadPointer].i = 0;

		if(++gReadPointer >= gFFTSize){
			ne10_fft_c2c_1d_float32_neon(frequencyDomain, timeDomainIn, cfg->twiddles, cfg->factors, gFFTSize, 0);
			for(unsigned int bin=0; bin<gFFTSize; bin++){
				float mag = pow(frequencyDomain[bin].r,2)*pow(frequencyDomain[bin].i,2);
				totEnergy += weights[bin]*mag;
			}

			totEnergy /= (float)gFFTSize;

			if(totEnergy > 5.0)
				rt_printf("%f\n", totEnergy);

			gReadPointer = 0;
		}
		// for(unsigned int ch=0; ch<context->audioChannels; ch++)
		// 	context->audioOut[n*context->audioChannels+ch] = context->audioIn[n*context->audioChannels+ch];

	}
}

void cleanup(BeagleRTContext *context, void *userData){
	NE10_FREE(timeDomainIn);
	NE10_FREE(frequencyDomain);
	NE10_FREE(cfg);
}
