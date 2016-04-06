#include <iostream>
#include <cstdlib>
#include <libgen.h>
#include <signal.h>
#include <getopt.h>
#include <BeagleRT.h>
#include <sndfile.h>
#include "drums.h"


using namespace std;

float *gDrumSampleBuffers[NUMBER_OF_DRUMS];
int gDrumSampleBufferLengths[NUMBER_OF_DRUMS];

// Handle Ctrl-C by requesting that the audio rendering stop
void interrupt_handler(int var)
{
	gShouldStop = true;
}

// Print usage information
void usage(const char * processName)
{
	cerr << "Usage: " << processName << " [options]" << endl;

	BeagleRT_usage();

	cerr << "   --help [-h]:             Print this menu\n";
}

int initDrums() {
	/* Load drums from WAV files */
	SNDFILE *sndfile ;
	SF_INFO sfinfo ;
	char filename[64];

	for(int i = 0; i < NUMBER_OF_DRUMS; i++) {
		snprintf(filename, 64, "audio/drum%d.wav", i);
		rt_printf("file: %s\n", filename);

		if (!(sndfile = sf_open (filename, SFM_READ, &sfinfo))) {
			printf("Couldn't open file %s\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gDrumSampleBuffers[j]);
			return 1;
		}

		if (sfinfo.channels != 1) {
			printf("Error: %s is not a mono file\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gDrumSampleBuffers[j]);
			return 1;
		}

		gDrumSampleBufferLengths[i] = sfinfo.frames;
		gDrumSampleBuffers[i] = (float *)malloc(gDrumSampleBufferLengths[i] * sizeof(float));
		if(gDrumSampleBuffers[i] == NULL) {
			printf("Error: couldn't allocate buffer for %s\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gDrumSampleBuffers[j]);
			return 1;
		}

		int subformat = sfinfo.format & SF_FORMAT_SUBMASK;
		int readcount = sf_read_float(sndfile, gDrumSampleBuffers[i], gDrumSampleBufferLengths[i]);

		/* Pad with zeros in case we couldn't read whole file */
		for(int k = readcount; k < gDrumSampleBufferLengths[i]; k++)
			gDrumSampleBuffers[i][k] = 0;

		if (subformat == SF_FORMAT_FLOAT || subformat == SF_FORMAT_DOUBLE) {
			double	scale ;
			int 	m ;

			sf_command (sndfile, SFC_CALC_SIGNAL_MAX, &scale, sizeof (scale)) ;
			if (scale < 1e-10)
				scale = 1.0 ;
			else
				scale = 32700.0 / scale ;
			printf("Scale = %f\n", scale);

			for (m = 0; m < gDrumSampleBufferLengths[i]; m++)
				gDrumSampleBuffers[i][m] *= scale;
		}

		sf_close(sndfile);
	}

	return 0;
}

int main(int argc, char *argv[])
{
	BeagleRTInitSettings settings;	// Standard audio settings

	struct option customOptions[] =
	{
		{"help", 0, NULL, 'h'},
		{NULL, 0, NULL, 0}
	};

	// Set default settings
	BeagleRT_defaultSettings(&settings);

	// Parse command-line arguments
	while (1) {
		int c;
		if ((c = BeagleRT_getopt_long(argc, argv, "h", customOptions, &settings)) < 0)
				break;
		switch (c) {
		case 'h':
				usage(basename(argv[0]));
				exit(0);
		case '?':
		default:
				usage(basename(argv[0]));
				exit(1);
		}
	}

	if(initDrums()) {
    	printf("Unable to load drum sounds. Check that you have all the WAV files!\n");
    	return -1;
    }

	// Initialise the PRU audio device
	if(BeagleRT_initAudio(&settings, 0) != 0) {
		cout << "Error: unable to initialise audio" << endl;
		return -1;
	}

	// Start the audio device running
	if(BeagleRT_startAudio()) {
		cout << "Error: unable to start real-time audio" << endl;
		return -1;
	}

	// Set up interrupt handler to catch Control-C and SIGTERM
	signal(SIGINT, interrupt_handler);
	signal(SIGTERM, interrupt_handler);

	// Run until told to stop
	while(!gShouldStop) {
		usleep(100000);
	}

	// Stop the audio device
	BeagleRT_stopAudio();

	// Clean up any resources allocated for audio
	BeagleRT_cleanupAudio();

	// All done!
	return 0;
}
