/* Nevo Segal / 150643546
 * assignment2_drums
 *
 * Second assignment for ECS732 RTDSP, to create a sequencer-based
 * drum machine which plays sampled drum sounds in loops.
 *
 * This code runs on BeagleBone Black with the BeagleRT environment.
 *
 * 2016 
 * Becky Stewart
 *
 * 2015
 * Andrew McPherson and Victor Zappi
 * Queen Mary University of London
 */


#ifndef _DRUMS_H
#define _DRUMS_H

#define NUMBER_OF_DRUMS 1

/* Start playing a particular drum sound */
void startPlayingDrum(int drumIndex);

#endif /* _DRUMS_H */
