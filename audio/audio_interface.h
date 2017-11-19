#ifndef AUDIO_INTERFACE_H
#define AUDIO_INTERFACE_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

// sets up microphone on the base to prepare recording
void setup_mic_base();

// sets up speaker on the base to prepare output
void setup_speaker_base();

// sets up microphone on the robot to prepare recording
void setup_mic_robot();

// sets up speaker on the robot to prepare output
void setup_speaker_robot();

#endif /* AUDIO_INTERFACE_H */