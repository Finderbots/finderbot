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

// sends audio data in buf to speakers on the base
void base_playback(uint16_t* buf, int buf_frame);

// reads audio data and store it in buf on the base
void base_capture(uint16_t* buf, int buf_frame);

// sends audio data in buf to speakers on the robot
void robot_playback(uint16_t* buf, int buf_frame);

// reads audio data and store it in buf on the base
void robot_capture(uint16_t* buf, int buf_frame);

#endif /* AUDIO_INTERFACE_H */