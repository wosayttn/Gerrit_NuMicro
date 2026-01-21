#ifndef _WAV_SAMPLE_H_
#define _WAV_SAMPLE_H_

extern const unsigned long gWAV_SAMPLE_termination;
extern const unsigned long gWAV_SAMPLE_start;
extern const unsigned long gWAV_SAMPLE_finish;
extern const unsigned long gWAV_SAMPLE_length;

extern const unsigned char gWAV_SAMPLE[];

#define WAV_SAMPLE_TERMINATION 0x00000000
#define WAV_SAMPLE_START       0x00000000
#define WAV_SAMPLE_FINISH      0x0002F04E
#define WAV_SAMPLE_LENGTH      0x0002F04E

#endif /* _WAV_SAMPLE_H_ */
