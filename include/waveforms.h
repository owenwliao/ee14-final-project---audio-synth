// complementary file for waveforms.c
#ifndef WAVEFORMS_H
#define WAVEFORMS_H

#define WAVE_TABLE_SIZE 1024

#define C4_DELAY 188
#define Db4_DELAY 178
#define D4_DELAY 168
#define Eb4_DELAY 158
#define E4_DELAY 149
#define F4_DELAY 141
#define Gb4_DELAY 133
#define G4_DELAY 125
#define Ab4_DELAY 118
#define A4_DELAY 111
#define Bb4_delay 105
#define B4_DELAY 100

void buildWaveTable(float waveTable[WAVE_TABLE_SIZE]);
void buildSquareWaveTable(float squareTable[WAVE_TABLE_SIZE]);
void buildTriangleWaveTable(float triangleTable[WAVE_TABLE_SIZE]);
void buildSawtoothWaveTable(float sawtoothTable[WAVE_TABLE_SIZE]);

#endif