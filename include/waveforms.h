// complementary file for waveforms.c
#ifndef WAVEFORMS_H
#define WAVEFORMS_H

#define WAVE_TABLE_SIZE 1024

void buildWaveTable(float waveTable[WAVE_TABLE_SIZE]);
void buildSquareWaveTable(float squareTable[WAVE_TABLE_SIZE]);
void buildTriangleWaveTable(float triangleTable[WAVE_TABLE_SIZE]);
void buildSawtoothWaveTable(float sawtoothTable[WAVE_TABLE_SIZE]);

#endif