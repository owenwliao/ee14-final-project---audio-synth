// for precomputed lookup tables for sine, square, triangle, sawtooth for each pitch
#include "waveforms.h"
#include "ee14lib.h"
#include "stm32l432xx.h"
#include <stdio.h>
#include <math.h>

#define WAVE_TABLE_SIZE 1024
#define PI 3.14159265358979323846

// arrays for waveforms
float sineTable[WAVE_TABLE_SIZE];
float squareTable[WAVE_TABLE_SIZE];
float triangleTable[WAVE_TABLE_SIZE];
float sawtoothTable[WAVE_TABLE_SIZE];

void buildWaveTable(float waveTable[WAVE_TABLE_SIZE]) {
    for (int i = 0; i < WAVE_TABLE_SIZE; i++) {
        double phase = (double)i / WAVE_TABLE_SIZE;
        double angle = 2 * PI * phase;
        waveTable[i] = (float)(sin(angle) * 2047.5 + 2047.5); // Scale to 0-4095
    }
}

void buildSquareWaveTable(float squareTable[WAVE_TABLE_SIZE]) {
    for (int i = 0; i < WAVE_TABLE_SIZE; i++) {
        if (i < WAVE_TABLE_SIZE / 2) {
            squareTable[i] = 4095; // High
        } else {
            squareTable[i] = 0; // Low
        }
    }
}

void buildTriangleWaveTable(float triangleTable[WAVE_TABLE_SIZE]) {
    for (int i = 0; i < WAVE_TABLE_SIZE; i++) {
        if (i < WAVE_TABLE_SIZE / 2) {
            triangleTable[i] = (float)(i * 4095.0 / (WAVE_TABLE_SIZE / 2)); // Ascending
        } else {
            triangleTable[i] = (float)((WAVE_TABLE_SIZE - i) * 4095.0 / (WAVE_TABLE_SIZE / 2)); // Descending
        }
    }
}

void buildSawtoothWaveTable(float sawtoothTable[WAVE_TABLE_SIZE]) {
    for (int i = 0; i < WAVE_TABLE_SIZE; i++) {
        sawtoothTable[i] = (float)(i * 4095.0 / WAVE_TABLE_SIZE); // Ascending
    }
}