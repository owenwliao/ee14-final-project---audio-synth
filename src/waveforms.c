// for precomputed lookup tables for sine, square, triangle, sawtooth for each pitch
#include "waveforms.h"
#include "ee14lib.h"
#include "stm32l432xx.h"
#include "dacOutput.h"
#include <stdio.h>
#include <math.h>

#define C4_SQUARE_DELAY 188
#define Db4_SQUARE_DELAY 178
#define D4_SQUARE_DELAY 168
#define Eb4_SQUARE_DELAY 158
#define E4_SQUARE_DELAY 149
#define F4_SQUARE_DELAY 141
#define Gb4_SQUARE_DELAY 133
#define G4_SQUARE_DELAY 125
#define Ab4_SQUARE_DELAY 118
#define A4_SQUARE_DELAY 111
#define Bb4_SQUARE_DELAY 105
#define B4_SQUARE_DELAY 100

#define C4_TRIANGLE_DELAY 124
#define Db4_TRIANGLE_DELAY