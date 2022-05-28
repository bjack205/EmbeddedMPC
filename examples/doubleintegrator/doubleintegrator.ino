#include <ArduinoEigen.h>

#include "osqp.h"
#include "qdldl.h"
#include "qdldl_types.h"

#include "problem.hpp"

#define LED_PIN 13
#define NCOLS 10
constexpr int kMaxNumNonzeros = 100;
constexpr int kQDLDLBufferSizeFloat = NCOLS + 1;
constexpr int kQDLDLBufferSizeInt = NCOLS + 1 + kMaxNumNonzeros + 3 * NCOLS;

QDLDL_float g_qdldl_buffer_float[kQDLDLBufferSizeFloat];
QDLDL_int g_qdldl_buffer_int[kQDLDLBufferSizeInt];

void setup() {
    pinMode(LED_PIN, OUTPUT);
    const int n = NCOLS;
    int offset = 0;

    QDLDL_int* Ap = g_qdldl_buffer_int;
    offset += n + 1;

    QDLDL_int* Ai = g_qdldl_buffer_int + offset; 
    offset += kMaxNumNonzeros;

    QDLDL_int* work = g_qdldl_buffer_int + offset;
    offset += n;

    QDLDL_int* Lnz = g_qdldl_buffer_int + offset;
    offset += n;

    QDLDL_int* etree = g_qdldl_buffer_int + offset;
    offset += n;
    QDLDL_etree(n, Ap, Ai, work, Lnz, etree);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}