#include <ArduinoEigenDense.h>

#include <EmbeddedMPC.h>
// #include "problemdata.hpp"
#include "problem.hpp"
// #include "osqp.h"
// #include "qdldl.h"
// #include "qdldl_types.h"

#define LED_PIN 13

void setup() {
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}