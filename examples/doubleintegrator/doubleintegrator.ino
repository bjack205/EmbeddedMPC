#include <ArduinoEigenDense.h>
#include <memory>

#include <EmbeddedMPC.h>
#include "workspace.h"
#include "problem_data.h"
#include "problem.hpp"

#define LED_PIN 13

MPCProblem prob(nstates, ninputs, nhorizon);

void setup() {
    pinMode(LED_PIN, OUTPUT);
    while (!Serial) { delay(10); }
    Serial.println("Connected to Teensy!");
    std::unique_ptr<OSQPWorkspace> p_work(&workspace);

    // Initialize problem
    prob.SetDynamics(dynamics_Adata, dynamics_Bdata, dynamics_fdata);
    prob.SetCostTerminal(cost_Qfdata, cost_qfdata);
    prob.SetCostState(cost_Qdata, cost_qdata);
    prob.SetCostInput(cost_Rdata, cost_rdata);

    // Solve an OSQP Problem
    uint64_t tstart = micros();
    osqp_solve(&workspace);
    uint64_t tsolve_us = micros() - tstart;
    float tsolve_ms = tsolve_us / 1000.0;
    Serial.print("Solve took "); Serial.print(tsolve_ms, 4); Serial.println(" ms");
    Serial.print("Status:                "); Serial.println((&workspace)->info->status);
    Serial.print("Number of iterations:  "); Serial.println((int)((&workspace)->info->iter));
    Serial.print("Objective value:       "); Serial.println((&workspace)->info->obj_val, 4);
    Serial.print("Primal residual:       "); Serial.println((&workspace)->info->pri_res, 4);
    Serial.print("Dual residual:         "); Serial.println((&workspace)->info->dua_res, 4);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
}