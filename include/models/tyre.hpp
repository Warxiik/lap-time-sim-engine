#pragma once

struct TyreState {
    double temperature; // normalized [0..1]
    double wear;        // normalized [0..1]
};

struct TyreParams {
    double base_grip;      // base grip coefficient μ at optimal conditions
};