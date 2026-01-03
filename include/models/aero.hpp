#pragma once

struct Aero {
    double drag_coefficient; // Cd
    double lift_coefficient; // Cl (negative for downforce)
    double frontal_area;     // m^2
};