#pragma once

#include <vector>

struct Engine {
    std::vector<double> rpm;
    std::vector<double> torque; // Nm
};

struct Gearbox {
    std::vector<double> ratios;
    double final_drive;
};

struct Drivetrain {
    Engine engine;
    Gearbox gearbox;
    double efficiency; // 0 to 1
};