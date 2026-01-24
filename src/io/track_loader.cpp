#include "io/track_loader.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

/**
 * @brief Loads track data from a CSV file.
 *
 * Expected CSV format (with header):
 *   length,curvature,grip,camber
 *   100.0,0.0,1.0,0.0
 *   50.0,0.02,1.0,0.01
 *   ...
 *
 * Where:
 *   - length: segment length in meters
 *   - curvature: 1/radius in 1/m (0 = straight, positive = left turn)
 *   - grip: surface grip coefficient (1.0 = normal dry asphalt)
 *   - camber: track banking angle in radians (positive = banked into turn)
 *
 * @param path Path to CSV file
 * @return Track structure with loaded segments
 * @throws std::runtime_error if file cannot be read or parsed
 */
Track load_track(const std::string& path) {
    Track track;
    track.total_length = 0.0;

    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open track file: " + path);
    }

    std::string line;

    // Skip header line
    if (!std::getline(file, line)) {
        throw std::runtime_error("Track file is empty: " + path);
    }

    // Parse data lines
    int line_number = 1;
    while (std::getline(file, line)) {
        ++line_number;

        // Skip empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        TrackSegment segment{};
        char comma;

        // Parse: length,curvature,grip,camber
        if (!(iss >> segment.length >> comma >>
                     segment.curvature >> comma >>
                     segment.grip >> comma >>
                     segment.camber)) {
            throw std::runtime_error(
                "Parse error in track file at line " + std::to_string(line_number) +
                ": " + line);
        }

        // Validate segment data
        if (segment.length <= 0.0) {
            throw std::runtime_error(
                "Invalid segment length at line " + std::to_string(line_number) +
                ": length must be positive");
        }

        if (segment.grip <= 0.0) {
            throw std::runtime_error(
                "Invalid grip coefficient at line " + std::to_string(line_number) +
                ": grip must be positive");
        }

        track.segments.push_back(segment);
        track.total_length += segment.length;
    }

    if (track.segments.empty()) {
        throw std::runtime_error("Track file contains no segments: " + path);
    }

    return track;
}
