#ifndef TRANSLATE_H
#define TRANSLATE_H

#include <vector>
#include <cmath>
#include <cstdint>

// Thresholds and constants
const int TRA_TH[2] = {128, 255};  // Grayscale threshold for line tracking
const int TRA_AngTH = 30;          // Angle threshold for line tracking

// ROI settings (x, y, w, h, weight)
struct ROI {
    int x, y, w, h;
    float weight;
};

const std::vector<ROI> TRA_ROIS = {
    {0, 100, 160, 20, 0.7},
    {0, 50, 160, 20, 0.3},
    {0, 0, 160, 20, 0.1}
};

const ROI OBS_ROI = {30, 10, 100, 100};  // Obstacle avoidance ROI

// Communication data packets
const uint8_t Run[] = {0x24, 0x4F, 0x4D, 0x56, 0x31, 0x26, 0x23};   // $OMV1&#  Run
const uint8_t Left[] = {0x24, 0x4F, 0x4D, 0x56, 0x32, 0x26, 0x23};  // $OMV2&#  Left
const uint8_t Right[] = {0x24, 0x4F, 0x4D, 0x56, 0x33, 0x26, 0x23}; // $OMV3&#  Right
const uint8_t Stop[] = {0x24, 0x4F, 0x4D, 0x56, 0x34, 0x26, 0x23};  // $OMV4&#  Stop

// Function to get the index of the largest blob
int Get_MaxIndex(const std::vector<Blob>& blobs) {
    int maxb_index = 0;
    int max_pixels = 0;
    for (size_t i = 0; i < blobs.size(); ++i) {
        if (blobs[i].pixels() > max_pixels) {
            max_pixels = blobs[i].pixels();
            maxb_index = i;
        }
    }
    return maxb_index;
}

// Function to calculate the deflection angle
float Calculate_Deflection_Angle(float centroid_sum, float weight_sum) {
    float center_pos = centroid_sum / weight_sum;
    float deflection_angle = -std::atan((center_pos - 80) / 60);
    return std::degrees(deflection_angle);
}

#endif // TRANSLATE_H