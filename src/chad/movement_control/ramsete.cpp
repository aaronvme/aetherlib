#include "chad/movement_control/ramsete.hpp"

#include <cmath>

#include "chad/util/util.hpp"

namespace chad {
int closestPathIndex(const std::vector<Pose>& path, const Pose& pose) {
    if (path.empty()) return -1;

    int bestIdx = 0;
    float bestDist = distanceToPoint(pose, path[0].x, path[0].y);

    for (size_t i = 1; i < path.size(); i++) {
        float d = distanceToPoint(pose, path[i].x, path[i].y);
        if (d < bestDist) {
            bestDist = d;
            bestIdx = static_cast<int>(i);
        }
    }
    return bestIdx;
}

int lookaheadIndex(const std::vector<Pose>& path, const Pose& pose,
                   float lookaheadDist) {
    if (path.empty()) return -1;
    int closest = closestPathIndex(path, pose);
    for (size_t i = static_cast<size_t>(closest); i < path.size(); i++) {
        if (distanceToPoint(pose, path[i].x, path[i].y) >= lookaheadDist) {
            return static_cast<int>(i);
        }
    }
    return static_cast<int>(path.size()) - 1;
}
}
