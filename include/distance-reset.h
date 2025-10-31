#pragma once

#include "config.h"
#include "lib/data/data.h"
#include <algorithm>

constexpr double MM_TO_INCH = 25.4;

struct dist_results {
    double front_dist_sensor = -1;
    double side_dist = -1;
    std::atomic<bool> front_done{false};
    std::atomic<bool> side_done{false};
};


inline double sample_dist_sensor(pros::Distance& dist_sensor, const uint32_t sample_time) {
    // Dist sensor only provides confidence >200 mm ~= 8 inches. Otherwise, we
    // will not use confidence when filtering samples.
    bool use_confidence = true;

    // The number of samples we will be able to take.
    const size_t sample_count = sample_time / PROCESS_DELAY;
    std::vector<std::pair<int32_t, int32_t>> samples(sample_count);

    // Take samples
    for (size_t i = 0; i < sample_count; ++i) {
        const int32_t dist = dist_sensor.get_distance();
        samples[i] = {dist, dist_sensor.get_confidence()};
        // If the distance is less than 250 mm, don't use confidence. While the
        // actual threshold is 200 mm, we add 50 mm of padding for consistency.
        if (dist <= 250) use_confidence = false;

        pros::delay(PROCESS_DELAY);
    }

    if (!use_confidence) {
        // Sort by distance
        std::ranges::sort(samples,
                          [](const std::pair<int32_t, int32_t>& a, const std::pair<int32_t, int32_t>& b) {
                              return a.first < b.first;
                          });

        // Pick the median and return the distance converted to inches.
        const auto [median_dist, _] = samples[sample_count / 2];
        return median_dist / MM_TO_INCH;
    }

    // Calculate weighted average.
    int32_t total_dist = 0;
    int32_t total_weight = 0;

    for (const auto [dist, confidence] : samples) {
        // Weighing algorithm may change based on how trustworthy is confidence
        // is, if at all.
        const int32_t weight = confidence * confidence;

        total_dist += dist * weight;
        total_weight += weight;
    }

    // Convert weighted average to inches.
    return static_cast<double>(total_dist) / total_weight / MM_TO_INCH;
}

inline double find_dist(pros::Distance& dist_sensor, const Point& dist_sensor_offset, const double rad, const Dir dir,
                        const uint32_t sample_time) {
function_start:
    // Get the distance.
    const double dist_in = sample_dist_sensor(dist_sensor, sample_time);

    // Our first step is to find the point the distance sensor is hitting. To do
    // this, though, we need to consider the different directions it could be
    // facing.

    // First, is the direction horizontal?
    const bool horizontal = dir == Dir::Left || dir == Dir::Right;
    // Second, get the sign of the direction
    const int32_t sign = dir_to_sign(dir);

    // The point below represents the vector for the distance sensor's ray. All
    // distance sensors face increments of 90 degrees, so we need not consider
    // arbitrary rotations.
    const Point expansion = Point(horizontal * sign, !horizontal * sign) * dist_in;

    // Calculate and rotate the dist target properly. Nothing else needs to be
    // rotated as the robot's position is the anchor itself, and we no longer
    // use the distance sensor offset.
    const Point dist_target =
        Point::rotate(dist_sensor_offset, dist_sensor_offset + expansion, rad);

    // With this, we will find the length of the hypotenuse. Note how we use
    // 0, 0 as the robot's pose because one, everything is relative, and two,
    // we aren't actually allowed to use the robot's pose here since we are
    // correcting it.
    const double hyp = Point::dist(Point(0, 0), dist_target);

    // The next step is to find the length of the short leg. If the direction is
    // vertical, then we use the target's x position. If the direction is
    // horizontal, then we use the y position.
    const double short_leg = std::abs(horizontal ? dist_target.y : dist_target.x);

    // Using Pythagorean's theorem, we calculate the length of the long leg,
    // which is also our distance from the wall.
    return std::sqrt(hyp * hyp - short_leg * short_leg);
}

inline void match_load_reset(const uint32_t sample_time, const double heading_mod = 0, int8_t quadrant = -1) {
    const Dir dir = quadrant == 2 || quadrant == 4 ? Dir::Right : Dir::Left;

    printf("match load reset called\n");
    bool flips = false;
    if (quadrant == 1) {
        quadrant = 3;
        flips = true;
    }
    if (quadrant == 4) {
        quadrant = 2;
        flips = true;
    }
    double theta = std::fmod(chassis.getPose().theta + 90 - heading_mod + 360, 360);
    theta = std::fmod(theta + 180, 180);
    theta = std::fmod(theta + 90, 90);
    if (theta > 45 && theta < 90) theta -= 90;
    const double rad = theta * M_PI / 180;
    printf("theta=%f, rad=%f\n", theta, rad);

    // Pick the side distance sensor based on the wall we are next to. Distance
    // sensors don't have enough range to detect the other wall.
    pros::Distance& side_dist_sensor = dir == Dir::Right ? right_dist_sensor : left_dist_sensor;
    const Point& side_dist_sensor_offset = dir == Dir::Right ? right_dist_sensor_offset : left_dist_sensor_offset;

    // The general thought process is that the robot's position, the point the
    // distance sensor is hitting, and the point on the wall which forms a line
    // perpendicular to the wall with the robot's center is a right triangle.

    // The segment of the triangle along the wall will be denoted the short leg,
    // as in most cases it will be the shortest leg. The segment connecting the
    // wall to our robot will be denoted the long leg.

    // While we can't use the robot's position, we can set it to 0, 0 and do our
    // calculations around that. This is because we only need to find the
    // dimensions of the triangle.

    // Find the distance
    double front_wall_dist = -1;
    std::atomic<bool> ready(false);
    pros::Task([&] {
        front_wall_dist = find_dist(front_dist1_sensor, front_dist1_sensor_offset, rad, Dir::Front, sample_time);
        ready.store(true);
    });
    const double side_wall_dist = find_dist(side_dist_sensor, side_dist_sensor_offset, rad, dir, sample_time);

    while (!ready.load()) {
        pros::delay(10);
    }

    printf("fwd=%f, swf=%f\n", front_wall_dist, side_wall_dist);

    // Check if either find function call aborted.
    if (front_wall_dist == -1 || side_wall_dist == -1) {
        // Abort the distance reset.
        printf("Could not find distance, aborting distance reset.\n");
        return;
    }

    // We are almost done. The next step is to find the quadrant we are in. The
    // user can choose to provide the quadrant, but if its default value of -1
    // is still retained, then we know the user chose not to provide it.

    /*                             IMPORTANT NOTE                             */
    /* The coordinate plane here is viewed from the SIDE of the field, with the
     * RED park zone on the LEFT, and with the BLUE park zone on the RIGHT.   */

    // Failsafe for when both the custom and manually calculated quadrants are
    // invalid. When this is the case, the distance reset will abort.

    const lemlib::Pose& pose = chassis.getPose();
    printf("current pose: %f, %f, %f\n", pose.x, pose.y, pose.theta);

    bool quadrant_cooked = false;

    if (quadrant != -1) {
        printf("goto custom quadrant\n");
        goto custom_quadrant;
    }

manual_quadrant:
    printf("manual quadrant\n");

    quadrant = 0; // Set quadrant to zero in the case we are on an axis

    if (pose.x > 0) {
        if (pose.y > 0) quadrant = 1;
        if (pose.y < 0) quadrant = 4;
    }
    if (pose.x < 0) {
        if (pose.y > 0) quadrant = 2;
        if (pose.y < 0) quadrant = 3;
    }

custom_quadrant: // Goto label if the user decided to use a custom quadrant
    printf("custom quadrant\n");

    if (quadrant < 1 || quadrant > 4) {
        printf("quadrant is cooked\n");
        if (quadrant_cooked) {
            printf("Quadrant cannot be found, aborting distance reset.\n");
            return;
        }
        quadrant_cooked = true;
        goto manual_quadrant;
    }

    printf("quadrant:%d\n", quadrant);


    // The wall is supposed to be three tiles away from 0, 0, resulting in 72
    // inches as each tile is two feet. The field wall takes up a good bit of
    // the field, though, about two inches. Tune this number if needed.
    constexpr double wall_pos = 71;

    /*                             IMPORTANT NOTE                             */
    /* From the same viewing perspective of the quadrants, the front walls would
     * be the side walls and the side walls would be the front walls. The
     * current wall names are based on the perspective of the red and blue
     * alliances                                                              */

    // These are named x and y positions, but we initialize it with the wall
    // position for now. We will adjust them based on the calculated distances.
    double x_pos = (quadrant == 2 || quadrant == 3 ? -1 : 1) * wall_pos;
    double y_pos = (quadrant == 1 || quadrant == 2 ? 1 : -1) * wall_pos;

    // Multiplying distances by the opposite sign of the wall positions will
    // "push" us away from the wall.
    x_pos += front_wall_dist * lemlib::sgn(x_pos) * -1;
    y_pos += side_wall_dist * lemlib::sgn(y_pos) * -1;

    if (flips) x_pos *= -1;
    if (flips) y_pos *= -1;

    // Correct the chassis pose.
    chassis.setPose(x_pos, y_pos, chassis.getPose().theta);
}

inline void heading_reset(const int32_t sample_time, const float closest_correct_heading) {
    /*
     * Alright so the general thought process for this is that if we have two
     * rays of distance sensors pointing towards the wall, the distances will be
     * different since are heading isn't completely straight. Now let's say ray
     * A from our leftmost front facing distance sensor is length like 30 mm and
     * ray B from our rightmost front facing distance sensor is length like 35
     * mm.
     *
     * Call the point where ray A collides with the wall, and call this point
     * P1. Now, on ray B call a point P2 where it is 30 mm (length of ray A)
     * away from the location of the origin of ray B. So what we have now is a
     * segment P1P2 that is parallel to the line formed by the origins of rays
     * A and B. Also, notice how P1 is on the wall, so it forms the right
     * triangle P1, P2, and one more point which is the 90 degree one.
     *
     * Anyways, counting similar triangles by heading we find that angle between
     * segments P1P2 and P1 and P3, the third point of the triangle, is the
     * deviation from the intended heading of the robot.
     *
     * We'll adjust this with some math using the param `closest_correct_heading
     */

    // Find the distance
    double front_dist1 = -1;
    std::atomic<bool> ready(false);
    pros::Task([&] {
        front_dist1 = sample_dist_sensor(front_dist1_sensor, sample_time);
        ready.store(true);
    });
    const double front_dist2 = sample_dist_sensor(front_dist2_sensor, sample_time);

    const double short_leg = std::abs(front_dist1 - front_dist2);

    while (!ready.load()) {
        pros::delay(10);
    }

    const double long_leg = std::abs(front_dist1_sensor_offset.x - front_dist2_sensor_offset.x);

    const double heading_deviation = std::atan(short_leg / long_leg);

    double heading;
    if (front_dist1 >= front_dist2) {
        heading = closest_correct_heading - heading_deviation;
    } else {
        heading = closest_correct_heading + heading_deviation;
    }

    chassis.setPose(chassis.getPose().x, chassis.getPose().y, heading);
}
