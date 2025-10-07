//
// Created by Hanyu Zhang on 10/8/25.
//

#pragma once

struct Pose {
    float x_, y_, h;

    Pose(const float x, const float y, const float h): x_(x), y_(y), h(h) {}
};