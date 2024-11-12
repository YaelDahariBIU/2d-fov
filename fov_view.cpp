#include <glm/glm.hpp>
#include <cmath>
#include <vector>

const float FoV = 1.25663706143f; // 72° field of vision in radians
const float halfFoV = FoV / 2;
const float halfFoVCos = cos(halfFoV); // 0.809016994375
const float halfFoVSin = sin(halfFoV); // 0.587785252292
const float visionRadius = 500;
const float personRadius = 5;
const float epsilon = 0.075f;
const float halfAuxRayTilt = 8.72664625995e-3f; // half degree in radians
const float halfAuxRayTiltCos = cos(halfAuxRayTilt); // 0.999961923064
const float halfAuxRayTiltSin = sin(halfAuxRayTilt); // 8.72653549837e-3

bool isZero(const glm::vec2& v) {
    return (fabs(v.x) + fabs(v.y)) <= epsilon;
}

float cross2d(const glm::vec2& a, const glm::vec2& b) {
    return (a.x * b.y) - (a.y * b.x);
}

glm::vec2 perp2d(const glm::vec2& v, bool clockwise = false) {
    if (clockwise)
        return glm::vec2(v.y, -v.x);
    return glm::vec2(-v.y, v.x);
}

bool areParallel(const glm::vec2& a, const glm::vec2& b) {
    return fabs(cross2d(a, b)) <= epsilon;
}

float invLerp(const glm::vec2& lineStart, const glm::vec2& lineVec, const glm::vec2& point, float len_2) {
    glm::vec2 t = point - lineStart;
    return glm::dot(t, lineVec) / len_2;
}

bool isPointOnLine(const glm::vec2& pt, const glm::vec2& lineStart, const glm::vec2& lineVec) {
    glm::vec2 v = pt - lineStart;
    return areParallel(v, lineVec);
}

glm::vec2 pointOnLine(const glm::vec2& lineStart, const glm::vec2& lineVec, float t) {
    return lineStart + (t * lineVec);
}

std::pair<glm::vec2, glm::vec2> rotateDir(const glm::vec2& dir, float cosA, float sinA) {
    float xC = dir.x * cosA;
    float yC = dir.y * cosA;
    float xS = dir.x * sinA;
    float yS = dir.y * sinA;
    return {
        glm::vec2(xC - yS, xS + yC),
        glm::vec2(xC + yS, -xS + yC)
    };
}

void setDirection(glm::vec2& dir, const glm::vec2& loc, const glm::vec2& lookAt) {
    glm::vec2 newDir = lookAt - loc;
    if (!isZero(newDir)) {
        dir = glm::normalize(newDir);
    }
}

float pointSegShortestDistance(const glm::vec2& lineStart, const glm::vec2& lineVec, const glm::vec2& pt, float len_2) {
    glm::vec2 e1ToPt = pt - lineStart;
    float num = glm::dot(e1ToPt, lineVec);
    float s = num / len_2;
    s = (s <= 0) ? 0 : ((s >= 1) ? 1 : s);
    glm::vec2 perp = e1ToPt - (s * lineVec);
    return glm::length2(perp);
}

bool lineSegCircleXsect(const glm::vec2& lineStart, const glm::vec2& lineVec, const glm::vec2& centre, float radius_2, float len_2) {
    return pointSegShortestDistance(lineStart, lineVec, centre, len_2) < radius_2;
}

enum class PointInSector {
    FrontSemicircle,
    Behind,
    Outside,
    Within
};

PointInSector isPointInSector(const glm::vec2& pt, const glm::vec2& centre, const glm::vec2& midDir, float radius_2, const std::vector<glm::vec2>& fovEdges) {
    glm::vec2 v = pt - centre;
    float dot = glm::dot(v, midDir);
    if (dot <= 0)
        return PointInSector::Behind;
    if (glm::length2(v) - radius_2 > epsilon)
        return PointInSector::Outside;
    if ((cross2d(fovEdges[0], v) <= 0) && (cross2d(v, fovEdges[1]) <= 0))
        return PointInSector::Within;
    return PointInSector::FrontSemicircle;
}

std::optional<std::pair<PointInSector, std::vector<glm::vec2>>> lineSegArcXsect(const glm::vec2& lineStart, const glm::vec2& lineVec, const glm::vec2& centre, float radius_2, float len_2, const glm::vec2& midDir, const std::vector<glm::vec2>& fovEdges) {
    glm::vec2 delta = lineStart - centre;
    float b = glm::dot(lineVec, delta);
    float d_2 = len_2;
    float c = glm::length2(delta) - radius_2;
    float det = (b * b) - (d_2 * c);

    if (det > 0) {
        float det_sqrt = sqrt(det);
        float t1 = (-b + det_sqrt) / d_2;
        float t2 = (-b - det_sqrt) / d_2;
        std::vector<glm::vec2> points;

        if (t1 >= 0 && t1 <= 1) {
            glm::vec2 p1 = pointOnLine(lineStart, lineVec, t1);
            if (isPointInSector(p1, centre, midDir, radius_2, fovEdges) == PointInSector::Within)
                points.push_back(p1);
        }
        if (t2 >= 0 && t2 <= 1) {
            glm::vec2 p2 = pointOnLine(lineStart, lineVec, t2);
            if (isPointInSector(p2, centre, midDir, radius_2, fovEdges) == PointInSector::Within)
                points.push_back(p2);
        }

        if (points.empty())
            return std::nullopt;

        return std::make_pair(PointInSector::Within, points);
    }
    return std::nullopt;
}


// end of part 1


