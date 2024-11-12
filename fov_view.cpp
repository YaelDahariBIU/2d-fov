#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <cmath>
#include <set>
#include <vector>
#include <iostream>
#include <algorithm>
#include <string>


const float FoV = 1.25663706143f; // 72° field of vision in radians
const float halfFoV = FoV / 2;
const float halfFoVCos = cos(halfFoV); // 0.809016994375
const float halfFoVSin = sin(halfFoV); // 0.587785252292
const float visionRadius = 500;
const float personRadius = 5;
const float epsilon = 0.075f;
const float epsilon_sq = epsilon * epsilon;
const float halfAuxRayTilt = 8.72664625995e-3f; // half degree in radians
const float halfAuxRayTiltCos = cos(halfAuxRayTilt); // 0.999961923064
const float halfAuxRayTiltSin = sin(halfAuxRayTilt); // 8.72653549837e-3
glm::vec3 wallColour(0.5f, 0.5f, 0.5f);  // Example wall color
glm::vec3 observerColour(0.0f, 1.0f, 0.0f);  // Example observer color
glm::vec3 targetColour(1.0f, 0.0f, 0.0f);  // Example target color
glm::vec3 targetSeenColour(1.5f, 0.0f, 1.5f);  // Example target color
// const std::string wallColour = "105, 105, 105";
// const std::string observerColour = "255, 128, 128";
// const std::string targetColour = "128, 128, 255";
// const std::string targetSeenColour = "255, 0, 255";

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
    FrontSemicircle = 0,
    Behind = 1,
    Outside = 2,
    Null = 3,
    Within = 4
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

std::pair<PointInSector, std::vector<glm::vec2>> lineSegArcXsect(const glm::vec2& lineStart, const glm::vec2& lineVec, const glm::vec2& centre, float radius_2, float len_2, const glm::vec2& midDir, const std::vector<glm::vec2>& fovEdges) {
    glm::vec2 delta = lineStart - centre;
    float b = glm::dot(lineVec, delta);
    float d_2 = len_2;
    float c = glm::length2(delta) - radius_2;
    float det = (b * b) - (d_2 * c);

    std::vector<glm::vec2> points;

    if (det > 0) {
        float det_sqrt = sqrt(det);
        float t1 = (-b + det_sqrt) / d_2;
        float t2 = (-b - det_sqrt) / d_2;

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
            return std::make_pair(PointInSector::Null, points);

        return std::make_pair(PointInSector::Within, points);
    }
    return std::make_pair(PointInSector::Null, points);
}


// end of part 1 (line 309)

enum class LineSegConfig {
    Disjoint,
    Parallel,
    Intersect
};


// Function to compute the intersection of two line segments
struct LineSegmentResult {
    LineSegConfig config = LineSegConfig::Disjoint;
    glm::vec2 point;
    float t;
};

// Function to check line segment intersection
LineSegmentResult lineSegLineSegXsect(
    const glm::vec2& line1Start, const glm::vec2& line1Vec,
    const glm::vec2& line2Start, const glm::vec2& line2Vec,
    bool shouldComputePoint = false, bool isLine1Ray = false) {

    LineSegmentResult result;
    result.config = LineSegConfig::Disjoint;

    glm::vec2 l1p = perp2d(line1Vec);
    float f = glm::dot(line2Vec, l1p);

    if (std::abs(f) <= epsilon) {
        result.config = LineSegConfig::Parallel;

        if (isLine1Ray && shouldComputePoint && isPointOnLine(line2Start, line1Start, line1Vec)) {
            float alpha = invLerp(line2Start, line2Vec, line1Start, glm::length2(line2Vec));
            if (alpha >= 0 && alpha <= 1) {
                result.t = 0;
                result.point = line1Start;
            } else if (alpha < 0) {
                result.point = line2Start;
                result.t = invLerp(line1Start, line1Vec, result.point, glm::length2(line1Vec));
            }
        }
    } else {
        glm::vec2 c = line1Start - line2Start;
        float e = glm::dot(c, l1p);

        if (((f > 0) && (e >= 0) && (e <= f)) || ((f < 0) && (e <= 0) && (e >= f))) {
            glm::vec2 l2p = perp2d(line2Vec);
            float d = glm::dot(c, l2p);

            if ((isLine1Ray && (((f > 0) && (d >= 0)) || ((f < 0) && (d <= 0)))) ||
                (((f > 0) && (d >= 0) && (d <= f)) || ((f < 0) && (d <= 0) && (d >= f)))) {
                result.config = LineSegConfig::Intersect;
                if (shouldComputePoint) {
                    float s = d / f;
                    result.t = s;
                    result.point = pointOnLine(line1Start, line1Vec, s);
                }
            }
        }
    }
    return result;
}

// Function to add angle points with auxiliary rays
void addAnglePointWithAux(const glm::vec2& point, const glm::vec2& prevEdge, const glm::vec2& nextEdge, const glm::vec2& sectorCentre, std::set<glm::vec2>& anglePoints) {
    size_t currentSize = anglePoints.size();
    anglePoints.insert(point);

    if (currentSize != anglePoints.size()) {
        glm::vec2 ray = point - sectorCentre;
        glm::vec2 aux1 = glm::rotate(ray, halfAuxRayTiltCos);
        glm::vec2 aux2 = glm::rotate(ray, -halfAuxRayTiltSin);
        aux1 += sectorCentre;
        aux2 += sectorCentre;

        glm::vec2 projAxis = perp2d(ray);

        if (nextEdge == glm::vec2() || nextEdge == prevEdge) {
            glm::vec2 lineVec = (point == prevEdge) ? prevEdge : -prevEdge;
            float p = dot(lineVec, projAxis);
            if (p <= 0) anglePoints.insert(aux1);
            if (p >= 0) anglePoints.insert(aux2);
        } else {
            float p1 = dot(prevEdge, projAxis);
            float p2 = dot(nextEdge, projAxis);
            if (p1 >= 0 && p2 <= 0) anglePoints.insert(aux1);
            else if (p1 <= 0 && p2 >= 0) anglePoints.insert(aux2);
        }
    }
}

// end of part 2 (line 453)

struct Edge {
    glm::vec2 start;   // Starting point of the edge
    glm::vec2 end;     // Ending point of the edge
    glm::vec2 vector;  // Direction vector from start to end
    float len_2;       // Squared length of the vector

    // Constructor
    Edge(const glm::vec2& p1, const glm::vec2& p2) 
        : start(p1), end(p2), vector(p2 - p1), len_2(glm::length2(p2 - p1)) {}
};




struct Sector {
    glm::vec2 centre;                // Center of the sector
    glm::vec2 midDir;                // Mid direction vector of the sector
    float radius_2;                  // Square of the radius of the sector
    float radius;                   // Radius of the sector
    std::vector<Edge> fovEdges;     // Field of view edges

    // Constructor
    Sector(const glm::vec2& center, const glm::vec2& direction, float radiusSquared, const std::vector<Edge>& edges)
        : centre(center), midDir(direction), radius_2(radiusSquared), fovEdges(edges) {}
};


void checkPolygon(const Polygon& polygon, const Sector& sector, std::set<glm::vec2>& anglePoints, std::vector<Edge>& blockingEdges) 
{
    int n = polygon.edges.size();
    Edge prevEdge = polygon.edges[n - 1];

    for (int i = 0; i < n; ++i) {
        Edge edge = polygon.edges[i];

        // Check if this edge intersects the sector's bounding circle
        if (lineSegCircleXsect(edge.start, edge.vector, sector.centre, sector.radius_2, edge.len_2)) {
            // Determine the relationship of the points to the sector

            // Patch I've added to the original code
            std::vector<glm::vec2> fovEdges;
            for (const auto& fovEdge : sector.fovEdges) {
                fovEdges.push_back(fovEdge.vector);
            }
            // End of patch

            PointInSector e1InSector = isPointInSector(edge.start, sector.centre, sector.midDir, sector.radius_2, fovEdges);
            PointInSector e2InSector = isPointInSector(edge.end, sector.centre, sector.midDir, sector.radius_2, fovEdges);

            // Early exit if both points are behind
            if (e1InSector == PointInSector::Behind && e2InSector == PointInSector::Behind) 
                continue;

            // Add both points to anglePoints if both are within the sector
            if (e1InSector == PointInSector::Within && e2InSector == PointInSector::Within) {
                addAnglePointWithAux(edge.start, prevEdge.start, edge.end, sector.centre, anglePoints);
                addAnglePointWithAux(edge.end, edge.start, (i + 1 < n) ? polygon.edges[i + 1].start : glm::vec2(), sector.centre, anglePoints);
                blockingEdges.push_back(edge);
            } else {
                // Handle cases where one or both points are outside the sector
                bool blocking = false;

                if (e1InSector == PointInSector::Within) {
                    addAnglePointWithAux(edge.start, prevEdge.start, edge.end, sector.centre, anglePoints);
                    blocking = true;
                }
                if (e2InSector == PointInSector::Within) {
                    addAnglePointWithAux(edge.end, edge.start, (i + 1 < n) ? polygon.edges[i + 1].start : glm::vec2(), sector.centre, anglePoints);
                    blocking = true;
                }

                // Check if the edge may intersect the sector's arc
                bool edgeMayIntersectArc = (e1InSector == PointInSector::Outside) || (e2InSector == PointInSector::Outside);
                bool testSegSegXsect = true;

                if (edgeMayIntersectArc) {

                    // Patch I've added to the original code
                    std::vector<glm::vec2> fovEdges;
                    for (const auto& fovEdge : sector.fovEdges) {
                        fovEdges.push_back(fovEdge.vector);
                    }
                    // End of patch

                    auto arcXsectResult = lineSegArcXsect(edge.start, edge.vector, sector.centre, sector.radius_2, edge.len_2, sector.midDir, fovEdges);

                    if (!arcXsectResult.second.empty() && arcXsectResult.first == PointInSector::Within) {
                        for (const auto& point : arcXsectResult.second) {
                            anglePoints.insert(point);
                        }
                        blocking = true;
                    }

                    testSegSegXsect = (arcXsectResult.first != PointInSector::Behind);
                }

                if (blocking) {
                    blockingEdges.push_back(edge);
                } else if (testSegSegXsect) {
                    for (const auto& sectorEdge : sector.fovEdges) {
                        if (lineSegLineSegXsect(edge.start, edge.vector, sectorEdge.start, sectorEdge.vector).config == LineSegConfig::Intersect) {
                            blockingEdges.push_back(edge);
                            break;
                        }
                    }
                }
            }
        }
        prevEdge = edge;
    }
}


std::vector<glm::vec2> makeRays(const Sector& sector, const std::vector<glm::vec2>& anglePoints) {
    std::vector<glm::vec2> rays;
    glm::vec2 ray = anglePoints[0] - sector.centre;
    rays.push_back(ray);

    for (size_t i = 1, j = 0, n = anglePoints.size(); i < n; ++i) {
        ray = anglePoints[i] - sector.centre;
        if (!areParallel(ray, rays[j])) {
            rays.push_back(ray);
            ++j;
        }
    }
    return rays;
}

struct Polygon {
    std::vector<glm::vec2> coords; // List of the polygon's vertices (in order) - might not be needed
    std::vector<Edge> edges; // List of edges that make up the polygon
    glm::vec3 colour;
    int stroke;
    bool fill;
};

struct Observer {
    glm::vec2 loc;
    glm::vec2 dir;
    glm::vec3 colour;
    Sector sector;
    FieldOfView fov;
    
};

struct FieldOfView {
    std::set<glm::vec2> anglePtSet;
    std::vector<glm::vec2> anglePoints;
    std::vector<Edge> blockingEdges;
    std::vector<glm::vec2> rays;
    std::vector<glm::vec2> hitPoints;
    std::vector<glm::vec2> ctrlPoints;
};

struct Target {
    glm::vec2 loc;
    glm::vec2 dir;
    glm::vec3 colour;
};

struct Scene {
    std::vector<Polygon> polygons;
    Observer observer;
    Target target;
};

void updateSector(Sector& sector, const Scene& scene) {
    sector.centre = scene.observer.loc;
    sector.midDir = scene.observer.dir;
    
    auto fovDirs = rotateDir(sector.midDir, halfFoVCos, halfFoVSin);
    fovDirs.first *= sector.radius;
    fovDirs.second *= sector.radius;
    
    glm::vec2 e0 = sector.centre + fovDirs.first;
    glm::vec2 e1 = sector.centre + fovDirs.second;
    
    sector.fovEdges[0].vector = fovDirs.first;
    sector.fovEdges[0].start = sector.centre;
    sector.fovEdges[0].end = e0;
    
    sector.fovEdges[1].vector = fovDirs.second;
    sector.fovEdges[1].start = sector.centre;
    sector.fovEdges[1].end = e1;
}

// end of part 3 (line 664)

// Sorting the points in a counter-clockwise direction:
void sortAngularPoints(std::vector<glm::vec2>& anglePoints, const glm::vec2& centre) {
    std::sort(anglePoints.begin(), anglePoints.end(), [&centre](const glm::vec2& a, const glm::vec2& b) {
        glm::vec2 aV = a - centre;
        glm::vec2 bV = b - centre;
        return cross2d(aV, bV) > 0;  // Sort counter-clockwise
    });
}


// Calculating the Quadratic Bézier curve control point:
glm::vec2 calcQuadBezCurveCtrlPoint(const glm::vec2& v1, const glm::vec2& v2, const glm::vec2& centre, float radius) {
    glm::vec2 ctrlPt = v1 + v2;    // Add the two vectors
    ctrlPt = glm::normalize(ctrlPt);  // Unit bisector mid ray

    // Calculate the cosine of the half-angle using the dot product
    float cosineHalfAngle = glm::dot(v1, ctrlPt);

    // Calculate the control point based on the given formula
    glm::vec2 result;
    result = centre + ctrlPt * (radius * (2 - cosineHalfAngle));
    
    return result;
}

// end of part 4 (line 695)

// Function to shoot rays
struct RayShootingResult {
    std::vector<glm::vec2> hitPoints;
    std::vector<glm::vec2> ctrlPoints;
};

RayShootingResult shootRays(const std::vector<glm::vec2>& rays, const std::vector<glm::vec2>& blockingEdges, const glm::vec2& centre, float radius) {
    size_t n = rays.size();
    std::vector<glm::vec2> hitPoints(n);
    std::vector<glm::vec2> ctrlPoints(n);

    glm::vec2 prevUnitRay(0.0f, 0.0f);
    bool prevPointOnArc = false;

    for (size_t i = 0; i < n; ++i) {
        glm::vec2 thisRay = rays[i];
        float len2 = glm::length2(thisRay);
        glm::vec2 unitRay = glm::normalize(thisRay);

        glm::vec2 hitPoint = hitPoints[i] = glm::vec2(0.0f, 0.0f);
        float t = -1.0f;
        glm::vec2 blocker;
        float hitDist2 = -1.0f;

        // Check for intersections with blocking edges
        for (size_t j = 0; j < blockingEdges.size(); ++j) {
            LineSegmentResult res = lineSegLineSegXsect(centre, thisRay, blockingEdges[j], blockingEdges[j], true, true);

            if (res.t >= 0.0f && (t == -1.0f || res.t < t)) {
                hitDist2 = glm::length2(res.point - centre);
                if (hitDist2 > epsilon_sq) {
                    t = res.t;
                    hitPoint = res.point;
                    blocker = blockingEdges[j];
                }
            }
        }

        bool pointOnArc = (t == -1.0f) || ((hitDist2 + epsilon_sq - radius * radius) >= 0);
        if (pointOnArc) {
            // If the ray hits the arc
            hitPoint = centre + unitRay * radius;

            if (prevPointOnArc) {
                bool needsArc = true;
                if (blocker != glm::vec2()) {
                    glm::vec2 connector = hitPoints[i - 1] - hitPoint;
                    needsArc = !areParallel(blocker, connector);
                }
                if (needsArc) {
                    ctrlPoints[i] = calcQuadBezCurveCtrlPoint(unitRay, prevUnitRay, centre, radius);
                }
            }

            prevUnitRay = unitRay;
        }

        prevPointOnArc = pointOnArc;
    }

    return { hitPoints, ctrlPoints };
}

// end of part 5 (line 818)

bool isSubjectVisible(const std::vector<Edge>& blockingEdges, const Sector& sector, const Scene& scene) {

    // Patch I've added to the original code
    std::vector<glm::vec2> fovEdges;
    for (const auto& fovEdge : sector.fovEdges) {
        fovEdges.push_back(fovEdge.vector);
    }
    // End of patch

    // Check if the target is within the sector using the isPointInSector function
    if (isPointInSector(scene.target.loc, sector.centre, sector.midDir, sector.radius_2, fovEdges) == PointInSector::Within) {
        glm::vec2 rayVec = scene.target.loc - sector.centre;
        
        // Loop through blocking edges to check for intersections
        for (const auto& edge : blockingEdges) {
            // Perform line segment intersection check with the ray from the sector's center
            LineSegmentResult res = lineSegLineSegXsect(sector.centre, rayVec, edge.start, edge.vector, true /* shouldComputePoint */);

            // Check if the ray intersects with the edge
            if (res.config == LineSegConfig::Intersect) {
                // If the intersection point is within the sector's radius, the subject is not visible
                if (glm::length2(res.point - sector.centre) > epsilon) {
                    return false;
                }
            }
        }

        // If no intersection was found, the subject is visible
        return true;
    }

    // If the target is not within the sector, it's not visible
    return false;
}

// end of part 6 (line 833)

void update(Scene& scene) {
    // Update the sector
    auto& sector = scene.observer.sector;
    updateSector(sector, scene);

    // Get field of view (fov) and angle point set
    auto& fov = scene.observer.fov;
    auto& anglePtSet = fov.anglePtSet;
    anglePtSet.clear();
    
    // Get the blocking edges for the field of view (fov)
    auto& blockingEdges = fov.blockingEdges;
    blockingEdges.clear();

    // Iterate over all polygons in the scene
    for (const auto& polygon : scene.polygons) {
        checkPolygon(polygon, sector, anglePtSet, blockingEdges);
    }

    // Prepare angle points for sorting (including sector edge endpoints)
    std::vector<glm::vec2> anglePoints = {
        sector.fovEdges[0].end
    };

    for (const auto& point : anglePtSet) {
        anglePoints.push_back(point);
    }

    anglePoints.push_back(sector.fovEdges[1].end);


    // Sort the angle points in counter-clockwise order
    sortAngularPoints(anglePoints, sector.centre);

    // Create rays from the angle points
    std::vector<glm::vec2> rays = makeRays(sector, anglePoints);

    // Patch I've added to the original code
    std::vector<glm::vec2> blockEdges;
    for (const auto& blockEdge : blockingEdges) {
        blockEdges.push_back(blockEdge.vector);
    }
    // End of patch

    // Shoot rays and collect the result
    RayShootingResult result = shootRays(rays, blockEdges, sector.centre, sector.radius);

    // Store the result in the fov
    fov.anglePoints = anglePoints;
    fov.rays = rays;
    fov.hitPoints = result.hitPoints;
    fov.ctrlPoints = result.ctrlPoints;

    // Determine if the target is visible based on blocking edges
    scene.target.colour = isSubjectVisible(blockingEdges, sector, scene) ? targetSeenColour : targetColour;
}

// end of part 7 (line 866)