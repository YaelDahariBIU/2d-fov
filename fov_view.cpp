#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

#include <SFML/Graphics.hpp>

#include <cmath>
#include <set>
#include <memory>
#include <vector>
#include <iostream>
#include <algorithm>
#include <string>


void render();

// Custom comparator for glm::vec2
struct Vec2Comparator {
    bool operator()(const glm::vec2& a, const glm::vec2& b) const {
        if (a.x < b.x) return true;
        if (a.x > b.x) return false;
        return a.y < b.y;
    }
};

enum class LineSegConfig {
    Disjoint,
    Parallel,
    Intersect
};

// Function to compute the intersection of two line segments
struct LineSegmentResult {
    LineSegConfig config;
    glm::vec2 point;
    float t;
};

struct Edge {
    glm::vec2 start;   // Starting point of the edge
    glm::vec2 end;     // Ending point of the edge
    glm::vec2 vector;  // Direction vector from start to end
    float len_2;       // Squared length of the vector

    // Constructor
    Edge(const glm::vec2& p1, const glm::vec2& p2) 
        : start(p1), end(p2), vector(p2 - p1), len_2(glm::length2(p2 - p1)) {}

    Edge() {}
};

struct Sector {
    glm::vec2 centre;                // Center of the sector
    glm::vec2 midDir;                // Mid direction vector of the sector
    float radius_2;                  // Square of the radius of the sector
    float radius;                   // Radius of the sector
    std::vector<Edge> fovEdges;     // Field of view edges

    // Constructor
    Sector(const glm::vec2& center, const glm::vec2& direction, float radius, const std::vector<Edge>& edges)
        : centre(center), midDir(direction), radius(radius), radius_2(radius * radius), fovEdges(edges) {}

    Sector() {}
};

struct Polygon {
    std::vector<glm::vec2> coords; // List of the polygon's vertices (in order) - might not be needed
    std::vector<Edge> edges; // List of edges that make up the polygon
    sf::Color colour;
    int stroke;
    bool fill;

    Polygon(const std::vector<glm::vec2> &coordinates, const sf::Color c, int s, bool f)
        : coords(coordinates), colour(c), stroke(s), fill(f) {}
};

struct FieldOfView {
    std::set<glm::vec2, Vec2Comparator> anglePtSet;
    std::vector<glm::vec2> anglePoints;
    std::vector<Edge> blockingEdges;
    std::vector<glm::vec2> rays;
    std::vector<glm::vec2> hitPoints;
    std::vector<glm::vec2> ctrlPoints;
};

struct Observer {
    glm::vec2 loc;
    glm::vec2 dir;
    sf::Color colour;
    Sector sector;
    FieldOfView fov;
    
    Observer() {}
};

struct Target {
    glm::vec2 loc;
    glm::vec2 dir;
    sf::Color colour;
};

struct Scene {
    std::vector<Polygon> polygons;
    Observer observer;
    Target target;

    Scene() {}
};

struct RayShootingResult {
    std::vector<glm::vec2> hitPoints;
    std::vector<glm::vec2> ctrlPoints;
};

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
sf::Color wallColour = sf::Color::White;
sf::Color observerColour = sf::Color::Blue;
sf::Color targetColour = sf::Color::Red;
sf::Color targetSeenColour = sf::Color::Green;  
sf::RenderWindow window(sf::VideoMode(640, 480), "2D Canvas Example");
Scene scene;


bool debug = false;

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
void addAnglePointWithAux(const glm::vec2& point, const glm::vec2& prevEdge, const glm::vec2& nextEdge, const glm::vec2& sectorCentre, std::set<glm::vec2, Vec2Comparator>& anglePoints) {
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




void checkPolygon(const Polygon& polygon, const Sector& sector, std::set<glm::vec2, Vec2Comparator>& anglePoints, std::vector<Edge>& blockingEdges) 
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

void update() {
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

// jump to line 1074

// Function to draw a line
void drawLine(sf::RenderWindow &window, const glm::vec2 &e0, const glm::vec2 &e1, float thickness, const sf::Color &colour) {
    sf::VertexArray line(sf::Lines, 2);
    line[0].position = sf::Vector2f(e0.x, e0.y);
    line[0].color = colour;
    line[1].position = sf::Vector2f(e1.x, e1.y);
    line[1].color = colour;

    // SFML does not support line thickness directly, but you can handle it using rectangles or other methods.
    window.draw(line);
}

// Function to draw a polygon
void drawPolygon(sf::RenderWindow &window, const std::vector<glm::vec2> &coords, bool fill, float stroke, const sf::Color &colour) {
    // Create a convex shape for the polygon
    sf::ConvexShape polygon;
    polygon.setPointCount(coords.size());

    // Set the points for the polygon
    for (size_t i = 0; i < coords.size(); ++i) {
        polygon.setPoint(i, sf::Vector2f(coords[i].x, coords[i].y));
    }

    // Set the fill color if needed
    if (fill) {
        sf::Color fillColor = colour;
        fillColor.a = 38; // 15% opacity (0.15 * 255)
        polygon.setFillColor(fillColor);
    } else {
        polygon.setFillColor(sf::Color::Transparent);
    }

    // Set the outline color and thickness
    if (stroke > 0) {
        polygon.setOutlineThickness(stroke);
        polygon.setOutlineColor(colour);
    } else {
        polygon.setOutlineThickness(0);
    }

    // Draw the polygon
    window.draw(polygon);
}

// end of part 8 (line 1099)

// Function to draw a circle
void drawCircle(sf::RenderWindow &window, float x, float y, float radius, const sf::Color &colour) {
    // Create a circle shape
    sf::CircleShape circle(radius);
    circle.setPosition(x - radius, y - radius); // Position is top-left corner, so adjust for the center
    circle.setFillColor(sf::Color(colour.r, colour.g, colour.b, 128)); // 50% opacity (0.5 * 255)
    circle.setOutlineColor(colour);
    circle.setOutlineThickness(1.0f); // Adjust the thickness if needed

    // Draw the circle
    window.draw(circle);
}

// Function to draw a person
void drawPerson(sf::RenderWindow &window, const glm::vec2 &location, float personRadius, const sf::Color &colour) {
    drawCircle(window, location.x, location.y, personRadius, colour);
}

// end of part 9 (line 1114)


// Function to draw the Field of View
void drawFoV(sf::RenderWindow &window, const FieldOfView &fov, const Sector &sector) {
    // Create a vertex array for the shape
    sf::VertexArray shape(sf::TriangleFan);
    shape.append(sf::Vertex(sf::Vector2f(sector.centre.x, sector.centre.y), sf::Color(255, 140, 0, 89))); // 89 = 0.35 * 255

    for (size_t i = 0; i < fov.hitPoints.size(); ++i) {
        const glm::vec2 &p = fov.hitPoints[i];
        const glm::vec2 &cp = i < fov.ctrlPoints.size() ? fov.ctrlPoints[i] : glm::vec2();

        if (i < fov.ctrlPoints.size() && cp != glm::vec2(0, 0)) {
            // If there is a control point, use a quadratic curve (simplified approach)
            // Note: SFML does not have built-in support for quadratic curves, so you'd need to implement it manually
            // As a simple approximation, you can add the control point directly to the vertex array
            shape.append(sf::Vertex(sf::Vector2f(cp.x, cp.y), sf::Color(255, 140, 0, 89)));
        }
        // Add the hit point to the vertex array
        shape.append(sf::Vertex(sf::Vector2f(p.x, p.y), sf::Color(255, 140, 0, 89)));
    }

    // Draw the shape
    window.draw(shape);
}

void mainLoop() {
    update();
    render();
}

// Function to toggle the debug view
void toggleDebugView() {
    debug = !debug;
    // Refresh or re-render your main loop (adjust based on your application setup)
    // Example: Assuming you have a function called mainLoop() for rendering
    mainLoop();
}

// end of part 10 (line 1135)

// jump to line 868

void render() {
    // Clear canvas
    window.clear(sf::Color::White);

    // Draw polygons
    for (const auto &polygon : scene.polygons) {
        drawPolygon(window, polygon.coords, polygon.fill, polygon.stroke, polygon.colour);
    }

    // Draw personnel (observer and target)
    drawPerson(window, scene.target.loc, personRadius, scene.target.colour);
    drawPerson(window, scene.observer.loc, personRadius, scene.observer.colour);

    // Draw Field of View
    auto &fov = scene.observer.fov;
    auto &sector = scene.observer.sector;
    drawFoV(window, fov, sector);

    if (debug) {
        float radius = visionRadius;
        glm::vec2 dir = scene.observer.dir;
        glm::vec2 pos = scene.observer.loc;

        // Calculate end points by rotating observer.dir
        std::pair<glm::vec2, glm::vec2> rotDir = rotateDir(dir, halfFoVCos, halfFoVSin);
        glm::vec2 e1 = rotDir.first;
        glm::vec2 e2 = rotDir.second;
        glm::vec2 cp = pos + dir * (radius * (2 - halfFoVCos));

        // Calculate line perpendicular to observer.dir
        bool clockwise = true;
        glm::vec2 p1 = perp2d(dir, clockwise) * radius + pos;
        glm::vec2 p2 = -p1 + pos * (radius * 2);

        // Draw perpendicular and sector
        sf::Vertex line[] = {
            sf::Vertex(sf::Vector2f(p1.x, p1.y), sf::Color::Black),
            sf::Vertex(sf::Vector2f(p2.x, p2.y), sf::Color::Black)
        };
        window.draw(line, 2, sf::Lines);

        // Draw sector
        sf::VertexArray sectorShape(sf::LineStrip);
        sectorShape.append(sf::Vertex(sf::Vector2f(pos.x, pos.y), sf::Color::Black));
        sectorShape.append(sf::Vertex(sf::Vector2f(e1.x, e1.y), sf::Color::Black));
        sectorShape.append(sf::Vertex(sf::Vector2f(cp.x, cp.y), sf::Color::Black));
        sectorShape.append(sf::Vertex(sf::Vector2f(e2.x, e2.y), sf::Color::Black));
        window.draw(sectorShape);

        // Draw black angle points
        if (!fov.anglePoints.empty()) {
            for (const auto &p : fov.anglePoints) {
                drawCircle(window, p.x, p.y, 3, sf::Color::Black);
            }
        }

        // Draw black blocking edges
        for (const auto &edge : fov.blockingEdges) {
            drawLine(window, edge.start, edge.end, 2, sf::Color::Black);
        }

        // Draw gray rays
        for (const auto &ray : fov.rays) {
            sf::Vertex rayLine[] = {
                sf::Vertex(sf::Vector2f(sector.centre.x, sector.centre.y), sf::Color(128, 128, 128)),
                sf::Vertex(sf::Vector2f(sector.centre.x + ray.x, sector.centre.y + ray.y), sf::Color(128, 128, 128))
            };
            window.draw(rayLine, 2, sf::Lines);
        }

        // Draw red hit points
        for (const auto &p : fov.hitPoints) {
            drawCircle(window, p.x, p.y, 3, sf::Color::Red);
        }

        // Draw blue control points
        for (const auto &p : fov.ctrlPoints) {
            drawCircle(window, p.x, p.y, 3, sf::Color::Blue);
        }
    }

    // Display the updated window
    window.display();
}


// end of part 11 (line 941)

// Function to get the cursor position as a glm::vec2
glm::vec2 getCursorPosition(const sf::Event::MouseButtonEvent &mouseEvent, const sf::RenderWindow &window) {
    sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
    return glm::vec2(static_cast<float>(pixelPos.x), static_cast<float>(pixelPos.y));
}

glm::vec2 getCursorPosition(const sf::Event::MouseMoveEvent &mouseEvent, const sf::RenderWindow &window) {
    sf::Vector2i pixelPos(mouseEvent.x, mouseEvent.y);
    return glm::vec2(static_cast<float>(pixelPos.x), static_cast<float>(pixelPos.y));
}

void handleClick(const sf::Event::MouseButtonEvent &mouseEvent) {
    // Update observer location based on cursor position
    scene.observer.loc = getCursorPosition(mouseEvent, window);
    window.clear();  // Clear window to prepare for a new frame
    mainLoop();      // Call the main loop function to redraw the scene
}

void handleMouseMove(const sf::Event::MouseMoveEvent &mouseEvent) {
    // Calculate the direction the observer should look at
    glm::vec2 lookAt = getCursorPosition(mouseEvent, window);
    setDirection(scene.observer.dir, scene.observer.loc, lookAt);
    window.clear();  // Clear window to prepare for a new frame
    mainLoop();      // Call the main loop function to redraw the scene
}

// Initialize the SFML canvas (window)
bool init2DCanvas(sf::RenderWindow &window) {
    try {
        // Set window properties
        window.clear(wallColour);
        return true;
    } catch (const std::exception &e) {
        std::cerr << "Unable to initialize Canvas. Exception: " << e.what() << std::endl;
        return false;
    }
}

// Construct edges from a list of polygons
void constructEdges(std::vector<Polygon> &polygons) {
    for (auto &p : polygons) {
        size_t pointCount = p.coords.size();
        if (pointCount < 2) continue;  // Skip if not enough points

        // Handle polygons with a single edge (two points)
        size_t edgeCount = (pointCount > 2) ? pointCount : (pointCount - 1);
        p.edges.resize(edgeCount);

        for (size_t j = 0; j < edgeCount; ++j) {
            size_t k = (j + 1) % pointCount;

            // Create and store the edge
            p.edges[j] = Edge(p.coords[j], p.coords[k]);
        }
    }
}

void start() {
    // Initialize canvas
    init2DCanvas(window);

    // Define wall colour
    sf::Color wallColour(128, 128, 128);

    // Set up the scene
    scene.polygons = {
        { { glm::vec2(70, 50), glm::vec2(190, 70), glm::vec2(170, 140), glm::vec2(100, 130) }, wallColour, 1, true },
        { { glm::vec2(230, 50), glm::vec2(350, 70), glm::vec2(330, 140), glm::vec2(305, 90) }, wallColour, 1, true },
        { { glm::vec2(475, 56), glm::vec2(475, 360), glm::vec2(616, 360), glm::vec2(616, 56) }, wallColour, 1, true },
        { { glm::vec2(374, 300), glm::vec2(374, 450) }, wallColour, 1, false },
        { { glm::vec2(188.57f, 381.9f), glm::vec2(167.82f, 304.47f), glm::vec2(328.25f, 261.48f),
            glm::vec2(268.21f, 321.53f), glm::vec2(330.05f, 357.24f), glm::vec2(207.16f, 428.19f),
            glm::vec2(226.62f, 355.56f) }, wallColour, 1, true },
        { { glm::vec2(100, 200), glm::vec2(120, 250), glm::vec2(60, 300) }, wallColour, 1, true },
        { { glm::vec2(0, 0), glm::vec2(640, 0), glm::vec2(640, 480), glm::vec2(0, 480) }, wallColour, 1, false }
    };

    // Construct edges for all polygons
    constructEdges(scene.polygons);

    // Set observer properties
    scene.observer.loc = glm::vec2(374, 203);
    scene.observer.dir = glm::normalize(glm::vec2(-0.707106781186f, 0.707106781186f));
    scene.observer.colour = sf::Color::Green; // Example observer colour

    // Set target properties
    scene.target.loc = glm::vec2(293, 100);
    scene.target.dir = glm::normalize(glm::vec2(-1, 0));
    scene.target.colour = sf::Color::Red; // Example target colour

    // Define the observer's sector and field of view (FOV)
    float visionRadius = 150.0f; // Example radius

    std::vector<Edge> edges;
    edges.push_back({glm::vec2(0, 0), glm::vec2(0, 0)});
    edges.push_back({glm::vec2(0, 0), glm::vec2(0, 0)});

    scene.observer.sector = Sector(glm::vec2(0, 0), glm::vec2(0, 0), visionRadius, edges);


    scene.observer.fov = {
        {}, // Empty blocking edges
        {}, // Empty anglePtSet
        {}  // Empty anglePoints
    };

    // Event handling (equivalent of adding event listeners)
    window.setActive();

    // Main loop (similar to requestAnimationFrame in JS)
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed) {
                // Handle click
                handleClick(event.mouseButton);
            }

            if (event.type == sf::Event::MouseMoved) {
                // Handle mouse move
                handleMouseMove(event.mouseMove);
            }
        }

        // Call main loop function
        mainLoop();

        window.display();
    }
}

int main() {
    start();
    return 0;
}
