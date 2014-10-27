//
//  DistanceSolver.cpp
//  VoronoiMesh
//
//  Created by Litherum on 9/21/14.
//  Copyright (c) 2014 Litherum. All rights reserved.
//

#include "DistanceSolver.h"

#include <cmath>
#include <map>
#include <set>
#include <sstream>

#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/vector_angle.hpp>

static inline Polyhedron::Vertex_const_handle endVertex(Polyhedron::Halfedge_const_handle halfedge) {
    return halfedge->opposite()->vertex();
}

struct CandidateInterval {
    CandidateInterval()
    {
    }

    CandidateInterval(float a, float b, Polyhedron::Halfedge_const_handle halfedge, CGAL::Point_3<Kernel> r,
                      CGAL::Point_3<Kernel> rBar, float d, boost::variant<CGAL::Point_3<Kernel>, Polyhedron::Vertex_const_handle> c, CGAL::Point_3<Kernel> beta)
    : a(a)
    , b(b)
    , halfedge(halfedge)
    , r(r)
    , rBar(rBar)
    , d(d)
    , c(c)
    , beta(beta)
    {
    }

    CGAL::Point_3<Kernel> aPoint() const {
        auto point = halfedge->vertex()->point();
        auto vector = endVertex(halfedge)->point() - point;
        return point + a * vector;
    }

    CGAL::Point_3<Kernel> bPoint() const {
        auto point = halfedge->vertex()->point();
        auto vector = endVertex(halfedge)->point() - point;
        return point + b * vector;
    }

    bool operator<(const CandidateInterval& other) const {
        // FIXME: This doesn't work for obtuse triangles
        auto point = halfedge->vertex()->point();
        auto line = CGAL::Line_3<Kernel>(point, endVertex(halfedge)->point());
        auto projected1 = line.projection(beta);
        auto projected2 = line.projection(other.beta);
        return (projected1 - point).squared_length() < (projected2 - point).squared_length();
    }

    CGAL::Point_3<Kernel> frontierPoint() const;

private:
    class FrontierPointVisitor;

public:
    // Extent
    float a;
    float b;
    Polyhedron::Halfedge_const_handle halfedge; // Edge-face pair
    CGAL::Point_3<Kernel> r; // Root
    CGAL::Point_3<Kernel> rBar; // Unfolded root
    float d; // Depth
    boost::variant<CGAL::Point_3<Kernel>, Polyhedron::Vertex_const_handle> c; // Frontier point
    CGAL::Point_3<Kernel> beta; // Access point
};

struct Event {
    Event(float label, boost::variant<std::set<CandidateInterval>::iterator, Polyhedron::Vertex_const_handle> data)
        : label(label)
        , eventData(new boost::variant<std::set<CandidateInterval>::iterator, Polyhedron::Vertex_const_handle>(data))
    {
    }

    bool operator<(const Event& other) const {
        return label < other.label;
    }

    float label;
    std::shared_ptr<boost::variant<std::set<CandidateInterval>::iterator, Polyhedron::Vertex_const_handle>> eventData;
};

class ModelPreprocessor {
public:
    ModelPreprocessor(const Polyhedron&, const std::vector<InitialPoint>&);
    void preprocessModel();
private:
    class PropagateVisitor;
    class EventLoopEventVisitor;
    class Printer;

    void insertInterval(CandidateInterval&&, const CGAL::Point_3<Kernel>& c, std::set<CandidateInterval>& edgeIntervals);
    std::set<CandidateInterval>::iterator deleteInterval(std::set<CandidateInterval>::iterator, std::set<CandidateInterval>& edgeIntervals);
    void propagate(const std::set<CandidateInterval>::iterator);
    boost::optional<CandidateInterval> project(const CandidateInterval& i, Polyhedron::Halfedge_const_handle ii) const;

    const Polyhedron& polyhedron;
    const std::vector<InitialPoint>& initialPoints;
    std::map<Polyhedron::Halfedge_const_handle, std::set<CandidateInterval>> halfedgeIntervalMap;
    std::set<Event> eventQueue;
    std::map<std::shared_ptr<boost::variant<std::set<CandidateInterval>::iterator, Polyhedron::Vertex_const_handle>>, float> permanentLabels;
};

class CandidateInterval::FrontierPointVisitor : public boost::static_visitor<CGAL::Point_3<Kernel>> {
public:
    CGAL::Point_3<Kernel> operator()(const CGAL::Point_3<Kernel>& point) const {
        return point;
    }

    CGAL::Point_3<Kernel> operator()(Polyhedron::Vertex_const_handle handle) const {
        return handle->point();
    }
};

CGAL::Point_3<Kernel> CandidateInterval::frontierPoint() const {
    return boost::apply_visitor(FrontierPointVisitor(), c);
}

ModelPreprocessor::ModelPreprocessor(const Polyhedron& polyhedron, const std::vector<InitialPoint>& initialPoints)
    : polyhedron(polyhedron)
    , initialPoints(initialPoints)
{
}

class DeleteEventVisitor : public boost::static_visitor<bool> {
public:
    DeleteEventVisitor(std::set<CandidateInterval>::iterator item)
        : item(item)
    {
    }

    bool operator()(std::set<CandidateInterval>::iterator i) const {
        return i == item;
    }

    bool operator()(Polyhedron::Vertex_const_handle) const {
        return false;
    }

private:
    std::set<CandidateInterval>::iterator item;
};

std::set<CandidateInterval>::iterator ModelPreprocessor::deleteInterval(std::set<CandidateInterval>::iterator item, std::set<CandidateInterval>& edgeIntervals) {
    for (auto i = eventQueue.begin(); i != eventQueue.end();) {
        if (boost::apply_visitor(DeleteEventVisitor(item), *(i->eventData)))
            i = eventQueue.erase(i);
        else
            ++i;
    }

    return edgeIntervals.erase(item);
}

static std::pair<float, float> intersectCoplanarLines(CGAL::Point_3<Kernel> p1, CGAL::Vector_3<Kernel> v1, CGAL::Point_3<Kernel> p2, CGAL::Vector_3<Kernel> v2) {
    float m;
    float n;
    if (std::abs(v1.x()) >= std::abs(v1.y()) && std::abs(v1.x()) >= std::abs(v1.z())) {
        float numerator = p1.y() - p2.y() + (p2.x() * v1.y() / v1.x()) - (p1.x() * v1.y() / v1.x());
        float denominator = v2.y() - (v2.x() * v1.y() / v1.x());
        n = numerator / denominator;
        m = (p2.x() + n * v2.x() - p1.x()) / v1.x();
    } else if (std::abs(v1.y()) >= std::abs(v1.x()) && std::abs(v1.y()) >= std::abs(v1.z())) {
        float numerator = p1.x() - p2.x() + (p2.y() * v1.x() / v1.y()) - (p1.y() * v1.x() / v1.y());
        float denominator = v2.x() - (v2.y() * v1.x() / v1.y());
        n = numerator / denominator;
        m = (p2.y() + n * v2.y() - p1.y()) / v1.y();
    } else {
        float numerator = p1.y() - p2.y() + (p2.z() * v1.y() / v1.z()) - (p1.z() * v1.y() / v1.z());
        float denominator = v2.y() - (v2.z() * v1.y() / v1.z());
        n = numerator / denominator;
        m = (p2.z() + n * v2.z() - p1.z()) / v1.z();
    }
    auto u1 = p1 + m * v1;
    auto u2 = p2 + n * v2;
    auto length = std::sqrt((u1 - u2).squared_length());
    assert(length < 0.0001);
    return std::make_pair(m, n);
}

static CGAL::Point_3<Kernel> calculateAccessPoint(Polyhedron::Halfedge_const_handle halfedge, float frontierPointFrac, CGAL::Point_3<Kernel> unfoldedRoot, CGAL::Point_3<Kernel> oldBeta) {
    assert(frontierPointFrac >= 0 && frontierPointFrac <= 1);
    if (frontierPointFrac == 0 || frontierPointFrac == 1)
        return oldBeta;

    auto point = halfedge->vertex()->point();
    auto frontierPoint = point + frontierPointFrac * (endVertex(halfedge)->point() - point);
    auto vectorToUnfoldedRoot = unfoldedRoot - frontierPoint;

    auto circulator = halfedge->facet_begin();
    for (++circulator; circulator != halfedge->facet_begin(); ++circulator) {
        auto circulatorPoint = circulator->vertex()->point();
        auto circulatorVector = endVertex(circulator)->point() - circulatorPoint;
        auto intersection = intersectCoplanarLines(frontierPoint, vectorToUnfoldedRoot, circulatorPoint, circulatorVector);
        if (intersection.second >= 0 && intersection.second <= 1)
            return circulatorPoint + intersection.second * circulatorVector;
    }
    assert(false);
    return CGAL::Point_3<Kernel>(0, 0, 0);
}

static float angle(CGAL::Vector_3<Kernel> v1, CGAL::Vector_3<Kernel> v2) {
    auto u1 = glm::normalize(glm::vec3(v1.x(), v1.y(), v1.z()));
    auto u2 = glm::normalize(glm::vec3(v2.x(), v2.y(), v2.z()));
    return glm::angle(u1, u2);
}

static float orientedAngle(CGAL::Vector_3<Kernel> v1, CGAL::Vector_3<Kernel> v2, CGAL::Vector_3<Kernel> ref) {
    auto u1 = glm::normalize(glm::vec3(v1.x(), v1.y(), v1.z()));
    auto u2 = glm::normalize(glm::vec3(v2.x(), v2.y(), v2.z()));
    auto u3 = glm::normalize(glm::vec3(ref.x(), ref.y(), ref.z()));
    return glm::orientedAngle(u1, u2, u3);
}

static boost::optional<std::pair<float, float>> compuateM(float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, CGAL::Point_3<Kernel> p, CGAL::Vector_3<Kernel> v) {
    // a*x^2 + b*y^2 + c*z^2 + d*x*y + e*x*z + f*y*z + g*x + h*y + i*z + j = 0
    // [x, y, z] = p + m*v
    // 0 = q*m^2 + r*m + s
    auto q = a * v.x() * v.x() + b * v.y() * v.y() + c * v.z() * v.z() + d * v.x() * v.y() + e * v.x() * v.z() + f * v.y() * v.z();
    auto r = 2 * a * p.x() * v.x() + 2 * b * p.y() + v.y() + 2 * c * p.z() * v.z()
            + d * (p.x() * v.y() + v.x() + p.y()) + e * (p.x() * v.z() + p.z() * v.x()) + f * (p.y() * v.z() + p.z() + v.y())
            + g * v.x() + h * v.y() + i * v.z();
    auto s = a * p.x() * p.x() + b * p.y() * p.y() + c * p.z() * p.z()
            + d * p.x() * p.y() + e * p.x() * p.z() + f * p.y() * p.z()
            + g * p.x() + h * p.y() + i * p.z() + j;
    auto discriminant = r * r - 4 * q * s;
    if (discriminant < 0)
        return boost::optional<std::pair<float, float>>();
    auto result1 = (-r + std::sqrt(discriminant)) / (2 * q);
    {
        auto substitution = p + result1 * v;
        auto epsilon = 0.1f;
        assert(std::abs(a * std::pow(substitution.x(), 2) + b * std::pow(substitution.y(), 2) + c * std::pow(substitution.z(), 2)
                + d * substitution.x() * substitution.y() + e * substitution.x() * substitution.z() + f * substitution.y() * substitution.z()
                + g * substitution.x() + h * substitution.y() + i * substitution.z() + j) < epsilon);
    }
    auto result2 = (-r - std::sqrt(discriminant)) / (2 * q);
    {
        auto substitution = p + result2 * v;
        auto epsilon = 0.1f;
        assert(std::abs(a * std::pow(substitution.x(), 2) + b * std::pow(substitution.y(), 2) + c * std::pow(substitution.z(), 2)
                + d * substitution.x() * substitution.y() + e * substitution.x() * substitution.z() + f * substitution.y() * substitution.z()
                + g * substitution.x() + h * substitution.y() + i * substitution.z() + j) < epsilon);
    }
    return std::make_pair(result1, result2);
}

boost::optional<float> calculateTiePoint(const CandidateInterval& interval1, const CandidateInterval& interval) {
    assert(interval1.halfedge == interval.halfedge);
    // interval1.d + |interval1.rBar - a| = interval.d + |interval.rBar - a|
    auto p = interval1.halfedge->vertex()->point();
    auto v = endVertex(interval1.halfedge)->point() - p;
    if (interval1.d == interval.d) {
        auto dr = interval1.rBar - interval.rBar;
        auto numerator = -2 * ((p - CGAL::Point_3<Kernel>(0, 0, 0)) * dr) + interval1.rBar.x() * interval1.rBar.x() - interval.rBar.x() * interval.rBar.x()
                + interval1.rBar.y() * interval1.rBar.y() - interval.rBar.y() * interval.rBar.y()
                + interval1.rBar.z() * interval1.rBar.z() - interval.rBar.z() * interval.rBar.z();
        auto denominator = 2 * (v * dr);
        auto result = numerator / denominator;
        {
            auto a = p + result * v;
            auto epsilon = 0.0001f;
            auto error = std::abs(interval1.d + std::sqrt((interval1.rBar - a).squared_length()) - interval.d - std::sqrt((interval.rBar - a).squared_length()));
            assert(error < epsilon);
        }
        if (result >= interval1.a && result <= interval1.b && result >= interval.a && result <= interval.b)
            return result;
        return boost::optional<float>();
    }
    auto c = interval1.d - interval.d;
    auto v2 = (interval1.rBar - interval.rBar) / c;
    auto d = interval.rBar.x() * interval.rBar.x() - interval1.rBar.x() * interval1.rBar.x()
            + interval.rBar.y() * interval.rBar.y() - interval1.rBar.y() * interval1.rBar.y()
            + interval.rBar.z() * interval.rBar.z() - interval1.rBar.z() * interval1.rBar.z();
    auto d2 = (d - c * c) / (2 * c);
    auto m = compuateM(v2.x() * v2.x() - 1, v2.y() * v2.y() - 1, v2.z() * v2.z() - 1,
            2 * v2.x() * v2.y(), 2 * v2.x() * v2.z(), 2 * v2.y() * v2.z(),
            2 * (v2.x() * d2 + interval1.rBar.x()), 2 * (v2.y() * d2 + interval1.rBar.y()), 2 * (v2.z() * d2 + interval1.rBar.z()),
            d2 * d2 - interval1.rBar.x() * interval1.rBar.x() - interval1.rBar.y() * interval1.rBar.y() - interval1.rBar.z() * interval1.rBar.z(),
            p, v);
    if (!m)
        return boost::optional<float>();
    auto result1 = m->first;
    {
        auto a = p + result1 * v;
        auto epsilon = 0.1f;
        assert(std::abs(interval1.d + std::sqrt((interval1.rBar - a).squared_length()) - interval.d - std::sqrt((interval.rBar - a).squared_length())) < epsilon);
    }
    if (result1 >= interval1.a && result1 <= interval1.b && result1 >= interval.a && result1 <= interval.b)
        return result1;
    auto result2 = m->second;
    {
        auto a = p + result1 * v;
        auto epsilon = 0.1f;
        assert(std::abs(interval1.d + std::sqrt((interval1.rBar - a).squared_length()) - interval.d - std::sqrt((interval.rBar - a).squared_length())) < epsilon);
    }
    if (result2 >= interval1.a && result2 <= interval1.b && result2 >= interval.a && result2 <= interval.b)
        return result2;
    return boost::optional<float>();
}

static float calculateFrontierPointFrac(Polyhedron::Halfedge_const_handle ei, float a, float b, CGAL::Point_3<Kernel> unfoldedRoot) {
    auto eiEndVertexPoint = endVertex(ei)->point();
    auto projected = CGAL::Line_3<Kernel>(ei->vertex()->point(), eiEndVertexPoint).projection(unfoldedRoot);
    auto frac = std::sqrt((projected - ei->vertex()->point()).squared_length()) / std::sqrt((eiEndVertexPoint - ei->vertex()->point()).squared_length());
    return glm::clamp(frac, a, b);
}

static void updateCAndBeta(CandidateInterval& interval) {
    auto point = interval.halfedge->vertex()->point();
    auto vector = endVertex(interval.halfedge)->point() - point;

    auto frontierPointFrac = calculateFrontierPointFrac(interval.halfedge, interval.a, interval.b, interval.rBar);
    if (frontierPointFrac == 0)
        interval.c = interval.halfedge->vertex();
    else if (frontierPointFrac == 1)
        interval.c = endVertex(interval.halfedge);
    else
        interval.c = point + vector * frontierPointFrac;
    interval.beta = calculateAccessPoint(interval.halfedge, frontierPointFrac, interval.rBar, interval.beta);
}

static void trimAtTiePoint(const CandidateInterval& iter, CandidateInterval& i, float CandidateInterval::*v1, float CandidateInterval::*v2) {
    assert(iter.halfedge == i.halfedge);

    assert(iter.a <= iter.b && i.a <= i.b);
    if (iter.b < i.a || i.b < iter.a)
        return;

    if (auto a = calculateTiePoint(iter, i))
        i.*v1 = *a;
    else
        i.*v1 = iter.*v2;
    // FIXME: Remove iter.*v2 from the event queue. This doesn't seem necessary or even possible

    {
        // This is very dangerous! I am only doing it because I know it won't corrupt the set.
        auto& interval1 = const_cast<CandidateInterval&>(iter);
        interval1.*v2 = i.*v1;
        updateCAndBeta(interval1);
    }
}

void ModelPreprocessor::insertInterval(CandidateInterval&& i, const CGAL::Point_3<Kernel>& c, std::set<CandidateInterval>& edgeIntervals) {
    if (edgeIntervals.empty()) {
        auto result = edgeIntervals.emplace(std::forward<CandidateInterval>(i)).first;
        if (i.a == 0)
            eventQueue.insert(Event(i.d + std::sqrt((i.halfedge->vertex()->point() - i.rBar).squared_length()), i.halfedge->vertex()));
        if (i.b == 1)
            eventQueue.insert(Event(i.d + std::sqrt((endVertex(i.halfedge)->point() - i.rBar).squared_length()), endVertex(i.halfedge)));
        // FIXME: Don't always insert c... maybe?
        eventQueue.insert(Event(i.d + std::sqrt((i.frontierPoint() - i.rBar).squared_length()), result));
        return;
    }

    auto i2Iter = edgeIntervals.upper_bound(i);
    auto i1Iter = edgeIntervals.end();
    if (i2Iter != edgeIntervals.end() && i2Iter != edgeIntervals.begin()) {
        i1Iter = i2Iter;
        --i1Iter;
    } else if (i2Iter == edgeIntervals.end())
        i1Iter = std::max_element(edgeIntervals.begin(), edgeIntervals.end());

    //assert(i1Iter == edgeIntervals.end() || i1Iter->beta != i.beta);
    assert(i1Iter == edgeIntervals.end() || i1Iter->halfedge == i.halfedge);
    assert(i2Iter == edgeIntervals.end() || i2Iter->halfedge == i.halfedge);

    while (i1Iter != edgeIntervals.end() &&
            i1Iter->a >= i.a &&
            i1Iter->a <= i.b &&
            i1Iter->d + std::sqrt((i1Iter->aPoint() - i1Iter->rBar).squared_length()) >= i.d + std::sqrt((i.aPoint() - i.rBar).squared_length())) {
        i1Iter = deleteInterval(i1Iter, edgeIntervals);
        assert(i1Iter == i2Iter);
        if (i1Iter == edgeIntervals.begin())
            i1Iter = edgeIntervals.end();
        else
            --i1Iter;
    }

    while (i2Iter != edgeIntervals.end() &&
            i2Iter->b >= i.a &&
            i2Iter->b <= i.b &&
            i2Iter->d + std::sqrt((i2Iter->bPoint() - i2Iter->rBar).squared_length()) >= i.d + std::sqrt((i.bPoint() - i.rBar).squared_length())) {
        i2Iter = deleteInterval(i1Iter, edgeIntervals);
    }

    // FIXME: Deal with trying to insert a dominated interval. Maybe this won't happen?
    if (i1Iter != edgeIntervals.end())
        trimAtTiePoint(*i1Iter, i, &CandidateInterval::a, &CandidateInterval::b);

    if (i2Iter != edgeIntervals.end())
        trimAtTiePoint(*i2Iter, i, &CandidateInterval::b, &CandidateInterval::a);

    if (i.a == i.b)
        return;

    updateCAndBeta(i);
    auto result = edgeIntervals.emplace(std::forward<CandidateInterval>(i)).first;
    if (i.a == 0)
        eventQueue.insert(Event(i.d + std::sqrt((i.halfedge->vertex()->point() - i.rBar).squared_length()), i.halfedge->vertex()));
    if (i.b == 1)
        eventQueue.insert(Event(i.d + std::sqrt((endVertex(i.halfedge)->point() - i.rBar).squared_length()), endVertex(i.halfedge)));
    // FIXME: Don't always insert c... maybe?
    eventQueue.insert(Event(i.d + std::sqrt((i.frontierPoint() - i.rBar).squared_length()), result));
}

boost::optional<CandidateInterval> ModelPreprocessor::project(const CandidateInterval& i, Polyhedron::Halfedge_const_handle ei) const {
    auto intervalHalfedge = i.halfedge;
    auto halfedgePoint = intervalHalfedge->vertex()->point();
    auto halfedgeVector = endVertex(intervalHalfedge)->point() - halfedgePoint;
    auto eiEndVertex = endVertex(ei);
    auto eiVector = eiEndVertex->point() - ei->vertex()->point();
    auto p1 = halfedgePoint + i.a * halfedgeVector;
    auto p2 = halfedgePoint + i.b * halfedgeVector;

    // rotate unwrappedRoot into the plane of ei->facet()->plane()
    auto unfoldedRoot = i.rBar;
    auto rotationAngle = orientedAngle(intervalHalfedge->facet()->plane().orthogonal_vector(),
            ei->facet()->plane().orthogonal_vector(),
            halfedgeVector);
    // FIXME: Use GLKit
    glm::vec3 glmPoint(halfedgePoint.x(), halfedgePoint.y(), halfedgePoint.z());
    glm::vec3 glmVector(halfedgeVector.x(), halfedgeVector.y(), halfedgeVector.z());
    glm::mat4 transformation = glm::translate(glmPoint) * glm::rotate(rotationAngle, glmVector) * glm::translate(-glmPoint);
    assert(transformation[0][3] == 0 && transformation[1][3] == 0 && transformation[2][3] == 0 && transformation[3][3] == 1);

    CGAL::Aff_transformation_3<Kernel> cgalTransformation(transformation[0][0], transformation[1][0], transformation[2][0], transformation[3][0],
            transformation[0][1], transformation[1][1], transformation[2][1], transformation[3][1],
            transformation[0][2], transformation[1][2], transformation[2][2], transformation[3][2]);
    unfoldedRoot = cgalTransformation.transform(unfoldedRoot);
    assert(std::sqrt((unfoldedRoot - ei->facet()->plane().projection(unfoldedRoot)).squared_length()) < 0.0001f);
    unfoldedRoot = ei->facet()->plane().projection(unfoldedRoot);

    auto aintersection = intersectCoplanarLines(ei->vertex()->point(), eiVector, unfoldedRoot, p1 - unfoldedRoot);
    auto bintersection = intersectCoplanarLines(ei->vertex()->point(), eiVector, unfoldedRoot, p2 - unfoldedRoot);

    auto a = aintersection.first;
    auto b = bintersection.first;

    if (aintersection.second < 0)
        a = (aintersection.first > 0 ? -1 : 1) * std::numeric_limits<float>::infinity();
    if (bintersection.second < 0)
        b = (bintersection.first > 0 ? -1 : 1) *std::numeric_limits<float>::infinity();

    if ((a > 1 && b > 1) || (a < 0 && b < 0))
        return boost::optional<CandidateInterval>();

    a = glm::clamp<float>(a, 0, 1);
    b = glm::clamp<float>(b, 0, 1);

    if (a > b)
        std::swap(a, b);

    auto frontierPointFrac = calculateFrontierPointFrac(ei, a, b, unfoldedRoot);
    boost::variant<CGAL::Point_3<Kernel>, Polyhedron::Vertex_const_handle> frontierPoint;
    if (frontierPointFrac == 0)
        frontierPoint = ei->vertex();
    else if (frontierPointFrac == 1)
        frontierPoint = eiEndVertex;
    else
        frontierPoint = ei->vertex()->point() + frontierPointFrac * eiVector;

    return CandidateInterval(a, b, ei, i.r, unfoldedRoot, i.d, frontierPoint, i.frontierPoint());
}

template <typename T> static inline int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// FIXME: We might want the minimum projected distance
static bool halfedgeContainsBeta(Polyhedron::Halfedge_const_handle halfedge, const CGAL::Point_3<Kernel>& beta) {
    if (halfedge->facet() == Polyhedron::Facet_const_handle())
        return false;
    auto projected = halfedge->facet()->plane().projection(beta);
    auto distance = (beta - projected).squared_length();
    auto facetCirculator = halfedge->facet_begin();
    auto p0 = facetCirculator->vertex()->point();
    ++facetCirculator;
    auto p1 = facetCirculator->vertex()->point();
    ++facetCirculator;
    auto p2 = facetCirculator->vertex()->point();
    ++facetCirculator;
    assert(facetCirculator == halfedge->facet_begin());
    assert(Polyhedron::Halfedge_const_handle(facetCirculator) == halfedge);
    auto projectedVector = projected - p0;
    auto v1 = p1 - p0;
    auto v2 = p2 - p0;
    auto cross1 = CGAL::cross_product(projectedVector, v1);
    auto cross2 = CGAL::cross_product(v2, projectedVector);
    auto epsilon = 1.0f;
    return sgn(cross1.x()) == sgn(cross2.x())
        && sgn(cross1.y()) == sgn(cross2.y())
        && sgn(cross1.z()) == sgn(cross2.z())
        && distance <= epsilon;
}

static Polyhedron::Halfedge_around_vertex_const_circulator facetBetaLiesWithin(Polyhedron::Vertex_const_handle vertex, CGAL::Point_3<Kernel> beta) {
    auto vertexCirculator = vertex->vertex_begin();
    do {
        if (halfedgeContainsBeta(vertexCirculator, beta))
            return vertexCirculator;
        ++vertexCirculator;
    } while (vertexCirculator != vertex->vertex_begin());
    assert(false);
    return Polyhedron::Halfedge_around_vertex_const_circulator();
}

static float projectFrac(CGAL::Vector_3<Kernel> of, CGAL::Vector_3<Kernel> onto) {
    // FIXME: Might be able to use CGAL::Line_3<Kernel>::projection() instead
    return of * onto / onto.squared_length();
}

static std::vector<Polyhedron::Halfedge_const_handle> oppositeEdgesThatNeedCandidateIntervals(Polyhedron::Vertex_const_handle vertex, CGAL::Point_3<Kernel> beta) {
    auto originalHalfedge = facetBetaLiesWithin(vertex, beta);
    auto originalSecondHalfedge = originalHalfedge;
    ++originalSecondHalfedge;
    assert(originalHalfedge->vertex() == vertex);
    assert(originalHalfedge->vertex() == originalSecondHalfedge->vertex());

    auto halfedge = originalHalfedge;
    auto secondHalfedge = originalSecondHalfedge;
    auto sum = angle(endVertex(secondHalfedge)->point() - vertex->point(), beta - vertex->point());
    halfedge = secondHalfedge;
    ++secondHalfedge;
    do {
        sum += angle(endVertex(secondHalfedge)->point() - vertex->point(), endVertex(halfedge)->point() - vertex->point());
        halfedge = secondHalfedge;
        ++secondHalfedge;
    } while (sum < M_PI);
    auto startingHalfedge = halfedge;

    halfedge = originalHalfedge;
    secondHalfedge = originalSecondHalfedge;
    sum = angle(endVertex(halfedge)->point() - vertex->point(), beta - vertex->point());
    secondHalfedge = halfedge;
    --halfedge;
    do {
        sum += angle(endVertex(halfedge)->point() - vertex->point(), endVertex(secondHalfedge)->point() - vertex->point());
        secondHalfedge = halfedge;
        --halfedge;
    } while (sum < M_PI);
    auto endingHalfedge = secondHalfedge;

    std::vector<Polyhedron::Halfedge_const_handle> result;
    do {
        assert(startingHalfedge != originalHalfedge);
        auto facetCirculator = startingHalfedge->facet_begin();
        assert(facetCirculator->vertex() == startingHalfedge->vertex());
        ++facetCirculator;
        result.push_back(facetCirculator);
        ++startingHalfedge;
    } while (startingHalfedge != endingHalfedge);
    return result;
}

static CandidateInterval createCandidateIntervalOverEntireEdge(Polyhedron::Halfedge_const_handle halfedge, CGAL::Point_3<Kernel> root) {
    auto triangleVertexToRoot = root - halfedge->vertex()->point();
    auto halfedgeVector = endVertex(halfedge)->point() - halfedge->vertex()->point();
    auto advanceFrac = projectFrac(triangleVertexToRoot, halfedgeVector);

    CGAL::Point_3<Kernel> frontierPoint;

    CandidateInterval candidateInterval;
    if (advanceFrac <= 0)
        candidateInterval = CandidateInterval(0, 1, halfedge, root, root, 0, halfedge->vertex(), root);
    else if (advanceFrac >= 1)
        candidateInterval = CandidateInterval(0, 1, halfedge, root, root, 0, endVertex(halfedge), root);
    else {
        frontierPoint = halfedge->vertex()->point() + halfedgeVector * advanceFrac;
        candidateInterval = CandidateInterval(0, 1, halfedge, root, root, 0, frontierPoint, root);
    }
    return candidateInterval;
}

class ModelPreprocessor::PropagateVisitor : public boost::static_visitor<> {
public:
    PropagateVisitor(ModelPreprocessor& modelPreprocessor, const CandidateInterval& i)
        : modelPreprocessor(modelPreprocessor)
        , i(i)
    {
    }

    void operator()(CGAL::Point_3<Kernel> c) const {
        auto opposite = i.halfedge->opposite();
        assert(opposite->vertex() == endVertex(i.halfedge));
        auto facetCirculator = opposite->facet_begin();
        assert(facetCirculator->opposite() == i.halfedge);
        if (facetCirculator->facet() == Polyhedron::Facet_const_handle())
            return;
        for (++facetCirculator; facetCirculator != opposite->facet_begin(); ++facetCirculator) {
            Polyhedron::Halfedge_const_handle halfedge = facetCirculator;
            std::cout << i.halfedge - modelPreprocessor.polyhedron.halfedges_begin() << " " << halfedge - modelPreprocessor.polyhedron.halfedges_begin() << std::endl;
            if (auto ii = modelPreprocessor.project(i, halfedge)) {
                assert(halfedge != i.halfedge && ii->halfedge == halfedge);
                modelPreprocessor.insertInterval(std::move(*ii), i.rBar, modelPreprocessor.halfedgeIntervalMap.emplace(halfedge, std::set<CandidateInterval>()).first->second);
            }
        }
    }

    void operator()(Polyhedron::Vertex_const_handle c) const {
        for (auto halfedge : oppositeEdgesThatNeedCandidateIntervals(c, i.beta)) {
            // FIXME: createCandidateIntervalOverEntireEdge might be doing the wrong thing here
            modelPreprocessor.insertInterval(createCandidateIntervalOverEntireEdge(halfedge, c->point()), c->point(), modelPreprocessor.halfedgeIntervalMap.emplace(halfedge, std::set<CandidateInterval>()).first->second);
        }
    }

private:
    ModelPreprocessor& modelPreprocessor;
    const CandidateInterval& i;
};

void ModelPreprocessor::propagate(const std::set<CandidateInterval>::iterator i) {
    boost::apply_visitor(PropagateVisitor(*this, *i), i->c);
}

class ModelPreprocessor::EventLoopEventVisitor : public boost::static_visitor<> {
public:
    EventLoopEventVisitor(ModelPreprocessor& modelPreprocessor)
        : modelPreprocessor(modelPreprocessor)
    {
    }

    void operator()(std::set<CandidateInterval>::iterator candidateInterval) const {
        modelPreprocessor.propagate(candidateInterval);
    }

    void operator()(Polyhedron::Vertex_const_handle) const {
    }

private:
    ModelPreprocessor& modelPreprocessor;
};

class ModelPreprocessor::Printer : public boost::static_visitor<std::string> {
public:
    Printer(ModelPreprocessor& modelPreprocessor)
        : modelPreprocessor(modelPreprocessor)
    {
    }

    std::string operator()(std::set<CandidateInterval>::iterator candidateInterval) const {
        std::ostringstream out;
        out << candidateInterval->halfedge - modelPreprocessor.polyhedron.halfedges_begin();
        return out.str();
    }

    std::string operator()(Polyhedron::Vertex_const_handle) const {
        return "Vertex";
    }

private:
    ModelPreprocessor& modelPreprocessor;
};

void ModelPreprocessor::preprocessModel() {
    for (const auto& initialPoint : initialPoints) {
        auto facetCirculator = initialPoint.facet->facet_begin();
        do {
            auto halfedge = facetCirculator;
            insertInterval(createCandidateIntervalOverEntireEdge(halfedge, initialPoint.point),
                    initialPoint.point, halfedgeIntervalMap.emplace(halfedge, std::set<CandidateInterval>()).first->second);
            ++facetCirculator;
        } while (facetCirculator != initialPoint.facet->facet_begin());
    }

    while (!eventQueue.empty()) {
        //for (auto& i : eventQueue)
        //    std::cout << "Event queue: " << boost::apply_visitor(Printer(*this), *i.eventData) << std::endl;
        auto eventIter = eventQueue.begin();
        const auto& event = permanentLabels.emplace(eventIter->eventData, eventIter->label).first->first;
        boost::apply_visitor(EventLoopEventVisitor(*this), *event);
        eventQueue.erase(eventIter);
    }
}

void preprocessModel(const Polyhedron& polyhedron, const std::vector<InitialPoint>& initialPoints) {
    ModelPreprocessor(polyhedron, initialPoints).preprocessModel();
}
