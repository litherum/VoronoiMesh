//
//  DistanceSolver.cpp
//  VoronoiMesh
//
//  Created by Litherum on 9/21/14.
//  Copyright (c) 2014 Litherum. All rights reserved.
//

#include "DistanceSolver.h"

#include <map>
#include <set>

#define GLM_FORCE_RADIANS
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/vector_angle.hpp>

static Polyhedron::Vertex_const_handle endVertex(Polyhedron::Halfedge_const_handle halfedge) {
    auto facetCirculator = halfedge->facet_begin();
    assert(Polyhedron::Halfedge_const_handle(facetCirculator) == halfedge);
    ++facetCirculator;
    return facetCirculator->vertex();
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

    std::set<CandidateInterval>::iterator insertInterval(CandidateInterval&&, const CGAL::Point_3<Kernel>& c, std::set<CandidateInterval>& edgeIntervals);
    std::set<CandidateInterval>::iterator deleteInterval(std::set<CandidateInterval>::iterator, std::set<CandidateInterval>& edgeIntervals);
    void propagate(const std::set<CandidateInterval>::iterator);
    boost::optional<CandidateInterval> project(const CandidateInterval& i, Polyhedron::Halfedge_const_handle ii) const;

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

ModelPreprocessor::ModelPreprocessor(const Polyhedron&, const std::vector<InitialPoint>& initialPoints)
    : initialPoints(initialPoints)
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
    for (auto i = eventQueue.begin(); i != eventQueue.end(); ++i) {
        if (boost::apply_visitor(DeleteEventVisitor(item), *(i->eventData)))
            i = eventQueue.erase(i);
        else
            ++i;
    }

    return edgeIntervals.erase(item);
}

static CGAL::Point_3<Kernel> calculateAccessPoint(Polyhedron::Halfedge_const_handle halfedge, float frontierPointFrac, CGAL::Point_3<Kernel> unfoldedRoot, CGAL::Point_3<Kernel> oldBeta) {
    assert(frontierPointFrac >= 0 && frontierPointFrac <= 1);
    if (frontierPointFrac == 0 || frontierPointFrac == 1)
        return oldBeta;

    auto point = halfedge->vertex()->point();
    auto vector = endVertex(halfedge)->point() - point;
    auto frontierPoint = point + frontierPointFrac * vector;

    auto circulator = halfedge->facet_begin();
    for (++circulator; circulator != halfedge->facet_begin(); ++circulator) {
        point = circulator->vertex()->point();
        auto point2 = endVertex(circulator)->point();
        auto intersection = CGAL::intersection(CGAL::Ray_3<Kernel>(point, point2), CGAL::Ray_3<Kernel>(frontierPoint, unfoldedRoot));
        assert(intersection);
        auto intersectionPoint = boost::get<CGAL::Point_3<Kernel>>(*intersection);
        float frac = std::sqrt((intersectionPoint - point).squared_length()) / std::sqrt((point2 - point).squared_length());
        if (frac >= 0 && frac <= 1)
            return intersectionPoint;
    }
    assert(false);
    return CGAL::Point_3<Kernel>(0, 0, 0);
}

/*static CGAL::Vector_3<Kernel> unitize(CGAL::Vector_3<Kernel> x) {
    return x / std::sqrt(x.squared_length());
}*/

static float angle(CGAL::Vector_3<Kernel> v1, CGAL::Vector_3<Kernel> v2) {
    auto u1 = glm::normalize(glm::vec3(v1.x(), v1.y(), v1.z()));
    auto u2 = glm::normalize(glm::vec3(v2.x(), v2.y(), v2.z()));
    return glm::angle(u1, u2);
    //return std::acos(unitize(v1) * unitize(v2));
}

CGAL::Point_3<Kernel> calculateTiePoint(const CandidateInterval& interval1, const CandidateInterval& interval2) {
    // Hyperbola. https://people.richland.edu/james/lecture/m116/conics/hypdef.html
    //float a = (interval1.d - interval2.d) / 2;
    //float b = ((interval1.rBar - interval2.rBar) / 2).squared_length() - a * a;

    // FIXME: Write this function
    assert(interval1.halfedge == interval2.halfedge);
    return CGAL::Point_3<Kernel>(0, 0, 0);
}

static float calculateFrontierPointFrac(Polyhedron::Halfedge_const_handle ei, float a, float b, CGAL::Point_3<Kernel> unfoldedRoot) {
    auto eiEndVertexPoint = endVertex(ei)->point();
    auto projected = CGAL::Line_3<Kernel>(ei->vertex()->point(), eiEndVertexPoint).projection(unfoldedRoot);
    auto frac = std::sqrt((projected - ei->vertex()->point()).squared_length()) / std::sqrt((eiEndVertexPoint - ei->vertex()->point()).squared_length());
    return glm::clamp(frac, a, b);
}

static void updateCAndBeta(CandidateInterval& interval) {
    auto point = interval.halfedge->vertex()->point();
    auto nextPoint = endVertex(interval.halfedge)->point();
    auto vector = nextPoint - point;

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

    auto point = i.halfedge->vertex()->point();
    auto nextPoint = endVertex(i.halfedge)->point();
    auto vector = nextPoint - point;

    auto a = calculateTiePoint(iter, i);
    auto frac = std::sqrt((a - point).squared_length()) / std::sqrt(vector.squared_length());
    if (frac >= iter.a && frac <= iter.b && frac >= i.a && frac <= i.b)
        i.*v1 = frac;
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

std::set<CandidateInterval>::iterator ModelPreprocessor::insertInterval(CandidateInterval&& i, const CGAL::Point_3<Kernel>& c, std::set<CandidateInterval>& edgeIntervals) {
    if (edgeIntervals.empty())
        return edgeIntervals.emplace(std::forward<CandidateInterval>(i)).first;

    auto i2Iter = edgeIntervals.upper_bound(i);
    auto i1Iter = edgeIntervals.end();
    if (i2Iter != edgeIntervals.end() && i2Iter != edgeIntervals.begin()) {
        i1Iter = i2Iter;
        --i1Iter;
    } else if (i2Iter == edgeIntervals.end())
        i1Iter = std::max_element(edgeIntervals.begin(), edgeIntervals.end());

    assert(i1Iter == edgeIntervals.end() || i1Iter->beta != i.beta);

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

    if (i1Iter != edgeIntervals.end())
        trimAtTiePoint(*i1Iter, i, &CandidateInterval::a, &CandidateInterval::b);

    if (i2Iter != edgeIntervals.end())
        trimAtTiePoint(*i1Iter, i, &CandidateInterval::b, &CandidateInterval::a);

    if (i.a == i.b)
        return std::set<CandidateInterval>::iterator();

    updateCAndBeta(i);
    auto result = edgeIntervals.emplace(std::forward<CandidateInterval>(i)).first;
    if (i.a == 0)
        eventQueue.insert(Event(i.d + std::sqrt((i.halfedge->vertex()->point() - i.rBar).squared_length()), i.halfedge->vertex()));
    if (i.b == 1)
        eventQueue.insert(Event(i.d + std::sqrt((endVertex(i.halfedge)->point() - i.rBar).squared_length()), endVertex(i.halfedge)));
    // FIXME: Don't always insert c... maybe?
    eventQueue.insert(Event(i.d + std::sqrt((i.frontierPoint() - i.rBar).squared_length()), result));
    return result;
}

static float fractionFromIntersection(CGAL::Point_3<Kernel> intersection, CGAL::Point_3<Kernel> point, CGAL::Vector_3<Kernel> vector) {
    return std::sqrt((intersection - point).squared_length()) / std::sqrt(vector.squared_length());
}

boost::optional<CandidateInterval> ModelPreprocessor::project(const CandidateInterval& i, Polyhedron::Halfedge_const_handle ei) const {
    auto intervalHalfedge = i.halfedge;
    auto halfedgePoint = intervalHalfedge->vertex()->point();
    auto halfedgeVector = endVertex(intervalHalfedge)->point() - intervalHalfedge->vertex()->point();
    auto eiEndVertex = endVertex(ei);
    auto eiVector = eiEndVertex->point() - ei->vertex()->point();
    auto p1 = intervalHalfedge->vertex()->point() + i.a * halfedgeVector;
    auto p2 = intervalHalfedge->vertex()->point() + i.b * halfedgeVector;

    // rotate unwrappedRoot into the plane of ei->facet()->plane()
    auto unfoldedRoot = i.rBar;
    float rotationAngle = angle(intervalHalfedge->facet()->plane().orthogonal_vector(), ei->facet()->plane().orthogonal_vector());
    glm::mat4 transformation;
    transformation *= glm::translate(-glm::vec3(halfedgePoint.x(), halfedgePoint.y(), halfedgePoint.z()));
    transformation *= glm::rotate(rotationAngle, glm::vec3(halfedgeVector.x(), halfedgeVector.y(), halfedgeVector.z()));
    transformation *= glm::translate(glm::vec3(halfedgePoint.x(), halfedgePoint.y(), halfedgePoint.z()));
    assert(transformation[0][3] == 0 && transformation[1][3] == 0 && transformation[2][3] == 0);
    CGAL::Aff_transformation_3<Kernel> cgalTransformation(transformation[0][0], transformation[1][0], transformation[2][0], transformation[3][0], transformation[0][1], transformation[1][1], transformation[2][1], transformation[3][1], transformation[0][2], transformation[1][2], transformation[2][2], transformation[3][2]);
    unfoldedRoot = cgalTransformation.transform(unfoldedRoot);

    auto intersection = CGAL::intersection(CGAL::Ray_3<Kernel>(unfoldedRoot, p1), CGAL::Ray_3<Kernel>(ei->vertex()->point(), endVertex(ei)->point()));
    assert(intersection);
    auto a = fractionFromIntersection(boost::get<CGAL::Point_3<Kernel>>(*intersection), ei->vertex()->point(), eiVector);
    intersection = CGAL::intersection(CGAL::Ray_3<Kernel>(unfoldedRoot, p2), CGAL::Ray_3<Kernel>(ei->vertex()->point(), endVertex(ei)->point()));
    assert(intersection);
    auto b = fractionFromIntersection(boost::get<CGAL::Point_3<Kernel>>(*intersection), ei->vertex()->point(), eiVector);

    if ((a > 1 && b > 1) || (a < 0 && b < 0))
        return boost::optional<CandidateInterval>();

    a = glm::clamp<float>(a, 0, 1);
    b = glm::clamp<float>(b, 0, 1);

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
        auto facetCirculator = i.halfedge->opposite()->facet_begin();
        assert(facetCirculator->opposite() == i.halfedge);
        for (++facetCirculator; facetCirculator != i.halfedge->facet_begin(); ++facetCirculator) {
            Polyhedron::Halfedge_const_handle halfedge = facetCirculator;
            if (boost::optional<CandidateInterval> ii = modelPreprocessor.project(i, halfedge))
                modelPreprocessor.insertInterval(std::move(*ii), i.rBar, modelPreprocessor.halfedgeIntervalMap.emplace(halfedge, std::set<CandidateInterval>()).first->second);
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

void ModelPreprocessor::preprocessModel() {
    for (const auto& initialPoint : initialPoints) {
        auto facetCirculator = initialPoint.facet->facet_begin();
        do {
            auto halfedge = facetCirculator;
            
            CandidateInterval candidateInterval = createCandidateIntervalOverEntireEdge(halfedge, initialPoint.point);
            auto frontierPoint = candidateInterval.frontierPoint();

            auto insertedInterval = insertInterval(std::move(candidateInterval), initialPoint.point, halfedgeIntervalMap.emplace(halfedge, std::set<CandidateInterval>()).first->second);
            if (insertedInterval != std::set<CandidateInterval>::iterator())
                eventQueue.emplace((frontierPoint - initialPoint.point).squared_length(), insertedInterval);
            eventQueue.emplace((halfedge->vertex()->point() - initialPoint.point).squared_length(), halfedge->vertex());
            ++facetCirculator;
        } while (facetCirculator != initialPoint.facet->facet_begin());
    }

    while (!eventQueue.empty()) {
        auto eventIter = eventQueue.begin();
        const auto& event = permanentLabels.emplace(eventIter->eventData, eventIter->label).first->first;
        boost::apply_visitor(EventLoopEventVisitor(*this), *event);
        eventQueue.erase(eventIter);
    }
}

void preprocessModel(const Polyhedron& polyhedron, const std::vector<InitialPoint>& initialPoints) {
    ModelPreprocessor(polyhedron, initialPoints).preprocessModel();
}
