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

enum class PredecessorType {
    CandidateInterval,
    Vertex
};

struct CandidateInterval;

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

private:
    struct FrontierPointVisitor : boost::static_visitor<CGAL::Point_3<Kernel>> {
        CGAL::Point_3<Kernel> operator()(const CGAL::Point_3<Kernel>& point) const {
            return point;
        }

        CGAL::Point_3<Kernel> operator()(Polyhedron::Vertex_const_handle handle) const {
            return handle->point();
        }
    };

public:
    CGAL::Point_3<Kernel> frontierPoint() const {
        return boost::apply_visitor(FrontierPointVisitor(), c);
    }

    // Extent
    float a;
    float b;
    Polyhedron::Halfedge_const_handle halfedge; // Edge-face pair
    CGAL::Point_3<Kernel> r; // Root
    CGAL::Point_3<Kernel> rBar; // Unfolded root
    float d; // Depth
    boost::variant<CGAL::Point_3<Kernel>, Polyhedron::Vertex_const_handle> c; // Frontier point
    CGAL::Point_3<Kernel> beta; // Access point
    bool operator<(const CandidateInterval& other) const {
        return a < other.a;
    }
};

struct EventType {
    virtual ~EventType() { }
};

struct CandidateIntervalEvent : public EventType {
    CandidateIntervalEvent(const std::set<CandidateInterval>::iterator& candidateInterval)
        : candidateInterval(candidateInterval)
    {
    }
    const std::set<CandidateInterval>::iterator candidateInterval;
};

struct VertexEvent : public EventType {
    VertexEvent(const Polyhedron::Vertex_const_handle& vertex)
        : vertex(vertex)
    {
    }
    const Polyhedron::Vertex_const_handle vertex;
};

struct Event {
    Event(float label, const Polyhedron::Vertex_const_handle& vertex)
        : label(label)
        , eventType(new VertexEvent(vertex))
    {
    }
    Event(float label, const std::set<CandidateInterval>::iterator& candidateInterval)
        : label(label)
        , eventType(new CandidateIntervalEvent(candidateInterval))
    {
    }
    bool operator<(const Event& other) const {
        return label < other.label;
    }

    float label;
    std::shared_ptr<EventType> eventType;
};

class ModelPreprocessor {
public:
    ModelPreprocessor(const Polyhedron&, const std::vector<InitialPoint>&);
    void preprocessModel();
private:
    class PropagateVisitor;

    std::set<CandidateInterval>::iterator insertInterval(CandidateInterval&&, const CGAL::Point_3<Kernel>& c, std::set<CandidateInterval>& edgeIntervals);
    void deleteInterval(std::set<CandidateInterval>::iterator, std::set<CandidateInterval>& edgeIntervals);
    void propagate(const std::set<CandidateInterval>::iterator);
    boost::optional<CandidateInterval> project(const CandidateInterval& i, Polyhedron::Halfedge_const_handle ii) const;

    const std::vector<InitialPoint>& initialPoints;
    std::map<Polyhedron::Halfedge_const_handle, std::set<CandidateInterval>> halfedgeIntervalMap;
    std::set<Event> eventQueue;
    std::map<std::shared_ptr<EventType>, float> permanentLabels;
};


ModelPreprocessor::ModelPreprocessor(const Polyhedron&, const std::vector<InitialPoint>& initialPoints)
    : initialPoints(initialPoints)
{
}

void ModelPreprocessor::deleteInterval(std::set<CandidateInterval>::iterator item, std::set<CandidateInterval>& edgeIntervals) {
    if (eventQueue.size() > 1) {
        auto trailingIter = eventQueue.begin();
        std::set<Event>::iterator leadingIter = trailingIter;
        for (++leadingIter; leadingIter != eventQueue.end(); ++trailingIter, ++leadingIter) {
            while (leadingIter != eventQueue.end()) {
                if (auto* candidateInterval = dynamic_cast<CandidateIntervalEvent*>(leadingIter->eventType.get())) {
                    if (candidateInterval->candidateInterval != item)
                        continue;
                    eventQueue.erase(leadingIter);
                    leadingIter = trailingIter;
                    ++leadingIter;
                }
            }
            if (leadingIter == eventQueue.end())
                break;
        }
    }
    if (!eventQueue.empty()) {
        auto beginningItem = eventQueue.begin();
        if (auto* candidateInterval = dynamic_cast<CandidateIntervalEvent*>(beginningItem->eventType.get())) {
            if (candidateInterval->candidateInterval == item)
                eventQueue.erase(beginningItem);
        }
    }

    edgeIntervals.erase(item);
}

std::set<CandidateInterval>::iterator ModelPreprocessor::insertInterval(CandidateInterval&& i, const CGAL::Point_3<Kernel>& c, std::set<CandidateInterval>& edgeIntervals) {
    // FIXME: Write this function
    while (!edgeIntervals.empty())
        deleteInterval(edgeIntervals.begin(), edgeIntervals);
    return edgeIntervals.emplace(std::forward<CandidateInterval>(i)).first;
}

static Polyhedron::Vertex_const_handle endVertex(Polyhedron::Halfedge_const_handle halfedge) {
    auto facetCirculator = halfedge->facet_begin();
    assert(Polyhedron::Halfedge_const_handle(facetCirculator) == halfedge);
    ++facetCirculator;
    return facetCirculator->vertex();
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

static float fractionFromIntersection(CGAL::Point_3<Kernel> intersection, CGAL::Point_3<Kernel> point, CGAL::Vector_3<Kernel> vector) {
    return std::sqrt((intersection - point).squared_length()) / std::sqrt(vector.squared_length());
}

static float calculateFrontierPointFrac(Polyhedron::Halfedge_const_handle ei, float a, float b, CGAL::Point_3<Kernel> unfoldedRoot) {
    auto eiEndVertexPoint = endVertex(ei)->point();
    auto projected = CGAL::Line_3<Kernel>(ei->vertex()->point(), eiEndVertexPoint).projection(unfoldedRoot);
    auto frac = std::sqrt((projected - ei->vertex()->point()).squared_length()) / std::sqrt((eiEndVertexPoint - ei->vertex()->point()).squared_length());
    return glm::clamp(frac, a, b);
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
    transformation *= glm::rotate(rotationAngle, glm::vec3(halfedgeVector.x(), halfedgeVector.y(), halfedgeVector.y()));
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

    return CandidateInterval(a, b, ei, i.r, unfoldedRoot, i.d, frontierPoint, calculateAccessPoint(ei, frontierPointFrac, unfoldedRoot, i.beta));
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
    return of * onto / onto.squared_length();
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
            boost::optional<CandidateInterval> ii = modelPreprocessor.project(i, halfedge);
            if (ii)
                modelPreprocessor.insertInterval(std::move(*ii), c, modelPreprocessor.halfedgeIntervalMap.emplace(halfedge, std::set<CandidateInterval>()).first->second);
        }
    }

    void operator()(Polyhedron::Vertex_const_handle c) const {
        for (auto halfedge : oppositeEdgesThatNeedCandidateIntervals(c, i.beta)) {
            // FIXME: Second argument is probably wrong
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

void ModelPreprocessor::preprocessModel() {
    for (const auto& initialPoint : initialPoints) {
        auto facetCirculator = initialPoint.facet->facet_begin();
        do {
            auto halfedge = facetCirculator;
            
            CandidateInterval candidateInterval = createCandidateIntervalOverEntireEdge(halfedge, initialPoint.point);
            auto frontierPoint = candidateInterval.frontierPoint();

            auto insertedInterval = insertInterval(std::move(candidateInterval), initialPoint.point, halfedgeIntervalMap.emplace(halfedge, std::set<CandidateInterval>()).first->second);
            eventQueue.emplace((frontierPoint - initialPoint.point).squared_length(), insertedInterval);
            eventQueue.emplace((halfedge->vertex()->point() - initialPoint.point).squared_length(), halfedge->vertex());
            ++facetCirculator;
        } while (facetCirculator != initialPoint.facet->facet_begin());
    }

    while (!eventQueue.empty()) {
        auto eventIter = eventQueue.begin();
        const auto& event = permanentLabels.emplace(eventIter->eventType, eventIter->label).first->first;
        if (auto* candidateInterval = dynamic_cast<CandidateIntervalEvent*>(event.get()))
            propagate(candidateInterval->candidateInterval);
        eventQueue.erase(eventIter);
    }
}

void preprocessModel(const Polyhedron& polyhedron, const std::vector<InitialPoint>& initialPoints) {
    ModelPreprocessor(polyhedron, initialPoints).preprocessModel();
}
