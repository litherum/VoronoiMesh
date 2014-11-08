//
//  DistanceSolver.h
//  VoronoiMesh
//
//  Created by Litherum on 9/21/14.
//  Copyright (c) 2014 Litherum. All rights reserved.
//

#ifndef __VoronoiMesh__DistanceSolver__
#define __VoronoiMesh__DistanceSolver__

#include "ModelBuilder.h"

#include <map>
#include <vector>

struct InitialPoint {
    InitialPoint(Polyhedron::Facet_const_handle facet, CGAL::Point_3<Kernel> point)
        : facet(facet)
        , point(point)
    {
    }
    Polyhedron::Facet_const_handle facet;
    CGAL::Point_3<Kernel> point;
};

struct IntervalDistancesValue {
    IntervalDistancesValue(float a, float b, CGAL::Point_3<Kernel> rBar, float d)
        : a(a)
        , b(b)
        , rBar(rBar)
        , d(d)
    {
    }
    float a;
    float b;
    CGAL::Point_3<Kernel> rBar;
    float d;
};

typedef std::map<Polyhedron::Vertex_const_handle, float> VertexDistances;
typedef std::multimap<Polyhedron::Halfedge_const_handle, IntervalDistancesValue> IntervalDistances;
typedef std::pair<VertexDistances, IntervalDistances> Distances;

Distances preprocessModel(const Polyhedron&, const std::vector<InitialPoint>& initialPoints);

#endif /* defined(__VoronoiMesh__DistanceSolver__) */
