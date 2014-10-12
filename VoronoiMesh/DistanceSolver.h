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

void preprocessModel(const Polyhedron&, const std::vector<InitialPoint>& initialPoints);

#endif /* defined(__VoronoiMesh__DistanceSolver__) */
