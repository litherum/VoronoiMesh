//
//  ModelBuilder.h
//  VoronoiMesh
//
//  Created by Litherum on 9/21/14.
//  Copyright (c) 2014 Litherum. All rights reserved.
//

#ifndef __VoronoiMesh__ModelBuilder__
#define __VoronoiMesh__ModelBuilder__

#include <CGAL/HalfedgeDS_vector.h>
#include <CGAL/Polyhedron_3.h>
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconditional-uninitialized"
#include <CGAL/Simple_cartesian.h>
#pragma clang diagnostic pop

#include <string>

typedef CGAL::Simple_cartesian<float> Kernel;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_3, CGAL::HalfedgeDS_vector> Polyhedron;

Polyhedron buildModel(std::string filename);

#endif /* defined(__VoronoiMesh__ModelBuilder__) */
