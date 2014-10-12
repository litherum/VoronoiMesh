//
//  ModelBuilder.cpp
//  VoronoiMesh
//
//  Created by Litherum on 9/21/14.
//  Copyright (c) 2014 Litherum. All rights reserved.
//

#include "ModelBuilder.h"

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <cassert>
#include <ply.h>

class BuildModel : public CGAL::Modifier_base<Polyhedron::HalfedgeDS> {
public:
    BuildModel(std::string filename)
    : m_filename(filename)
    {
        
    }
    
private:
    virtual void operator()(Polyhedron::HalfedgeDS& halfedgeDS) override {
        float vec[3];
        FaceList faceList;
        PlyProperty vertexProperties[] = {
            {const_cast<char*>("x"), PLY_START_TYPE, PLY_FLOAT,
                static_cast<int>(reinterpret_cast<char*>(&(vec[0])) - reinterpret_cast<char*>(&vec)),
                PLY_SCALAR, PLY_START_TYPE, PLY_START_TYPE, 0},
            {const_cast<char*>("y"), PLY_START_TYPE, PLY_FLOAT,
                static_cast<int>(reinterpret_cast<char*>(&(vec[1])) - reinterpret_cast<char*>(&vec)),
                PLY_SCALAR, PLY_START_TYPE, PLY_START_TYPE, 0},
            {const_cast<char*>("z"), PLY_START_TYPE, PLY_FLOAT,
                static_cast<int>(reinterpret_cast<char*>(&(vec[2])) - reinterpret_cast<char*>(&vec)),
                PLY_SCALAR, PLY_START_TYPE, PLY_START_TYPE, 0}
        };
        
        PlyProperty faceProperties[] = {
            {const_cast<char*>("vertex_indices"), PLY_START_TYPE, PLY_INT,
                static_cast<int>(reinterpret_cast<char*>(&(faceList.vertexIds)) - reinterpret_cast<char*>(&faceList)),
                1, PLY_START_TYPE, PLY_CHAR,
                static_cast<int>(reinterpret_cast<char*>(&(faceList.numVertices)) - reinterpret_cast<char*>(&faceList))}
        };
        
        typedef typename Polyhedron::HalfedgeDS::Vertex Vertex;
        typedef Vertex::Point Point;
        
        int nElems;
        char** elemNames;
        int fileType;
        float version;
        PlyFile* plyFile = ply_open_for_reading(const_cast<char*>(m_filename.c_str()), &nElems, &elemNames, &fileType, &version);
        assert(plyFile);
        
        CGAL::Polyhedron_incremental_builder_3<Polyhedron::HalfedgeDS> incrementalBuilder(halfedgeDS, true);
        int nProps;
        int numFacets;
        ply_get_element_description(plyFile, const_cast<char*>("vertex"), &nElems, &nProps);
        ply_get_element_description(plyFile, const_cast<char*>("face"), &numFacets, &nProps);
        incrementalBuilder.begin_surface(nElems, numFacets);
        
        ply_get_element_setup(plyFile, const_cast<char*>("vertex"), sizeof(vertexProperties) / sizeof(PlyProperty), vertexProperties);
        
        ply_get_element_description(plyFile, const_cast<char*>("vertex"), &nElems, &nProps);
        
        for (int i = 0; i < nElems; ++i) {
            ply_get_element(plyFile, vec);
            incrementalBuilder.add_vertex(Point(vec[0], vec[1], vec[2]));
        }
        
        // Faces
        ply_get_element_setup(plyFile, const_cast<char*>("face"), sizeof(faceProperties) / sizeof(PlyProperty), faceProperties);
        
        ply_get_element_description(plyFile, const_cast<char*>("face"), &nElems, &nProps);
        
        for (int i = 0; i < nElems; ++i) {
            ply_get_element(plyFile, &faceList);
            assert(faceList.numVertices == 3); // We only know how to render triangles
            
            if (incrementalBuilder.test_facet(faceList.vertexIds, faceList.vertexIds + faceList.numVertices)) {
                auto handle = incrementalBuilder.add_facet(faceList.vertexIds, faceList.vertexIds + faceList.numVertices);
                handle->facet()->plane() = Polyhedron::Plane_3(handle->vertex()->point(),
                                                               handle->next()->vertex()->point(),
                                                               handle->next()->next()->vertex()->point());
            }
        }
        
        incrementalBuilder.end_surface();
        ply_close(plyFile);
    }
    struct FaceList {
        int* vertexIds;
        char numVertices;
    };
    std::string m_filename;
};

Polyhedron buildModel(std::string filename) {
    Polyhedron polyhedron;
    BuildModel buildModel(filename);
    polyhedron.delegate(buildModel);
    return polyhedron;
}