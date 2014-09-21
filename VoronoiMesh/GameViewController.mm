//
//  GameViewController.m
//  VoronoiMesh
//
//  Created by Litherum on 9/20/14.
//  Copyright (c) 2014 Litherum. All rights reserved.
//

#import "GameViewController.h"

#import <CGAL/HalfedgeDS_vector.h>
#import <CGAL/Polyhedron_3.h>
#import <CGAL/Polyhedron_incremental_builder_3.h>
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconditional-uninitialized"
#import <CGAL/Simple_cartesian.h>
#pragma clang diagnostic pop
#import <ply.h>
#import <boost/functional/hash.hpp>
#import <cassert>
#import <vector>

typedef CGAL::Simple_cartesian<float> Kernel;
typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_3, CGAL::HalfedgeDS_vector> Polyhedron;

@implementation GameViewController

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

-(void)awakeFromNib
{
    Polyhedron model = buildModel(std::string([[[NSBundle mainBundle] pathForResource:@"bun_zipper" ofType:@"ply"] UTF8String]));
    // create a new scene
    SCNScene *scene = [SCNScene scene];

    // create and add a camera to the scene
    SCNNode *cameraNode = [SCNNode node];
    cameraNode.camera = [SCNCamera camera];
    [scene.rootNode addChildNode:cameraNode];
    
    // place the camera
    cameraNode.position = SCNVector3Make(0, 0, 5);
    
    // create and add a light to the scene
    SCNNode *lightNode = [SCNNode node];
    lightNode.light = [SCNLight light];
    lightNode.light.type = SCNLightTypeOmni;
    lightNode.position = SCNVector3Make(2, 2, 5);
    [scene.rootNode addChildNode:lightNode];
    
    // create and add an ambient light to the scene
    SCNNode *ambientLightNode = [SCNNode node];
    ambientLightNode.light = [SCNLight light];
    ambientLightNode.light.type = SCNLightTypeAmbient;
    ambientLightNode.light.color = [NSColor darkGrayColor];
    [scene.rootNode addChildNode:ambientLightNode];

    std::vector<SCNVector3> vertexPositions;
    std::vector<SCNVector3> vertexNormals;
    for (auto vertexIterator = model.vertices_begin(); vertexIterator != model.vertices_end(); ++vertexIterator) {
        auto& point = vertexIterator->point();
        vertexPositions.push_back(SCNVector3Make(point.x(), point.y(), point.z()));

        CGAL::Vector_3<Kernel> averagedNormal(0, 0, 0);
        int count = 0;
        auto vertexCirculator = vertexIterator->vertex_begin();
        do {
            if (vertexCirculator == Polyhedron::Halfedge_around_vertex_circulator())
                break;
            auto facet = vertexCirculator->facet();
            if (facet != Polyhedron::Face_handle()) {
                auto normal = facet->plane().orthogonal_vector();
                normal = normal / std::sqrt(normal.squared_length());
                averagedNormal = averagedNormal + normal;
                ++count;
            }
            ++vertexCirculator;
        } while (vertexCirculator != vertexIterator->vertex_begin());
        averagedNormal = averagedNormal / count;
        vertexNormals.push_back(SCNVector3Make(averagedNormal.x(), averagedNormal.y(), averagedNormal.z()));
    }

    std::vector<int> vertexIndices;
    for (auto facetIterator = model.facets_begin(); facetIterator != model.facets_end(); ++facetIterator) {
        auto facetCirculator = facetIterator->facet_begin();
        do {
            vertexIndices.push_back(static_cast<int>(facetCirculator->vertex() - model.vertices_begin()));
            ++facetCirculator;
        } while (facetCirculator != facetIterator->facet_begin());
    }
    SCNGeometry *bunnyGeometry = [SCNGeometry
                                  geometryWithSources:[NSArray arrayWithObjects:[SCNGeometrySource geometrySourceWithVertices:vertexPositions.data()
                                                                                                                        count:vertexPositions.size()],
                                                                                [SCNGeometrySource geometrySourceWithNormals:vertexNormals.data()
                                                                                                                       count:vertexNormals.size()], nil]
                                  elements:[NSArray arrayWithObjects:[SCNGeometryElement geometryElementWithData:[NSData dataWithBytes:vertexIndices.data() length:vertexIndices.size() * sizeof(int)]
                                                                                                   primitiveType:SCNGeometryPrimitiveTypeTriangles
                                                                                                  primitiveCount:model.size_of_facets()
                                                                                                   bytesPerIndex:sizeof(int)], nil]];

    SCNMaterial *redMaterial = [SCNMaterial material];
    redMaterial.ambient.contents = [NSColor darkGrayColor];
    redMaterial.diffuse.contents = [NSColor blueColor];
    redMaterial.specular.contents = [NSColor darkGrayColor];
    redMaterial.shininess = 0.1;
    redMaterial.lightingModelName = SCNLightingModelPhong;
    redMaterial.doubleSided = YES;
    bunnyGeometry.materials = [NSArray arrayWithObjects:redMaterial, nil];

    SCNNode *bunnyNode = [SCNNode node];
    bunnyNode.geometry = bunnyGeometry;
    bunnyNode.scale = SCNVector3Make(20, 20, 20);
    bunnyNode.position = SCNVector3Make(0, -2, 0);
    [scene.rootNode addChildNode:bunnyNode];

    // animate the 3d object
    CABasicAnimation *animation = [CABasicAnimation animationWithKeyPath:@"rotation"];
    animation.toValue = [NSValue valueWithSCNVector4:SCNVector4Make(0, 1, 0, M_PI*2)];
    animation.duration = 9;
    animation.repeatCount = MAXFLOAT; //repeat forever
    [bunnyNode addAnimation:animation forKey:nil];

    // set the scene to the view
    self.gameView.scene = scene;
    
    // allows the user to manipulate the camera
    self.gameView.allowsCameraControl = YES;
    
    // show statistics such as fps and timing information
    self.gameView.showsStatistics = YES;
    
    // configure the view
    self.gameView.backgroundColor = [NSColor blackColor];  
}

@end
