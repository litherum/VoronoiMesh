//
//  GameViewController.m
//  VoronoiMesh
//
//  Created by Litherum on 9/20/14.
//  Copyright (c) 2014 Litherum. All rights reserved.
//

#import "GameViewController.h"

#import "DistanceSolver.h"
#import "ModelBuilder.h"

#import <vector>
#import <OpenGL/gl3.h>
#import <GLKit/GLKMath.h>

@implementation GameViewController

static CGAL::Point_3<Kernel> facetCenter(Polyhedron::Facet_const_handle facet) {
    CGAL::Vector_3<Kernel> result(0, 0, 0);
    auto circulator = facet->facet_begin();
    do {
        result = result + (circulator->vertex()->point() - CGAL::Point_3<Kernel>(0, 0, 0));
        ++circulator;
    } while (circulator != facet->facet_begin());
    return CGAL::Point_3<Kernel>(0, 0, 0) + (result / facet->facet_degree());
}

- (void)program:(SCNProgram *)program handleError:(NSError *)error
{
    NSLog(@"Program error: %@", error);
}

-(void)awakeFromNib
{
    const Polyhedron model = buildModel(std::string([[[NSBundle mainBundle] pathForResource:@"bun_zipper" ofType:@"ply"] UTF8String]));

    {
        std::vector<InitialPoint> initialPoints;
        Polyhedron::Facet_const_handle facet = model.facets_begin();
        initialPoints.push_back(InitialPoint(facet, facetCenter(facet)));
        //preprocessModel(model, initialPoints);
    }

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
        const auto& point = vertexIterator->point();
        vertexPositions.push_back(SCNVector3Make(point.x(), point.y(), point.z()));

        CGAL::Vector_3<Kernel> averagedNormal(0, 0, 0);
        int count = 0;
        auto vertexCirculator = vertexIterator->vertex_begin();
        do {
            if (vertexCirculator == Polyhedron::Halfedge_around_vertex_circulator())
                break;
            const auto& facet = vertexCirculator->facet();
            if (facet == Polyhedron::Face_handle()) {
                ++vertexCirculator;
                continue;
            }
            auto normal = facet->plane().orthogonal_vector();
            normal = normal / std::sqrt(normal.squared_length());
            averagedNormal = averagedNormal + normal;
            ++count;
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
                                  geometryWithSources:@[[SCNGeometrySource geometrySourceWithVertices:vertexPositions.data()
                                                                                                count:vertexPositions.size()],
                                                        [SCNGeometrySource geometrySourceWithNormals:vertexNormals.data()
                                                                                               count:vertexNormals.size()]]
                                  elements:@[[SCNGeometryElement geometryElementWithData:[NSData dataWithBytes:vertexIndices.data() length:vertexIndices.size() * sizeof(int)]
                                                                           primitiveType:SCNGeometryPrimitiveTypeTriangles
                                                                          primitiveCount:vertexIndices.size() / 3
                                                                           bytesPerIndex:sizeof(int)]]];

    SCNProgram *bunnyProgram = [SCNProgram program];
    NSURL *vertexShaderURL = [[NSBundle mainBundle] URLForResource:@"Bunny" withExtension:@"vs"];
    NSURL *geometryShaderURL = [[NSBundle mainBundle] URLForResource:@"Bunny" withExtension:@"gs"];
    NSURL *fragmentShaderURL = [[NSBundle mainBundle] URLForResource:@"Bunny" withExtension:@"fs"];
    bunnyProgram.vertexShader = [NSString stringWithContentsOfURL:vertexShaderURL encoding:NSASCIIStringEncoding error:NULL];
    bunnyProgram.geometryShader = [NSString stringWithContentsOfURL:geometryShaderURL encoding:NSASCIIStringEncoding error:NULL];
    bunnyProgram.fragmentShader = [NSString stringWithContentsOfURL:fragmentShaderURL encoding:NSASCIIStringEncoding error:NULL];

    [bunnyProgram setSemantic:SCNGeometrySourceSemanticVertex forSymbol:@"position" options:nil];
    [bunnyProgram setSemantic:SCNModelViewProjectionTransform forSymbol:@"mvpMatrix" options:nil];

    bunnyProgram.delegate = self;

    SCNMaterial *bunnyMaterial = [SCNMaterial material];
    bunnyMaterial.program = bunnyProgram;

    bunnyGeometry.firstMaterial = bunnyMaterial;

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

    const NSOpenGLPixelFormatAttribute attributes[] = {NSOpenGLPFADoubleBuffer, NSOpenGLPFADepthSize, 32, NSOpenGLPFAOpenGLProfile, NSOpenGLProfileVersion3_2Core, 0};
    self.gameView.pixelFormat = [[NSOpenGLPixelFormat alloc] initWithAttributes:attributes];

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
