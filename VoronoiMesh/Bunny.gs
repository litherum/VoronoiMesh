#version 410

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

uniform mat4 mvpMatrix;

uniform float dummy;

uniform samplerBuffer dataStructureTexture;

in int gs_vertexID[];

flat out vec4 fs_color;
out vec4 fs_position;

void main() {
    fs_color = vec4(1.0, 0.0, 0.0, 1.0);

    int vertexDistancesTexels = floatBitsToInt(texelFetch(dataStructureTexture, 0).x) / 4;
    int intervalDistancesTexels = floatBitsToInt(texelFetch(dataStructureTexture, 1).x) / 4;

    int texelsPerVertexDistance = 2;
    int texelsPerIntervalDistance = 8;

    int vertexDistanceStartTexels = 2;
    int intervalDistanceStart = 2 + vertexDistancesTexels;

    // Look for vertices in our primitive
    for (int i = 0; i < vertexDistancesTexels / texelsPerVertexDistance; ++i) {
        uint vertex = floatBitsToUint(texelFetch(dataStructureTexture, vertexDistanceStartTexels + texelsPerVertexDistance * i).x);
        if (vertex == gs_vertexID[0] || vertex == gs_vertexID[1] || vertex == gs_vertexID[2]) {
            fs_color = vec4(0.0, 1.0, 0.0, 1.0);
        }
    }

/*
    // Look for intervals in our primitive
    for (int i = 0; i < intervalDistancesTexels / texelsPerIntervalDistance; ++i) {
        uint vertex1 = floatBitsToUint(texelFetch(dataStructureTexture, intervalDistanceStart + texelsPerIntervalDistance * i).x);
        uint vertex2 = floatBitsToUint(texelFetch(dataStructureTexture, intervalDistanceStart + texelsPerIntervalDistance * i + 1).x);
        if ((vertex1 == gs_vertexID[0] || vertex1 == gs_vertexID[1] || vertex1 == gs_vertexID[2]) ||
                ((vertex2 == gs_vertexID[0] || vertex2 == gs_vertexID[1] || vertex2 == gs_vertexID[2]))) {
            fs_color = vec4(0.0, 0.0, 1.0, 1.0);
        }
    }
*/

    vec3 n = normalize(cross(gl_in[1].gl_Position.xyz - gl_in[0].gl_Position.xyz, gl_in[2].gl_Position.xyz - gl_in[0].gl_Position.xyz));
    for (uint i = 0; i < 3; ++i) {
        gl_Position = mvpMatrix * (gl_in[i].gl_Position + vec4(dummy * n, 0.0));
        gl_PrimitiveID = gl_PrimitiveIDIn;
        fs_position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}