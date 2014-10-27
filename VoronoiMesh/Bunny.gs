#version 410

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

uniform mat4 mvpMatrix;

uniform float dummy;
layout(std140) uniform DataStructureBlock {
    vec4 dataStructure[4096];
};

flat out vec4 edgeData[3];
out vec4 fs_position;

void main() {
    // Look inside dataStructure and populate edgeData from it
    for (uint i = 0; i < 3; ++i)
        edgeData[i] = gl_in[i].gl_Position;

    vec3 n = normalize(cross(gl_in[1].gl_Position.xyz - gl_in[0].gl_Position.xyz, gl_in[2].gl_Position.xyz - gl_in[0].gl_Position.xyz));
    for (uint i = 0; i < 3; ++i) {
        gl_Position = mvpMatrix * (gl_in[i].gl_Position + vec4((0.008 + dummy + dataStructure[352].y) * n, 0.0));
        gl_PrimitiveID = gl_PrimitiveIDIn;
        fs_position = gl_in[i].gl_Position;
        EmitVertex();
    }
    EndPrimitive();
}