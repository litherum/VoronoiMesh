#version 410

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

uniform mat4 mvpMatrix;

void main() {
    vec3 n = normalize(cross(gl_in[1].gl_Position.xyz - gl_in[0].gl_Position.xyz, gl_in[2].gl_Position.xyz - gl_in[0].gl_Position.xyz));
    for (uint i = 0; i < 3; ++i) {
        gl_Position = mvpMatrix * (gl_in[i].gl_Position + vec4(0.005 * n, 0.0));
        EmitVertex();
    }
    EndPrimitive();
}