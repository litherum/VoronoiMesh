#version 410

layout(location = 0) out vec4 color;

uniform float dummy;
layout(std140) uniform DataStructureBlock {
    vec4 somedata[4];
};

void main(void) {
    color = vec4(/*gl_PrimitiveID / 100000.0*/ 0.0, somedata[0].y, dummy, 1.0);
}

