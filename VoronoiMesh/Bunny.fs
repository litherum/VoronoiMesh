#version 410

layout(location = 0) out vec4 color;

void main(void) {
    color = vec4(gl_PrimitiveID / 100000.0, 0.0, 0.0, 1.0);
}

