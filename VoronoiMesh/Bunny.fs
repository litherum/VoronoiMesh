#version 410

layout(location = 0) out vec4 color;

flat in vec4 fs_color;

void main(void) {
    color = fs_color;
}

