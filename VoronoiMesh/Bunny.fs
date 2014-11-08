#version 410

layout(location = 0) out vec4 color;

flat in vec4 fs_color;

in vec4 fs_position;

void main(void) {
    color = fs_color;
}

