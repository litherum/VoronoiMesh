#version 410

layout(location = 0) out vec4 color;

flat in vec4 edgeData[3];

in vec4 fs_position;

void main(void) {
    float d1 = length(edgeData[0] - fs_position);
    float d2 = length(edgeData[1] - fs_position);
    float d3 = length(edgeData[2] - fs_position);
    float minimum = min(d1, min(d2, d3));
    float max1 = length(edgeData[0] - edgeData[1]);
    float max2 = length(edgeData[1] - edgeData[2]);
    float max3 = length(edgeData[2] - edgeData[0]);
    float maximum = max(max1, max(max2, max3));
    color = vec4(minimum / maximum, 0.0, 0.0, 1.0);
    //color = vec4(gl_PrimitiveID / 100000.0, 0.0, 0.0, 1.0);
}

