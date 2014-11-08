#version 410

in vec4 vs_position;

out int gs_vertexID;

void main()
{
    gl_Position = vs_position;
    gs_vertexID = gl_VertexID;
}
