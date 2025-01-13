#version 330

// GLSL version 3.3
// vertex shader

layout(location = 0) in vec3 position; // input:  attribute with index '0' with 3 elements per vertex
layout(location = 1) in vec3 color;    // input:  attribute with index '1' with 3 elements (=rgb) per vertex

layout(location = 2) in vec4 instanceTransRow0;
layout(location = 3) in vec4 instanceTransRow1;
layout(location = 4) in vec4 instanceTransRow2;
layout(location = 5) in vec4 instanceTransRow3;

out vec4 fragColor;                    // output: computed fragmentation color

uniform mat4 worldToView;            // parameter: the camera matrix

void main() {
  // Mind multiplication order for matrixes
  gl_Position = worldToView * mat4(instanceTransRow0, instanceTransRow1, instanceTransRow2, instanceTransRow3) * vec4(position, 1.0);
  fragColor = vec4(color, 1.0);
}


