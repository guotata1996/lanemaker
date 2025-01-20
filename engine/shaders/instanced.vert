#version 330

// GLSL version 3.3

uniform mat4 worldToView;            // parameter: the camera matrix

layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texCoordinate;

layout(location = 2) in vec3 instanceColor;
layout(location = 3) in vec4 instanceTransRow0;
layout(location = 4) in vec4 instanceTransRow1;
layout(location = 5) in vec4 instanceTransRow2;
layout(location = 6) in vec4 instanceTransRow3;

out vec4 instanceColor4;                    // output: computed fragmentation color
out vec2 texc;

void main() {
  // Mind multiplication order for matrixes
  gl_Position = worldToView * mat4(instanceTransRow0, instanceTransRow1, instanceTransRow2, instanceTransRow3) * vec4(position, 1.0);
  instanceColor4 = vec4(instanceColor, 1.0);
  texc = texCoordinate;
}


