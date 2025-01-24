#version 330

// GLSL version 3.3
// vertex shader

layout(location = 0) in vec3 position; // input:  attribute with index '0' with 3 elements per vertex
layout(location = 1) in vec3 color;    // input:  attribute with index '1' with 3 elements (=rgb) per vertex
layout(location = 2) in float objectID;
out vec4 fragColor;                    // output: computed fragmentation color

uniform mat4 worldToView;            // parameter: the camera matrix
uniform sampler1D objectInfo;

void main() {
  // Mind multiplication order for matrixes
  gl_Position = worldToView * vec4(position, 1.0);
  int objectID_I = int(round(objectID));
  float objFlag = objectID_I == -1 ? 0 : texelFetch(objectInfo, objectID_I, 0).r;
  objFlag = int(round(objFlag * 16));
  if (objFlag == 0)
  {
    fragColor = vec4(color, 1.0);
  }
  else if (objFlag == 1)
  {
    fragColor = vec4(color * 1.5, 1.0);
  }
  else if (objFlag == 2)
  {
    // Invisible
    gl_Position = vec4(0,0,0,0);
  }
  else if (objFlag == 3)
  {
    fragColor = vec4(color * vec3(1,0,0), 1.0);
  }
  else if (objFlag == 4)
  {
    fragColor = vec4(color * vec3(0,1,0), 1.0);
  }
  else
  {
    // Error
    fragColor = vec4(0,0,0,1);
  }
}


