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
  int objectID_I = int(round(objectID));
  float objFlag = objectID_I == -1 ? 0 : texelFetch(objectInfo, objectID_I, 0).r;
  int objFlag_I = int(round(objFlag * 16));

  vec3 adjustedPosition = position;
  
  if ((objFlag_I & 1) == 0)
  {
    fragColor = vec4(color, 1.0);
  }
  else
  {
    // highlight
    fragColor = vec4(color * 1.5, 1.0);
    adjustedPosition.z = position.z + 0.02;
  }
  
  if ((objFlag_I & 4) != 0)
  {
    // green
    fragColor = vec4(fragColor * vec4(0.5, 2, 0.5, 1));
    adjustedPosition.z = position.z + 0.02;
  }

  if ((objFlag_I & 2) == 0)
  {
    // hidden
    gl_Position = worldToView * vec4(adjustedPosition, 1.0);
  }
  else
  {
    gl_Position = vec4(0,0,0,0);
  }
}
