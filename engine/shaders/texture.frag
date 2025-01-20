#version 330 core

uniform sampler2D texture;
in vec2 texc;
in vec4 instanceColor4;
out vec4 finalColor;

void main() {
  vec4 texColor = texture2D(texture, texc);
  finalColor = texColor * instanceColor4;
}
