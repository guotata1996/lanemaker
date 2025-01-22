#version 330 core

uniform sampler2D tex;
in vec2 texc;
in vec4 instanceColor4;
out vec4 finalColor;

void main() {
  vec4 texColor = texture(tex, texc);
  finalColor = texColor * instanceColor4;
}
