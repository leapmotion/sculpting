#version 120

uniform sampler2D color_tex;
uniform vec2 sample_offset;
uniform float light_threshold;

float weights[11] = float[](0.0181588,
                            0.0408404,
                            0.0767134,
                            0.120345,
                            0.157675,
                            0.172534,
                            0.157675,
                            0.120345,
                            0.0767134,
                            0.0408404,
                            0.0181588);

const float BLOOM_MULT = 1.1;

void main() {
  vec3 sum = vec3(0.0, 0.0, 0.0);
  vec3 thresh = vec3(light_threshold);
  vec2 offset = vec2(0.0, 0.0);
  vec2 baseOffset = -5.0 * sample_offset;
  vec3 texValue;

  for (int s = 0; s < 11; ++s) {
    texValue = texture2D(color_tex, gl_TexCoord[0].st + baseOffset + offset).rgb;
    texValue = clamp(BLOOM_MULT*(texValue - thresh) + thresh, vec3(0), vec3(10));
    sum += texValue * weights[s];
    offset += sample_offset;
  }

  gl_FragColor.rgb = sum;
  gl_FragColor.a = 1.0;
}
