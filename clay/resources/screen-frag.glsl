uniform sampler2D color_texture;
uniform sampler2D bloom_texture;
uniform sampler2D depth_texture;
uniform float exposure;
uniform float bloom_strength;
uniform float contrast;
uniform float width;
uniform float height;
uniform float vignette_radius;
uniform float vignette_strength;

void main(void)
{
  vec3 color = texture2D(color_texture,gl_TexCoord[0].st).rgb;
  color += texture2D(bloom_texture, gl_TexCoord[0].st).rgb*bloom_strength;

  vec2 diff = (gl_FragCoord.xy - vec2(width/2.0, height/2.0))/vignette_radius;
  float vignette = dot(diff, diff);
  color *= (1.0 - (vignette_strength * vignette));

  color = 1.0 - exp2(-color * exposure);
  color = contrast*(color - vec3(0.5)) + vec3(0.5);
  vec3 final = color;

  gl_FragColor = vec4(final, 1.0); 
}
