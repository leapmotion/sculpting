uniform sampler2D preview_texture;
uniform float centerX;
uniform float centerY;
uniform float sizeX;
uniform float sizeY;
uniform float alpha;

void main(void)
{
  vec2 screenPos = gl_FragCoord.xy;

  float ratioX = (screenPos.x - centerX)/sizeX;
  float ratioY = (screenPos.y - centerY)/sizeY;

  ratioX = clamp(ratioX+0.5, 0.0, 1.0);
  ratioY = 1.0 - clamp(ratioY+0.5, 0.0, 1.0);

  vec3 color = texture2D(preview_texture, vec2(ratioX, ratioY)).rgb;

  gl_FragColor = vec4(color, alpha);
}
