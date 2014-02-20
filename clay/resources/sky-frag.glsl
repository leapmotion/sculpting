uniform samplerCube cubemap;
varying vec3 worldNormal;

void main()
{
  vec3 normalised_normal = normalize(worldNormal);
  vec3 color = textureCube(cubemap,normalised_normal).rgb;
  color = clamp(color, vec3(0.0), vec3(10.0));
  gl_FragColor = vec4(color,1.0);
}
