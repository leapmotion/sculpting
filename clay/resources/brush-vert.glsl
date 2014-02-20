varying vec3 worldPosition;
varying vec3 worldNormal;
varying vec3 vertexColor;

void main()
{
  vec4 wpos       = gl_Vertex;
  worldPosition   = wpos.xyz;
  worldNormal     = gl_Normal;
  vertexColor     = vec3(1.0);
  gl_Position     = gl_ModelViewProjectionMatrix * wpos;
}
