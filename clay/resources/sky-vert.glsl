varying vec3 worldNormal;

void main()
{
  worldNormal = gl_Normal;
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
