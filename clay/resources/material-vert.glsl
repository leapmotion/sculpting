attribute vec4 vertex;
attribute vec3 normal;
attribute vec3 color;

uniform mat4 transform;
uniform mat4 transformit;

varying vec3 worldPosition;
varying vec3 worldNormal;
varying vec3 vertexColor;

void main()
{
  vec4 wpos       = transform * vertex;
  worldPosition   = wpos.xyz;
  worldNormal     = (transformit * vec4(normal,0.0)).xyz;
  vertexColor     = color;
  gl_Position     = gl_ModelViewProjectionMatrix * wpos;
}
