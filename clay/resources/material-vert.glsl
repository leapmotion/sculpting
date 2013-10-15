attribute vec4 vertex;
attribute vec3 normal;

uniform mat4 transform;
uniform mat4 transformit;

varying vec3 worldPosition;
varying vec3 worldNormal;

void main()
{
	vec4 wpos       = transform * vertex;
	worldPosition   = wpos.xyz;
	worldNormal     = (transformit * vec4(normal,0.0)).xyz;
	gl_Position			= gl_ModelViewProjectionMatrix * wpos;
}
