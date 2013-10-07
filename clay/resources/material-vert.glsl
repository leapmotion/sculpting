uniform mat4 transform;
uniform mat4 transformit;

varying vec3 worldPosition;
varying vec3 worldNormal;

void main()
{
	vec4 wpos       = transform * gl_Vertex;
	vec3 wnrm       = (transformit * vec4(gl_Normal,0.0)).xyz;
	worldPosition   = wpos.xyz;
	worldNormal     = wnrm;
	gl_Position			= gl_ModelViewProjectionMatrix * wpos;
}
