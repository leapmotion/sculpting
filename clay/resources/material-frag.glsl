#define MAX_LIGHTS 10

uniform vec3 campos;
uniform samplerCube irradiance;
uniform samplerCube radiance;

uniform float ambientFactor;
uniform float diffuseFactor;
uniform float reflectionFactor;
uniform vec3 surfaceColor;
uniform bool useRefraction;

varying vec3 worldPosition; 
varying vec3 worldNormal;
uniform float reflectionBias;
uniform float refractionBias;
uniform float refractionIndex;

uniform int numLights;
uniform vec3 lightPositions[MAX_LIGHTS];
uniform float lightWeights[MAX_LIGHTS];
uniform vec3 lightColor;
uniform float lightExponent;
uniform float lightRadius;
uniform float alphaMult;

void main()
{
	float alpha = 1.0;
	vec3 normal = normalize(worldNormal);
	vec3 eyedir = normalize(worldPosition-campos);

	vec3 reflectray = normalize(reflect(eyedir, normal));
	vec3 refractray = normalize(refract(eyedir, normal, refractionIndex));

	vec3 ambientcolor = vec3(ambientFactor);
	vec3 diffusecolor = textureCube(irradiance, normal).rgb * diffuseFactor;
	vec3 reflectcolor = textureCube(radiance, reflectray, reflectionBias).rgb * reflectionFactor;
	
	for (int i=0; i<numLights && i<MAX_LIGHTS; i++)
	{
		float lightdist = length(lightPositions[i] - worldPosition);
		vec3 lightdir = (lightPositions[i] - worldPosition)/lightdist;
		float d = clamp(dot(lightdir, normal), 0.0, 1.0);
		vec3 r = normalize(reflect(lightdir, normal));
		float s = pow(clamp(dot(r, eyedir), 0.0, 1.0), lightExponent);
		float falloff = clamp(1.0 - lightdist/lightRadius, 0.0, 1.0);
		diffusecolor += lightWeights[i]*falloff*d*lightColor;
		reflectcolor += lightWeights[i]*falloff*reflectionFactor*s*lightColor;
	}

	vec3 blend;
	if (useRefraction)
	{
		vec3 refractcolor = (1.0-diffuseFactor)*textureCube(radiance, refractray, refractionBias).rgb;
		blend = mix(mix(reflectcolor, refractcolor, dot(normal, -eyedir)), reflectcolor, diffuseFactor);
	}
	else
	{
		alpha = alphaMult*dot(normal, -eyedir);
		blend = mix((1.0-alpha)*reflectcolor, reflectcolor, diffuseFactor);
	}

	gl_FragColor = vec4((ambientcolor+diffusecolor)*surfaceColor + blend, alpha);
}
