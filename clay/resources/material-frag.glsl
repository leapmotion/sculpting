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
varying vec3 vertexColor;
uniform float reflectionBias;
uniform float refractionBias;
uniform float refractionIndex;

uniform int numLights;
uniform vec3 brushPositions[MAX_LIGHTS];
uniform float brushWeights[MAX_LIGHTS];
uniform float brushRadii[MAX_LIGHTS];
uniform vec3 lightColor;
uniform float lightExponent;
uniform float lightRadius;
uniform float alphaMult;

const float HIGHLIGHT_INTENSITY = 0.4;

void main()
{
  vec3 normal = normalize(worldNormal);
  vec3 eyedir = normalize(worldPosition-campos);

  vec3 reflectray = normalize(reflect(eyedir, normal));
  vec3 refractray = normalize(refract(eyedir, normal, refractionIndex));

  vec3 ambientcolor = vec3(ambientFactor);
  vec3 diffusecolor = (vertexColor * textureCube(irradiance, normal).rgb) * diffuseFactor;
  vec3 reflectcolor = textureCube(radiance, reflectray, reflectionBias).rgb * reflectionFactor;

  float highlightMult = 1.0;

  for (int i=0; i<numLights && i<MAX_LIGHTS; i++) {
    float lightdist = length(brushPositions[i] - worldPosition);
    vec3 lightdir = (brushPositions[i] - worldPosition)/lightdist;
    float distMult = (lightdist / brushRadii[i]);
    distMult = distMult * distMult;
    if (lightdist < brushRadii[i]) {
      highlightMult += distMult*HIGHLIGHT_INTENSITY*brushWeights[i];
    }

    if (i < numLights - 1) {
      float d = clamp(dot(lightdir, normal), 0.0, 1.0);
      vec3 r = normalize(reflect(lightdir, normal));
      float s = pow(clamp(dot(r, eyedir), 0.0, 1.0), lightExponent);
      float falloff = clamp(1.0 - lightdist/lightRadius, 0.0, 1.0);
      diffusecolor += brushWeights[i]*falloff*d*lightColor;
      reflectcolor += brushWeights[i]*falloff*reflectionFactor*s*lightColor;
    }
  }

  if (useRefraction) {
    vec3 refractcolor = highlightMult*(1.0-diffuseFactor)*vertexColor*textureCube(radiance, refractray, refractionBias).rgb;
    vec3 blend = mix(mix(reflectcolor, refractcolor, dot(normal, -eyedir)), reflectcolor, diffuseFactor);
    gl_FragColor = vec4((ambientcolor+diffusecolor)*surfaceColor + blend, 1.0);
    gl_FragColor.rgb *= highlightMult;
  } else {
    float alpha = alphaMult*dot(normal, -eyedir);
    gl_FragColor = vec4((ambientcolor+diffusecolor)*surfaceColor + reflectcolor, alpha);
  }

}
