#define MAX_METABALLS 30
#define POWER 8.0

uniform vec2 positions[MAX_METABALLS];
uniform float radii[MAX_METABALLS];
uniform vec4 colors[MAX_METABALLS];
uniform float weights[MAX_METABALLS];
uniform float ambients[MAX_METABALLS];
uniform int num;
uniform samplerCube radiance;
uniform samplerCube irradiance;
uniform mat3 normalTransform;
uniform float epsilon;
uniform float diffuseFactor;
uniform float reflectionFactor;

const float thresh_upper = 500.0;
const float thresh_lower = 0.0;
const float smooth_mult = 1.0/(thresh_upper - thresh_lower);
const vec3 eye_dir = vec3(0, 0, -1);
const float ambient_scale = 8.0;

void main(void)
{
  vec4 color = vec4(0.0);
  float sumW = 0.0;
  vec2 sumWVec = vec2(0.0);
  float sumG = 0.0;
  vec2 sumGVec = vec2(0.0);

  float epsilonSquared = epsilon*epsilon;

  // calculate total energy
  for (int i=0; i<num && i<MAX_METABALLS; i++)
  {
    float r = radii[i];
    vec2 diff = gl_FragCoord.xy - positions[i];

    // step 1
    float x = length(diff);
    vec2 xVec = diff/x;

    // step 2
    float rN = pow(r, POWER);
    float xN = pow(x, POWER);
    float xN_1 = pow(x, POWER-1.0);
    float sum = rN + xN;
    float term1 = rN / sum;
    float w = weights[i] * term1;
    float term2 = xN_1 / sum;
    vec2 wVec = -POWER * xVec * w * term2;

    // step 3
    float h = r*r - x*x;
    vec2 hVec = -2.0 * diff;

    // step 4
    float g = h * w;
    vec2 gVec = h*wVec + w*hVec;

    // accumulate
    sumW += w;
    sumWVec += wVec;
    sumG += g;
    sumGVec += gVec;
    color.rgb += (1.0 + ambient_scale*ambients[i]*ambients[i])*w*colors[i].rgb;
    color.a += w*colors[i].a;
  }
  color /= sumW;

  // step 5
  float f = sumG / sumW;
  vec2 fVec = (sumW*sumGVec - sumG*sumWVec) / (sumW * sumW);

  // step 6
  float mult = 0.0;
  float p = f + epsilonSquared;
  if (p >= thresh_lower)
  {
    if (p < thresh_upper)
    {
      mult = smoothstep(1.0, 0.0, smooth_mult*(thresh_upper - p));
    }
    else
    {
      mult = 1.0;
    }
    vec2 pVec = 0.5 * fVec / sqrt(p);

    // step 7
    vec3 normal = normalize(vec3(-pVec.x, -pVec.y, 1));

    vec3 diffuse = diffuseFactor * color.rgb * textureCube(irradiance, normalTransform*normal).rgb;
    vec3 reflection = reflectionFactor * textureCube(radiance, normalTransform*reflect(eye_dir, normal)).rgb;
    color.rgb = diffuse + reflection;
  }

  color.a *= mult;
  gl_FragColor = color;
}
