uniform sampler2D color_texture;
uniform sampler2D bloom_texture;
uniform sampler2D depth_texture;
uniform float exposure;
uniform float bloom_strength;
uniform float contrast;
uniform float width;
uniform float height;
uniform float vignette_radius;
uniform float vignette_strength;
uniform bool use_ao;
uniform bool only_ao;

#define PI    3.14159265

float znear = 0.01; //Z-near
float zfar = 4.0; //Z-far

//user variables
int samples = 16; //ao sample count

float radius = 3.0; //ao radius
float aoclamp = 0.01; //depth clamp - reduces haloing at screen edges
bool noise = true; //use noise instead of pattern for sample dithering
float noiseamount = 0.003; //dithering amount

float diffarea = 0.5; //self-shadowing reduction
float gdisplace = 0.6; //gauss bell center
float aowidth = 1.0; //gauss bell width

float lumInfluence = 0.5; //how much luminance affects occlusion
const vec3 lumcoeff = vec3(0.299,0.587,0.114);


vec2 rand(vec2 coord) //generating noise/pattern texture for dithering
{
  float noiseX = ((fract(1.0-coord.s*(width/2.0))*0.25)+(fract(coord.t*(height/2.0))*0.75))*2.0-1.0;
  float noiseY = ((fract(1.0-coord.s*(width/2.0))*0.75)+(fract(coord.t*(height/2.0))*0.25))*2.0-1.0;

  if (noise)
  {
    noiseX = clamp(fract(sin(dot(coord ,vec2(12.9898,78.233))) * 43758.5453),0.0,1.0)*2.0-1.0;
    noiseY = clamp(fract(sin(dot(coord ,vec2(12.9898,78.233)*2.0)) * 43758.5453),0.0,1.0)*2.0-1.0;
  }
  return vec2(noiseX,noiseY)*noiseamount;
}

float readDepth(in vec2 coord) 
{
  if (gl_TexCoord[0].x<0.0||gl_TexCoord[0].y<0.0) return 1.0;
  return (2.0 * znear) / (zfar + znear - texture2D(depth_texture, coord ).x * (zfar-znear));
}

float compareDepths(in float depth1, in float depth2,inout int far)
{   
  float garea = aowidth; //gauss bell width    
  float diff = (depth1 - depth2)*100.0; //depth difference (0-100)
  //reduce left bell width to avoid self-shadowing 
  if (diff<gdisplace)
  {
    garea = diffarea;
  }
  else
  {
    far = 1;
  }

  float gauss = pow(2.7182,-2.0*(diff-gdisplace)*(diff-gdisplace)/(garea*garea));
  return gauss;
}   

float calAO(float depth,float dw, float dh)
{   
  //float dd = (1.0-depth)*radius;
  float dd = radius;
  float temp = 0.0;
  float temp2 = 0.0;
  float coordw = gl_TexCoord[0].x + dw*dd;
  float coordh = gl_TexCoord[0].y + dh*dd;
  float coordw2 = gl_TexCoord[0].x - dw*dd;
  float coordh2 = gl_TexCoord[0].y - dh*dd;

  vec2 coord = vec2(coordw , coordh);
  vec2 coord2 = vec2(coordw2, coordh2);

  int far = 0;
  temp = compareDepths(depth, readDepth(coord),far);
  //DEPTH EXTRAPOLATION:
  if (far > 0)
  {
    temp2 = compareDepths(readDepth(coord2),depth,far);
    temp += (1.0-temp)*temp2;
  }

  return temp;
} 

void main(void)
{
  float ao;
  if (use_ao)
  {
    vec2 noise = rand(gl_TexCoord[0].st); 
    float depth = readDepth(gl_TexCoord[0].st);

    float w = (1.0 / width)/clamp(depth,aoclamp,1.0)+(noise.x*(1.0-noise.x));
    float h = (1.0 / height)/clamp(depth,aoclamp,1.0)+(noise.y*(1.0-noise.y));

    float pw;
    float ph;

    float dl = PI*(3.0-sqrt(5.0));
    float dz = 1.0/float(samples);
    float l = 0.0;
    float z = 1.0 - dz/2.0;

    for (int i = 0; i <= samples; i ++)
    {     
      float r = sqrt(1.0-z);

      pw = cos(l)*r;
      ph = sin(l)*r;
      ao += calAO(depth,pw*w,ph*h);        
      z = z - dz;
      l = l + dl;
    }

    ao /= float(samples);
    ao = 1.0-ao;    
  }

  vec3 color = texture2D(color_texture,gl_TexCoord[0].st).rgb;
  color += texture2D(bloom_texture, gl_TexCoord[0].st).rgb*bloom_strength;

  vec2 diff = (gl_FragCoord.xy - vec2(width/2.0, height/2.0))/vignette_radius;
  float vignette = dot(diff, diff);
  color *= (1.0 - (vignette_strength * vignette));

  color = 1.0 - exp2(-color * exposure);
  color = contrast*(color - vec3(0.5)) + vec3(0.5);
  vec3 final = color;

  if (use_ao)
  {
    float lum = dot(color.rgb, lumcoeff);
    vec3 luminance = vec3(lum, lum, lum);

    final = vec3(color*mix(vec3(ao),vec3(1.0),luminance*lumInfluence));

    if (only_ao)
    {
      final = vec3(mix(vec3(ao),vec3(1.0),luminance*lumInfluence)); //ambient occlusion only
    }
  }

  gl_FragColor = vec4(final, 1.0); 
}
