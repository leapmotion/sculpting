#version 120

uniform sampler2D	color_tex;
uniform sampler2D depth_tex;
uniform vec2 offset_mult;
uniform float depth_min;
uniform float depth_max;
uniform float offset_min;
uniform float offset_max;

#define BLUR_RADIUS 10
#define BLUR_WIDTH (2*BLUR_RADIUS + 1)

const float weights[BLUR_WIDTH] = float[](0.009167927656011385,
																					0.014053461291849008,
																					0.020595286319257878,
																					0.028855245532226279,
																					0.038650411513543079,
																					0.049494378859311142,
																					0.060594058578763078,
																					0.070921288047096992,
																					0.079358891804948081,
																					0.084895951965930902,
																					0.086826196862124602,
																					0.084895951965930902,
																					0.079358891804948081,
																					0.070921288047096992,
																					0.060594058578763092,
																					0.049494378859311121,
																					0.038650411513543079,
																					0.028855245532226279,
																					0.020595286319257885,
																					0.014053461291849008,
																					0.009167927656011385);

void main() {
	float cur_offset;
	if (depth_min > 0)
	{
		float cur_depth = texture2D(depth_tex, gl_TexCoord[0].st).r;
		float depth_ratio = (cur_depth - depth_min)/(depth_max - depth_min);
		depth_ratio = clamp(depth_ratio, 0, 1);
		depth_ratio = depth_ratio*depth_ratio*depth_ratio;
		cur_offset = depth_ratio*(offset_max - offset_min) + offset_min;
	}
	else
	{
		cur_offset = 0;
	}

	if (cur_offset > 0.000001)
	{
		vec2 sample_offset = vec2(cur_offset*offset_mult.x, cur_offset*offset_mult.y);

		vec3 sum = vec3(0);
		vec2 offset = vec2(0);
		vec2 base_offset = -BLUR_RADIUS * sample_offset;

		for (int s = 0; s < BLUR_WIDTH; ++s)
		{
			sum += texture2D(color_tex, gl_TexCoord[0].st + base_offset + offset).rgb * weights[s];
			offset += sample_offset;
		}

		gl_FragColor.rgb = sum;
		gl_FragColor.a = 1.0;
	}
	else
	{
		gl_FragColor = texture2D(color_tex, gl_TexCoord[0].st);
		gl_FragColor.a = 1.0;
	}
}
