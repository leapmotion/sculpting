uniform sampler2D input_texture;
uniform float light_threshold;

void main()
{
	vec4 col = texture2D(input_texture, gl_TexCoord[0].st);

	col = clamp(1.3*(col - vec4(light_threshold)) + vec4(light_threshold), vec4(0), vec4(10));
	col.a = 1.0;
	gl_FragColor = col;
}

