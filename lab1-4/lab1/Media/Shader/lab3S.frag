#version 330 core

out vec4 out_Color;
in vec4 brushColor;

void main(void)
{
	float x = gl_FragCoord.x /1200;
	float y = gl_FragCoord.y /800;
	out_Color = vec4(x, y, 0.9, 1.0);
}