#version 330 core
// Fragment Shader – file "bc.frag"

out vec4 out_Color;
in vec4 brushColor;

void main(void)
{
	out_Color = vec4(0.5, 1.0, 1.0, 1.0);
}