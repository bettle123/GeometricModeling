#version 330 core

in  vec3 in_Position;
out vec4 color;
uniform mat4 MVP;

void main(void)
{
	gl_Position = MVP* vec4(in_Position, 1.0);
	color = vec4(0.95f,0.56f,0.23f,0.1f);
}
