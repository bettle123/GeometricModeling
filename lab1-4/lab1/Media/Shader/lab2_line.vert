#version 330 core

in  vec3 in_Position;
uniform mat4 MVP;

out vec4 baseColor;

void main(void)
{
	vec3 k = in_Position*5;
    gl_Position = MVP* vec4(k, 1.0);
	
	float tmp  = in_Position.x+in_Position.y+in_Position.z;
	baseColor = vec4(in_Position/tmp,1.0f);
}
