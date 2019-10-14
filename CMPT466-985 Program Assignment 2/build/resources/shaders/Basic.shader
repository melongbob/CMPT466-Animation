#shader vertex
#version 330 core
layout (location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;
uniform mat4 u_MVP;
out vec3 u_color;
void main()
{
   gl_Position = u_MVP*vec4(aPos.x, aPos.y, aPos.z, 1.0);
   u_color = aColor;
};


#shader fragment
#version 330 core
layout(location =0) out vec4 FragColor;

//uniform vec3 u_color;
in vec3 u_color;

void main()
{
   FragColor =vec4(u_color,1.0f);
};