#include "Shader.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>


#include "Renderer.h"



//loading the shaders(vertex and framgent) from a file
Shader::Shader(const std::string& filepath)
	: m_FilePath(filepath), m_RendererID(0)
{
	ShaderProgramSource source = ParseShader(filepath);
	m_RendererID = CreateShader(source.vertexsource, source.fragmentsource);	
}


Shader::~Shader()
{
	glDeleteProgram(m_RendererID);
}

 ShaderProgramSource Shader::ParseShader(const std::string& filepath)
{
	std::fstream stream(filepath);

	enum class ShaderType
	{
		NONE = -1, VERTEX = 0, FRAGMENT = 1
	};


	std::string line;
	std::stringstream ss[2];
	ShaderType type = ShaderType::NONE;
	while (getline(stream, line))
	{
		if (line.find("#shader") != std::string::npos)
		{
			if (line.find("vertex") != std::string::npos)
				//set the mode to vertex
				type = ShaderType::VERTEX;
			else if (line.find("fragment") != std::string::npos)
				//set the mode to fragment
				type = ShaderType::FRAGMENT;
		}
		else
		{
			ss[(int)type] << line << '\n';
		}
	}

	return{ ss[0].str(), ss[1].str() };
}

 
 unsigned int Shader::CompileShader(const std::string& source, unsigned int type)
{
	unsigned int id = glCreateShader(type);
	const char* src = source.c_str();
	glShaderSource(id, 1, &src, NULL);
	glCompileShader(id);


	int result{};
	glGetShaderiv(id, GL_COMPILE_STATUS, &result);
	if (!result)
	{
		int length{};
		glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
		char* message = (char*)alloca(length * sizeof(char));
		glGetShaderInfoLog(id, length, &length, message);
		std::cout << "Failed to complie " << (type == GL_VERTEX_SHADER ? "vertex" : "fragment") << " Shader!" << std::endl;
		std::cout << message << std::endl;
		glDeleteShader(id);
		return 0;



	}

	return id;
}


unsigned int Shader::CreateShader(const std::string& vertexShader, const std::string& fragmentShader)
{
	unsigned int program = glCreateProgram();
	unsigned int vertexshader = CompileShader(vertexShader, GL_VERTEX_SHADER);
	unsigned int fragmentshader = CompileShader(fragmentShader, GL_FRAGMENT_SHADER);
	glAttachShader(program, vertexshader);
	glAttachShader(program, fragmentshader);
	glLinkProgram(program);
	glValidateProgram(program);
	glDeleteShader(vertexshader);
	glDeleteShader(fragmentshader);

	return program;
}


void Shader::Bind() const
{
	glUseProgram(m_RendererID);

}


void Shader::Unbind() const
{
	glUseProgram(0);
}

//set the unform color
void Shader::SetUniform4f(const std::string& name, float v0, float v1, float v2)
{
	glUniform3f(GetUniformLocation(name), v0, v1, v2);
}

//set the uniform for a matrix
void Shader::SetUniformMat4f(const std::string& name, const glm::mat4& matrix)
{
	glUniformMatrix4fv(GetUniformLocation(name),1,GL_FALSE, &matrix[0][0]);
}
//alocating the uniform
int Shader::GetUniformLocation(const std::string& name)
{
    int location= glGetUniformLocation(m_RendererID, name.c_str());
	if (location == -1)
		std::cout << "Warming: uniform '" << name << "' doesnt't exit" << std::endl;
	return location;
}