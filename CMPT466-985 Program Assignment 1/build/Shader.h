#pragma once
#include <string>
#include <unordered_map>
#include <glm/glm.hpp>


struct ShaderProgramSource
{
	std::string vertexsource;
	std::string fragmentsource;
};



class Shader
{
private: 
	std::string m_FilePath;
	unsigned int m_RendererID;
	


public:
	//loading the shader
	Shader(const std::string& filepath);
	~Shader();

	void Bind() const;
	void Unbind() const;
	//unfrom input color
	void SetUniform4f(const std::string& name, float v0, float v1, float v2);
	void SetUniformMat4f(const std::string& name, const glm::mat4& matrix);
private:

	ShaderProgramSource ParseShader(const std::string& filepath);
	unsigned int CreateShader(const std::string& vertexShader, const std::string& fragmentShader);
	unsigned int CompileShader(const std::string& source, unsigned int type);
	int GetUniformLocation(const std::string& name);
};

