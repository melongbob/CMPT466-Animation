#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cstdio>  
#include <cstdlib>  
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "Renderer.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "VertexArray.h"
#include "VertexBufferLayout.h"
#include "Shader.h"
#include "ForwardKinematic.h"  


const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// OpenGL call back functions' declarations
void processInput(GLFWwindow *window);

//create an instance of FK class
ForwardKinematic FK_Model;

//Main entry
int main(int argc, char *argv[])
{


	//OpenGL initialize operations
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	// uncomment this statement to fix compilation on OS X
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); 


	// glfw window creation
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "CMPT466 Programming Assignment I", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}

	//set this window the current one
	glfwMakeContextCurrent(window);
	//glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);


	//GLEW initialization
	glewExperimental = GL_TRUE;

	if (glewInit() != GLEW_OK)
	{
		std::cout << "GLEW initialization failed!" << std::endl;
		glfwDestroyWindow(window);
		glfwTerminate();
		return 1;
	}
	

	// reshape display window function, locating camera
	glViewport(0, 0, SCR_WIDTH, SCR_WIDTH);
	glm::mat4 proj = glm::perspective(45.0f, (GLfloat)SCR_WIDTH / (GLfloat)SCR_HEIGHT, 1.0f, 1000.0f);
	glm::mat4 look = glm::lookAt(glm::vec3(195.0f*cos(10.0f), 110.0f,195.0f*sin(10.0f)), glm::vec3(0.0, 50.0, 0.0), glm::vec3(0.0, 1.0, 0.0));
	glm::mat4 result = proj*look;

	//Load bvh file and reconstruct the skeleton
	FK_Model.load_BVH("../data/running.bvh");

	//Print the loaded human skeleton
	FK_Model.print();


	//floor data
	{
		float floor[]{
			500.0f, -100.0f, -500.0f,  1.0f, 1.0f, 0.0f,
			500.0f, -100.0f, 500.0f,  1.0f, 1.0f, 0.0f,
			-500.0f, -100.0f, 500.0f, 1.0f, 1.0f, 0.0f,
			-500.0f, -100.0f, -500.0f, 1.0f, 1.0f, 0.0f
		};
		unsigned int indices[] = {
			0,2,4,
			4,6,0
		};


		//generating and initiating vertex array
		VertexBuffer vbo(floor, size(floor)*sizeof(float));
		IndexBuffer ebo(indices, size(indices));
		VertexArray vao;
		VertexBufferLayout layout;
		layout.Push<float>(3,2);
		vao.AddBuffer(layout);

		//loading vertex and fragment shader
		Shader shader("resources/shaders/Basic.shader");
		shader.Bind();

		//model,view,projection application
		shader.SetUniformMat4f("u_MVP", result);

		//relaesing the buffers
		vbo.Unbind();
		ebo.Unbind();
		vao.Unbind();
		shader.Unbind();


		Renderer renderer;
		glShadeModel(GL_SMOOTH);
		glEnable(GL_DEPTH_TEST);

		//While loop
		while (!glfwWindowShouldClose(window))
		{
			//call back function for keyboard press
			processInput(window);
			
			//cleaning the frame each time
			renderer.Clear();
			
			//drawing the floor
			renderer.Draw(GL_TRIANGLES,vao, ebo, shader);

			//recalculate the skeleton joints' positions with new frame's data
			FK_Model.calculateJointPos();

			//drawing skeleton
			FK_Model.draw(renderer, shader);

			vao.Unbind();

			// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.) and the interval
			glfwSwapInterval(1.9f);
			glfwSwapBuffers(window);			
			glfwPollEvents();
		}

	}
	// glfw: terminate, clearing all previously allocated GLFW resources.
	glfwTerminate();
	return 0;
}


void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		FK_Model.addFrame();
}

