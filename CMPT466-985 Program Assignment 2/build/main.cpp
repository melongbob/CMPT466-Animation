#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cstdio>  
#include <cstdlib> 
#include <vector>


#include "Skeleton.h"  
#include "Renderer.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "VertexArray.h"
#include "VertexBufferLayout.h"
#include "Shader.h"
#include "ForwardKinematics.h"  


const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
float viewingAngle = 45.0f;

// OpenGL callback functions' declarations
void processInput(GLFWwindow *window);//s
void drawsphere(std::vector<float> &spherepos, std::vector<unsigned int> &sphereind, Shader &shader, Renderer &renderer, glm::mat4 &result);
std::pair< std::vector<float>, std::vector<unsigned int >> sphere(float radius, float slice, float stack);


//create an instance of Skeleton class
Skeleton skeleton;

//create an instance of forward kinematic class
ForwardKinematics forwardKinematics;

//create an instance of inverse kinematic class
InverseKinematics inverseKinematics;

//target for inverse kinematics
Target target;

//Main entry
int main(int argc, char *argv[])
{


	//OpenGL initilize operations
	glfwInit();//s
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	// uncomment this statement to fix compilation on OS X
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); 


	// glfw window creation
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "CMPT466 Programming Assignment II", NULL, NULL);
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


	//Load bvh file and reconstruct the skeleton
	skeleton.load_BVH("../data/running.bvh");

	//Print the loaded human skeleton
	skeleton.print();

	//initialize a pointer to the root of the skeleton
	Joint* root = skeleton.getRoot();

	//get the right end site joint
	Joint* endEffector = nullptr;
	skeleton.getJoint("R_Wrist_End", root, endEffector);

	//get the right shoulder joint
	Joint* rightShoulder = nullptr;
	skeleton.getJoint("RightShoulder", root, rightShoulder);

	//Initialize InverseKinematics instance
	inverseKinematics.initialize(endEffector, rightShoulder, &target);

	//Do Forward Kinematics once for generating the default T-Pose skeleton 
	forwardKinematics.calculateJointPosWithQuaternion(root);


	{
		//floor data
		float floor[]{
			500.0f, -100.0f, -500.0f,  1.0f, 1.0f, 0.0f,
			500.0f, -100.0f,  500.0f,  1.0f, 1.0f, 0.0f,
		   -500.0f, -100.0f,  500.0f,  1.0f, 1.0f, 0.0f,
		   -500.0f, -100.0f, -500.0f,  1.0f, 1.0f, 0.0f
		};
		unsigned int indices[] = {
			0,2,4,
			4,6,0
		};


		//axis data
		float axisx[]{
			 0.0f, 0.0f, 70.0f, 1.0f, 0.0f, 0.0f,
			10.0f, 0.0f, 70.0f, 1.0f, 0.0f, 0.0f
		};

		float axisy[]
		{
			0.0f, 0.0f, 70.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 10.0f, 70.0f,0.0f, 1.0f, 0.0f
		};

		float axisz[]
		{
			0.0f, 0.0f, 70.0f, 0.0f, 0.0f, 1.0f,
			0.0f, 0.0f, 80.0f, 0.0f, 0.0f, 1.0f
		};

		unsigned int indicesaxis[] = {
			0,2
		};

		//generating and initiating vertex array
		VertexBuffer vbo(floor, std::size(floor) * sizeof(float));
		IndexBuffer ebo(indices, std::size(indices));
		VertexArray vao;
		VertexBufferLayout layout;
		layout.Push<float>(3, 2);
		vao.AddBuffer(layout);

		vbo.Unbind();
		ebo.Unbind();
		vao.Unbind();

		//------------ Axis-----------------------
		VertexBuffer vbox(axisx, std::size(axisx) * sizeof(float));
		IndexBuffer ebox(indicesaxis, std::size(indicesaxis));
		VertexArray vaox;
		vaox.AddBuffer(layout);

		vbox.Unbind();
		ebox.Unbind();
		vaox.Unbind();


		VertexBuffer vboy(axisy, std::size(axisy) * sizeof(float));
		IndexBuffer eboy(indicesaxis, std::size(indicesaxis));
		VertexArray vaoy;
		vaoy.AddBuffer(layout);

		vboy.Unbind();
		eboy.Unbind();
		vaoy.Unbind();

		VertexBuffer vboz(axisz, std::size(axisz) * sizeof(float));
		IndexBuffer eboz(indicesaxis, std::size(indicesaxis));
		VertexArray vaoz;
		vaoy.AddBuffer(layout);

		vboz.Unbind();
		eboz.Unbind();
		vaoz.Unbind();
		//------------------------------------------


		// generating, compliling vertex and fragment shader and creating a program
		Shader shader("resources/shaders/Basic.shader");

		Renderer renderer;
		glEnable(GL_DEPTH_TEST);


		//While loop
		while (!glfwWindowShouldClose(window))
		{
			//call back function for keyboard press
			processInput(window);


			glViewport(0, 0, SCR_WIDTH, SCR_WIDTH);
			glm::mat4 proj = glm::perspective(45.0f, (GLfloat)SCR_WIDTH / (GLfloat)SCR_HEIGHT, 1.0f, 1000.0f);
			glm::mat4 view = glm::lookAt(glm::vec3(180.0f*cosf(viewingAngle*3.141592658f / 180.0f), 100.0f, 180.0f*sinf(viewingAngle*3.141592658f / 180.0f)), glm::vec3(0.0, 30.0, 0.0), glm::vec3(0.0, 1.0, 0.0));
			glm::mat4 result = proj * view;
			shader.SetUniformMat4f("u_MVP", result);



			//cleaning the frame each time
			renderer.Clear();

			//drawing the floor
			renderer.Draw(GL_TRIANGLES, vao, ebo, shader);

			// draw legend axis
			renderer.Draw(GL_LINES, vaox, ebox, shader);
			renderer.Draw(GL_LINES, vaoy, eboy, shader);
			renderer.Draw(GL_LINES, vaoz, eboz, shader);


			//draw skeleton
			skeleton.draw(shader, renderer);

			//loading sphere data
			auto data = sphere(2.0f, 5.0f, 5.0f);
			std::vector<float> spherepos = data.first;
			std::vector<unsigned int> sphereind = data.second;

			//draw sphere
			drawsphere(spherepos, sphereind, shader, renderer, result);


			vao.Unbind();

			// glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.) and the interval
			glfwSwapInterval(2);
			glfwSwapBuffers(window);
			glfwPollEvents();
		}

	}
	// glfw: terminate, clearing all previously allocated GLFW resources.
	glfwTerminate();
	return 0;
}


//creating sphere
std::pair<std::vector<float>, std::vector<unsigned int>> sphere(float Radius, float Slices, float Stacks)
{
	std::vector<float> vertices;
	std::vector<unsigned int> indices;

	for (size_t i = 0; i <= Stacks; ++i) {

		float V = i / (float)Stacks;
		float phi = V * glm::pi <float>();

		// Loop Through Slices
		for (int j = 0; j <= Slices; ++j) {

			float U = j / (float)Slices;
			float theta = U * (glm::pi <float>() * 2);

			// Calc The Vertex Positions
			float x = cosf(theta) * sinf(phi);
			float y = cosf(phi);
			float z = sinf(theta) * sinf(phi);

			// Push Back Vertex Data
			vertices.push_back(x*Radius);
			vertices.push_back(y*Radius);
			vertices.push_back(z*Radius);
			vertices.push_back(1.0f);
			vertices.push_back(0.0f);
			vertices.push_back(0.0f);


		}
	}

	// Calc The Index Positions
	for (size_t i = 0; i < Slices * Stacks + Slices; ++i) {

		indices.push_back(i * 2);
		indices.push_back(2 * (i + Slices + 1));
		indices.push_back(2 * (i + Slices));

		indices.push_back(2 * (i + Slices + 1));
		indices.push_back(i * 2);
		indices.push_back(2 * (i + 1));
	}

	return{ vertices,indices };
}
 //draw sphere
void drawsphere(std::vector<float> &spherepos, std::vector<unsigned int> &sphereind, Shader &shader, Renderer &renderer, glm::mat4 &result)
{
	VertexBuffer vbosph(&spherepos.front(), std::size(spherepos) * sizeof(float));
	IndexBuffer ebosph(&sphereind.front(), std::size(sphereind));
	VertexArray vaosph;
	VertexBufferLayout layout2;
	layout2.Push<float>(3, 2);
	vaosph.AddBuffer(layout2);

	vbosph.Unbind();
	ebosph.Unbind();
	vaosph.Unbind();
	glm::mat4 myMatrix = glm::translate(result, glm::vec3(target.x, target.y, target.z));

	shader.SetUniformMat4f("u_MVP", myMatrix);
	renderer.Draw(GL_TRIANGLES, vaosph, ebosph, shader);

}


//OpenGL call back function: kepboard responsing functiion: once w is pressed on keyboard, one more frame data is loaded
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		skeleton.addFrame();
	if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS)
	{
		viewingAngle = viewingAngle + 1.0f;
		if (viewingAngle >= 360.0f)
		{
			viewingAngle = 0.0f;
		}
	}
	if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS)
	{
		viewingAngle = viewingAngle - 1.0f;
		if (viewingAngle <= -360.0f)
		{
			viewingAngle = 0.0f;
		}
	}
	if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
		inverseKinematics.setMode(0);
	if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
		inverseKinematics.setMode(1);
	if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
	{
		target.x = -31.7568f;
		target.y = 18.1144f;
		target.z = 0;
		inverseKinematics.reset();
	}
	if (glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS)
	{
		target.x = target.x + 1.0f;
		inverseKinematics.IK();

	}
	if (glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS)
	{
		target.x = target.x - 1.0f;
		inverseKinematics.IK();

	}
	if (glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS)
	{
		target.y = target.y + 1.0f;
		inverseKinematics.IK();

	}
	if (glfwGetKey(window, GLFW_KEY_4) == GLFW_PRESS)
	{
		target.y = target.y - 1.0f;
		inverseKinematics.IK();

	}
	if (glfwGetKey(window, GLFW_KEY_5) == GLFW_PRESS)
	{
		target.z = target.z + 1.0f;
		inverseKinematics.IK();

	}
	if (glfwGetKey(window, GLFW_KEY_6) == GLFW_PRESS)
	{
		target.z = target.z - 1.0f;
		inverseKinematics.IK();

	}
}
