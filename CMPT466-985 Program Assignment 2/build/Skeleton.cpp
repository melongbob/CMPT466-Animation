#pragma warning(disable : 4996)

#include "Skeleton.h"
#include "VertexBufferLayout.h"

using namespace std;

/*draw a skeleton with one frame data*/
void Skeleton::draw(Shader &shader, Renderer &renderer)
{
	Points.clear();
	Bones.clear();
	LocalCoorSys.clear();

	calculateDisplayBuff(root);

	vector<float> TempDraw{};

	//loading points, colors, axis
	for (size_t i = 0; i < Points.size(); i++)
	{
		//Joint node vertex position
		TempDraw.push_back(Points[i].x);
		TempDraw.push_back(Points[i].y);
		TempDraw.push_back(Points[i].z);

		//Color Magenta
		TempDraw.push_back(1.0f);
		TempDraw.push_back(0.0f);
		TempDraw.push_back(1.0f);

		for (size_t j = 0; j < 3; j++)
		{
			//Local coordinate system vertex position
			TempDraw.push_back(LocalCoorSys[i][j].x);
			TempDraw.push_back(LocalCoorSys[i][j].y);
			TempDraw.push_back(LocalCoorSys[i][j].z);

			if (j % 3 == 0)
			{
				//Color Red
				TempDraw.push_back(1.0f);
				TempDraw.push_back(0.0f);
				TempDraw.push_back(0.0f);
			}
			else if (j % 3 == 1)
			{
				//Color Green
				TempDraw.push_back(0.0f);
				TempDraw.push_back(1.0f);
				TempDraw.push_back(0.0f);
			}
			else if (j % 3 == 2)
			{
				//Color Blue
				TempDraw.push_back(0.0f);
				TempDraw.push_back(0.0f);
				TempDraw.push_back(1.0f);
			}
		}
	}
	
	vector<unsigned int> IndicesAxis{};
	vector<unsigned int> IndicesPonits{};

	//points & axis indices
	//Each vertex has 2 attributes: position and color, each attribute has 3 components
	for (size_t i = 0; i < TempDraw.size() / 3; i += 8)
	{
		IndicesPonits.push_back(i);

		IndicesAxis.push_back(i);	  // i: index for joint node vertex
		IndicesAxis.push_back(i + 2); // i+2: index for local coordinate x-axis
		IndicesAxis.push_back(i);
		IndicesAxis.push_back(i + 4); // i+4: index for local coordinate y-axis
		IndicesAxis.push_back(i);
		IndicesAxis.push_back(i + 6); // i+6: index for local coordinate z-axis
	}

	VertexBuffer vbopoints(&TempDraw.front(), TempDraw.size() * sizeof(float));
	IndexBuffer ebopoints(&IndicesPonits.front(), IndicesPonits.size());
	IndexBuffer eboaxis(&IndicesAxis.front(), IndicesAxis.size());
	VertexArray vao;
	VertexBufferLayout layout;
	layout.Push<float>(3, 2);
	vao.AddBuffer(layout);

	vbopoints.Unbind();
	ebopoints.Unbind();
	vao.Unbind();
	shader.Unbind();


	//Draw joints
	glPointSize(7);
	renderer.Draw(GL_POINTS, vao, ebopoints, shader);

	//Draw x,y,z axis	
	glLineWidth(1.0f);
	renderer.Draw(GL_LINES, vao, eboaxis, shader);

	TempDraw.clear();

	//loading bones
	for (size_t i = 0; i < Bones.size(); i++)
	{
		for (size_t j = 0; j < 2; j++)
		{
			//Bone vertex position
			TempDraw.push_back(Bones[i][j].x);
			TempDraw.push_back(Bones[i][j].y);
			TempDraw.push_back(Bones[i][j].z);

			//Color Black
			TempDraw.push_back(0.0f);
			TempDraw.push_back(0.0f);
			TempDraw.push_back(0.0f);
		}

	}

	vector <unsigned int> IndicesBones{};

	//Bones indices for drawing
	//Each vertex has 2 attributes: position and color, each attribute has 3 components
	for (size_t i = 0; i < TempDraw.size() / 3; i += 2)
	{
		IndicesBones.push_back(i);
	}

	VertexBuffer vbobones(&TempDraw.front(), TempDraw.size() * sizeof(float));
	IndexBuffer ebobones(&IndicesBones.front(), IndicesBones.size());
	vao.AddBuffer(layout);

	vbobones.Unbind();
	ebobones.Unbind();

	//Draw bones
	glLineWidth(1.5f);
	renderer.Draw(GL_LINES, vao, ebobones, shader);
}

/*create a node*/
Joint* Skeleton::createJoint()
{
	Joint * node = new Joint();
	unsigned char name[NAMESIZE] = {0};
	int i;
	for (i = 0; i < NAMESIZE && *p != '\n'; i++)
	{
		name[i] = *p;
		//node->name[i] = *p;
		p++;
	}
	if (i == NAMESIZE) --i;
	name[i - 1] = 0;
	//node->name[i - 1] = 0;		//take care of '/r'£¡£¡  
	node->name = (char*)name;
	p++;

	return node;
}

/*load bvh file: loadHiarachy()+loadFrameData()*/
bool Skeleton::load_BVH(const char* pfile)
{
	if (pfile == 0)
		return false;

	FILE *f;

	if (!(f = fopen(pfile, "rb")))
	{
		printf("file load failed!\n");
		return false;
	}

	//get the length of the file  
	int iStart = ftell(f);
	fseek(f, 0, SEEK_END);
	int iEnd = ftell(f);
	rewind(f);
	int iFileSize = iEnd - iStart;

	//create the buffer for the bvh file 
	unsigned char *buffer = new unsigned char[iFileSize];

	if (!buffer)
	{
		printf("mem alloc failed!!\n");
		return false;
	}

	//load the file to the buffer
	if (fread(buffer, 1, iFileSize, f) != (unsigned)iFileSize)
	{
		printf("failed!!\n");
		delete[]buffer;
		return false;
	}

	//load the skeleton structure as a tree structure
	loadHiarachy(buffer);

	//load frame data
	loadFrameData(buffer);

	delete[]buffer;
	fclose(f);
	return true;
}

/*load the skeleton structure from the bvh file*/
bool Skeleton::loadHiarachy(unsigned char * buffer)
{
	//check if the file is bvh 
	p = buffer;
	const char * fileheader = "HIERARCHY";
	for (int i = 0; i < 9; i++)
	{
		if (*p != fileheader[i])
		{
			delete[]buffer;
			return false;
		}
		p++;
	}


	//load the root name
	p += 7;
	root = createJoint();
	//push the root to the top of the stack
	father.push(root);
	currentNode = root;

	//Load root offset, it is different from other joints
	while (*p != 'O') p++;
	p += 7;
	root->LocalPos.x = (float)atof((char*)p);

	p += 5;
	if (*p == ' ') p++;
	root->LocalPos.y = (float)atof((char*)p);

	p += 5;
	if (*p == ' ') p++;
	root->LocalPos.z = (float)atof((char*)p);
	p += 5;

	//for assignemnt 2
	root->GlobalPos = root->LocalPos;

	//go to rotation information as root is usually XYZ order  
	while (*p != 'r') p++;
	p--;
	//get root rotation information 
	root->rotationOrder = getRotationOrder();


	p += 30;
	//end root information loading

	//use stack to build the tree structure skeleton    
	int temp = 0;
	int counter = 1; // count the number of brackets
	for (bool running = true; running; )
	{
		if (*p == 0)
		{
			delete[]buffer;
			clear();
			return 0;
		}

		//deal with buffer accordingly
		switch (*p)
		{
		case 13://Enter
		case '\n': {p++;  break; }//line break  
		case ' ': {p++;  break; }//space
		case '\t': {p++;  break; }//tab
		case '{': {
			father.push(currentNode);
			p++;
			counter++;
			break;
		}
		case '}': {
			father.pop();

			if (!father.empty())
				currentNode = father.top();
			p++;
			counter--;
			//check if the loading of hierarchy is done
			if (counter == 0) running = false;
			break;
		}
		case 'J': {
			jointCount++;
			p += 6;
			Joint *JointNode = createJoint();
			JointNode->parent = currentNode;
			currentNode->addChild(JointNode);
			currentNode = JointNode;
			break;
		}
		case 'O': {
			//Get Local Position's x value
			p += 7;
			currentNode->LocalPos.x = atof((const char *)p);

			//Get Local Position's y value
			p += 7;
			while (*p != ' ' && *p != '  ') p++;
			p++;
			currentNode->LocalPos.y = atof((const char *)p);

			//Get Local Position's z value
			p += 7;
			while (*p != ' ' && *p != '  ') p++;
			p++;
			currentNode->LocalPos.z = atof((const char *)p);

			//for assignemnt 2
			currentNode->GlobalPos = currentNode->LocalPos;

			while (*p++ != '\n');
			break;
		}
		case 'C': {
			p += 11;
			currentNode->rotationOrder = getRotationOrder();
			p += 30;
			break;
		}
		case 'E': {
			p += 4;
			Joint *Endnode = createJoint();
			Endnode->parent = currentNode;
			currentNode->addChild(Endnode);
			currentNode = Endnode;
			break;
		}
		default:
			printf("_%c_ _%d_ file format error!! \n", *p, *p);
			delete[]buffer;
			clear();
			return false;
		}
		temp++;
	}
	//end hierarchy's loading
}

/*load the frame data from the bvh file*/
bool Skeleton::loadFrameData(unsigned char * buffer)
{
	while (*p != 'F') p++;
	p += 8;

	//number of frames
	frameCount = (unsigned)atoi((char *)p);

	while (*p != 'F') p++;
	p += 12;

	//total time length 
	frameTime = (float)atof((char*)p);


	while (*p++ != '\n');

	// allocate space for the framedata  
	frameData = new float*[frameCount];
	if (frameData == nullptr)
	{
		delete[]buffer;
		clear();
		return false;
	}
	//check if joint number is correct  
	if (jointCount == 1)
	{
		delete[]buffer;
		clear();
		return false;
	}


	//total data for one frame: root's translation + all joints' rotation information
	int dataCount = jointCount * 3 + 3;

	//allocate memory for each frame
	for (unsigned int i = 0; i < frameCount; i++)
	{
		frameData[i] = new float[dataCount];
		if (frameData == nullptr)
		{
			delete[]buffer;
			clear();
			return false;
		}
	}

	//load frame data, one frame by one frame	
	for (unsigned int i = 0; i < frameCount; i++)
	{
		//ignore space
		p += 1;
		for (int j = 0; j < dataCount; j++)
		{

			frameData[i][j] = (float)atof((char*)p);
			if (j == (dataCount - 1))
			{
				while (*p != '\r' && *p != '\n') p++;
				continue;
			}
			while (*p != ' ' && *p != '  ') p++;
			p++; //ignore tab
		}
		//ignore line breaks and enters 
		p += 2;
	}
}

/*calculate the local coordinate system and store them into display buffer for visualization*/
std::vector<Vector4> Skeleton::calculateLocalCoorSys(Joint* joint)
{
	Vector4 quatInverse = Vector4(-joint->Globalquat.x, -joint->Globalquat.y, -joint->Globalquat.z, joint->Globalquat.w);

	Vector4 GlobalXaxis = Vector4(1.0f, 0.0f, 0.0f, 0.0f).quatMulply(quatInverse);
	Vector4 GlobalYaxis = Vector4(0.0f, 1.0f, 0.0f, 0.0f).quatMulply(quatInverse);
	Vector4 GlobalZaxis = Vector4(0.0f, 0.0f, 1.0f, 0.0f).quatMulply(quatInverse);
	joint->GlobalXAxis = joint->Globalquat.quatMulply(GlobalXaxis);
	joint->GlobalYAxis = joint->Globalquat.quatMulply(GlobalYaxis);
	joint->GlobalZAxis = joint->Globalquat.quatMulply(GlobalZaxis);

	Vector4 xaxis = joint->GlobalXAxis * 4.0f + joint->GlobalPos;
	Vector4 yaxis = joint->GlobalYAxis * 4.0f + joint->GlobalPos;
	Vector4 zaxis = joint->GlobalZAxis * 4.0f + joint->GlobalPos;


	std::vector<Vector4> coorSys;
	//coorSys.clear();
	coorSys.push_back(xaxis);
	coorSys.push_back(yaxis);
	coorSys.push_back(zaxis);
	//LocalCoorSys.push_back(coorSys);

	return coorSys;
}

/*recursively delete joints */
void Skeleton::deleteRecursive(Joint* r)
{
	for (int i = 0; i < r->childNum; i++)
	{
		deleteRecursive(r->getChild(i));
	}
	delete r;
}

/*clear all data*/
void Skeleton::clear()
{
	currentNode = nullptr;
	p = nullptr;
	while (!father.empty()) father.pop();
	frameTime = 1;


	for (unsigned int i = 0; i < frameCount; i++)
		delete[]frameData[i];
	delete[]frameData;
	frameData = nullptr;
	pFrame = nullptr;
	frameCount = 0;
	currentFrame = 0;
	jointCount = 1;

	if (root != nullptr)
	{
		deleteRecursive(root);
	}
	root = nullptr;
}

//print the hierarchy of the skeleton
void Skeleton::printRecursive(Joint* r, int n)
{
	for (int i = 0; i < n; i++) printf(" -");

	std::cout << r->name;

	printf(" Local Position %f,%f,%f -%d- ", r->LocalPos.x, r->LocalPos.y, r->LocalPos.z, r->rotationOrder);
	switch (r->rotationOrder)
	{
	case 1:
		printf("zyx");
		break;
	case 2:
		printf("yzx");
		break;
	case 3:
		printf("zxy");
		break;
	case 5:
		printf("xzy");
		break;
	case 6:
		printf("yxz");
		break;
	case 7:
		printf("xyz");
		break;
	default:
		break;
	}
	printf("\n");
	for (int i = 0; i < r->childNum; i++)
	{
		printRecursive(r->getChild(i), n + 1);
	}
}

