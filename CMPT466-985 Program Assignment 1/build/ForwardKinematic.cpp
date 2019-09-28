#pragma warning(disable : 4996)

#include "ForwardKinematic.h"
#include "VertexBufferLayout.h"




/*recursively calculate each joint's global position from the root (in-order traversal)*/
void ForwardKinematic::calculateJointPosRecursivelyWithQuaternion(Joint* joint)
{
	//check if joint is root. If yes, set the first three elements as root joint's global translation
	if (joint->parent == nullptr)
	{
		pFrame = frameData[currentFrame];
		joint->LocalPos.x = pFrame[0];
		joint->LocalPos.y = pFrame[1];
		joint->LocalPos.z = pFrame[2];
		pFrame += 3; // pFrame points to joints' Euler angles by adding by 3.
	}

	/*------------------------------coding part start------------------------------------------*/
	/*Please modify the code accordingly inside these sub-functions                            */
	/*Coding Part: 1) calculate local rotation in quaternion from euler angles for current node*/
	Vector4 localQuat = computeLocalQuaternion(joint);

	/*Coding Part: 2) calculate global rotation quaternion for child nodes*/
	joint->Globalquat = computeGlobalQuaternion(joint, localQuat);

	/*Coding Part: 3) calculate current node's global position*/
	Vector4 GlobalPosition = computeGlobalPosition(joint);
	/*------------------------------coding part end--------------------------------------------*/

	//store the result local coordinate system into the display buffer
	calculateLocalCoorSys(joint, GlobalPosition);

	//store the result globalPosition into the display buffer
	Points.push_back(GlobalPosition);

	//calculate and store the bones into the display buffer
	if (joint->parent != nullptr)
	{
		std::vector<Vector4> bone;
		Vector4 start = joint->parent->GlobalPos;
		Vector4 end = GlobalPosition;
		bone.push_back(start);
		bone.push_back(end);
		Bones.push_back(bone);
	}

	//recursively call self for position computation
	for (int i = 0; i < joint->childNum; i++)
	{
		calculateJointPosRecursivelyWithQuaternion(joint->getChild(i));
	}
}

/*compute local rotation quaternion with specific rotation angles (angle1, angle2, angle3) and a euler rotation order*/
Vector4 ForwardKinematic::computeLocalQuaternion(Joint* joint)
{
	/*add/edit your code here for part 1*/

	//defult return
	return Vector4(0.0f, 0.0f, 0.0f, 1.0f);
}

/*compute global rotation quaternion accumulated from root joint for a joint*/
Vector4 ForwardKinematic::computeGlobalQuaternion(Joint* joint, Vector4 localQuat)
{
	/*add/edit your code here for part 2*/

	//default return
	return localQuat;
}

//based on global quaternion and local position, compute global position for a joint
Vector4 ForwardKinematic::computeGlobalPosition(Joint* joint)
{
	/*add/edit your code here for part 2*/

	//default return
	joint->GlobalPos = joint->LocalPos;
	return joint->LocalPos;
}

/*draw a skeleton with one frame data*/
void ForwardKinematic::draw(Renderer &renderer, Shader &shader)
{

	vector<float> TempDraw{};
	//loading points, colors, axis
	for (size_t i = 0; i < Points.size(); i++)
	{
		TempDraw.push_back(Points[i].x);
		TempDraw.push_back(Points[i].y);
		TempDraw.push_back(Points[i].z);
		TempDraw.push_back(1.0f);
		TempDraw.push_back(0.0f);
		TempDraw.push_back(1.0f);
		for (size_t j = 0; j < 3; j++)
		{
			TempDraw.push_back(LocalCoorSys[i][j].x);
			TempDraw.push_back(LocalCoorSys[i][j].y);
			TempDraw.push_back(LocalCoorSys[i][j].z);
			if (j % 3 == 0)
			{
				TempDraw.push_back(1.0f);
				TempDraw.push_back(0.0f);
				TempDraw.push_back(0.0f);
			}
			else if (j % 3 == 2)
			{
				TempDraw.push_back(0.0f);
				TempDraw.push_back(0.0f);
				TempDraw.push_back(1.0f);

			}

			else if (j % 3 == 1)
			{
				TempDraw.push_back(0.0f);
				TempDraw.push_back(1.0f);
				TempDraw.push_back(0.0f);

			}
		}
	}

	vector<unsigned int> IndicesAxis{};
	vector<unsigned int> IndicesPonits{};

	//points & axis indices 
	for (size_t i = 0; i < TempDraw.size(); i += 8)
	{
		IndicesPonits.push_back(i);
		IndicesAxis.push_back(i);
		IndicesAxis.push_back(i + 2);
		IndicesAxis.push_back(i);
		IndicesAxis.push_back(i + 4);
		IndicesAxis.push_back(i);
		IndicesAxis.push_back(i + 6);
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
			TempDraw.push_back(Bones[i][j].x);
			TempDraw.push_back(Bones[i][j].y);
			TempDraw.push_back(Bones[i][j].z);
			TempDraw.push_back(0.0f);
			TempDraw.push_back(0.0f);
			TempDraw.push_back(0.0f);
		}

	}

	vector <unsigned int> IndicesBones{};

	//bones indices for drawing
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
Joint* ForwardKinematic::createJoint()
{
	Joint * node = new Joint();
	int i;
	for (i = 0; i < NAMESIZE && *p != '\n'; i++)
	{
		node->name[i] = *p;
		p++;
	}
	if (i == NAMESIZE) --i;
	node->name[i - 1] = 0;		//take care of '/r'£¡£¡  
	p++;

	return node;
}

/*identify the rotation order: ZYX = 1, YZX = 2,ZXY = 3,XZY = 5,YXZ = 6,XYZ = 7*/
unsigned char ForwardKinematic::getRotationOrder()
{
	return (unsigned char)((*p - 'X' + 1) * 1 + (*(p + 10) - 'X' + 1) * 2 + (*(p + 20) - 'X' + 1) * 4 - 10);
}

/*load bvh file: loadHiarachy()+loadFrameData()*/
bool ForwardKinematic::load_BVH(const char* pfile)
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
bool ForwardKinematic::loadHiarachy(unsigned char * buffer)
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
bool ForwardKinematic::loadFrameData(unsigned char * buffer)
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
			if (j == 77)
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
void ForwardKinematic::calculateLocalCoorSys(Joint* joint, Vector4 GlobalPosition)
{
	Vector4 quatInverse = Vector4(-joint->Globalquat.x, -joint->Globalquat.y, -joint->Globalquat.z, joint->Globalquat.w);

	Vector4 GlobalXaxis = quaternionMultiplication(Vector4(1.0f, 0.0f, 0.0f, 0.0f), quatInverse);
	Vector4 GlobalYaxis = quaternionMultiplication(Vector4(0.0f, 1.0f, 0.0f, 0.0f), quatInverse);
	Vector4 GlobalZaxis = quaternionMultiplication(Vector4(0.0f, 0.0f, 1.0f, 0.0f), quatInverse);
	GlobalXaxis = quaternionMultiplication(joint->Globalquat, GlobalXaxis);
	GlobalYaxis = quaternionMultiplication(joint->Globalquat, GlobalYaxis);
	GlobalZaxis = quaternionMultiplication(joint->Globalquat, GlobalZaxis);

	Vector4 xaxis = GlobalXaxis * 4.0f + GlobalPosition;
	Vector4 yaxis = GlobalYaxis * 4.0f + GlobalPosition;
	Vector4 zaxis = GlobalZaxis * 4.0f + GlobalPosition;

	std::vector<Vector4> coorSys;
	//coorSys.clear();
	coorSys.push_back(xaxis);
	coorSys.push_back(yaxis);
	coorSys.push_back(zaxis);
	LocalCoorSys.push_back(coorSys);
}

/*recursively delete joints */
void ForwardKinematic::deleteRecursive(Joint* r)
{
	for (int i = 0; i < r->childNum; i++)
	{
		deleteRecursive(r->getChild(i));
	}
	delete r;
}

/*clear all data*/
void ForwardKinematic::clear()
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
void ForwardKinematic::printRecursive(Joint* r, int n)
{
	for (int i = 0; i < n; i++) printf(" -");
	printf("%s", r->name);

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

