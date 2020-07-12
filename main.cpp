#include"Common.h"
#include"LocalWrapping.h"
#include"GlobalWrapping.h"
#include "lsd.h"
#include <GL\freeglut.h>
#include<cmath>

#include <iomanip>
#include <opencv2\highgui\highgui.hpp>
using namespace cv;

#define WindowTitle  "Rectangling Panoramic Images"
#define GL_BGR_EXT 0x80E0
#define GL_BGRA_EXT 0x80E1
GLuint texGround;
vector<vector<CoordinateDouble>> outputmesh;
vector<vector<CoordinateDouble>> mesh;
CVMat img;//��¼ͼ��
GLuint matToTexture(cv::Mat mat, GLenum minFilter = GL_LINEAR,
	GLenum magFilter = GL_LINEAR, GLenum wrapFilter = GL_REPEAT) {
	//cv::flip(mat, mat, 0);
	// Generate a number for our textureID's unique handle
	GLuint textureID;
	glGenTextures(1, &textureID);

	// Bind to our texture handle
	glBindTexture(GL_TEXTURE_2D, textureID);

	// Catch silly-mistake texture interpolation method for magnification
	if (magFilter == GL_LINEAR_MIPMAP_LINEAR ||
		magFilter == GL_LINEAR_MIPMAP_NEAREST ||
		magFilter == GL_NEAREST_MIPMAP_LINEAR ||
		magFilter == GL_NEAREST_MIPMAP_NEAREST)
	{
		//cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
		magFilter = GL_LINEAR;
	}

	// Set texture interpolation methods for minification and magnification
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

	// Set incoming texture format to:
	// GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
	// GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
	// Work out other mappings as required ( there's a list in comments in main() )
	GLenum inputColourFormat = GL_BGR_EXT;
	if (mat.channels() == 1)
	{
		inputColourFormat = GL_LUMINANCE;
	}

	// Create the texture
	glTexImage2D(GL_TEXTURE_2D,     // Type of texture
		0,                 // Pyramid level (for mip-mapping) - 0 is the top level
		GL_RGB,            // Internal colour format to convert to
		mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
		mat.rows,          // Image height i.e. 480 for Kinect in standard mode
		0,                 // Border width in pixels (can either be 1 or 0)
		inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
		GL_UNSIGNED_BYTE,  // Image data type
		mat.ptr());        // The actual image data itself

						   // If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher

	return textureID;
}
//**��ʾͼ��
void display(void){
	glLoadIdentity();
	// �����Ļ
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//�����������ݣ�Ϊimg
	glBindTexture(GL_TEXTURE_2D, texGround);
	//����ÿ������
	for (int row = 0; row < 20; row++) {
		for (int col = 0; col < 20; col++) {
			CoordinateDouble &coord = outputmesh[row][col];
			CoordinateDouble &localcoord = mesh[row][col];
			coord.row /= img.rows;
			coord.col /= img.cols;
			coord.row -= 0.5;
			coord.col -= 0.5;
			coord.row *= 2;
			coord.col *= 2;
			coord.row=clamp(coord.row, -1, 1);
			coord.col=clamp(coord.col, -1, 1);
			//cout << coord << " ";

			localcoord.row /= img.rows;
			localcoord.col /= img.cols;
			localcoord.row = clamp(localcoord.row, 0, 1);
			localcoord.col = clamp(localcoord.col, 0, 1);
		}
		// cout << endl;
	}
	//system("pause");
	
	//����ÿ������
	for (int row = 0; row < 19; row++) {
		for (int col = 0; col < 19; col++) {
			CoordinateDouble local_left_top = mesh[row][col];
			CoordinateDouble local_right_top = mesh[row][col+1];
			CoordinateDouble local_left_bottom = mesh[row+1][col];
			CoordinateDouble local_right_bottom = mesh[row+1][col+1];
			

			CoordinateDouble global_left_top = outputmesh[row][col];
			CoordinateDouble global_right_top = outputmesh[row][col + 1];
			CoordinateDouble global_left_bottom = outputmesh[row + 1][col];
			CoordinateDouble global_right_bottom = outputmesh[row + 1][col + 1];
			
			//glTexCoord2d��������ԭmesh�ϵ��ĸ����꣬��glVertex3d�������������mesh�ϵ��ĸ�����
			glBegin(GL_QUADS);
				glTexCoord2d(local_right_top.col, local_right_top.row); glVertex3d(global_right_top.col, -1*global_right_top.row, 0.0f);
				glTexCoord2d(local_right_bottom.col, local_right_bottom.row); glVertex3d(global_right_bottom.col, -1*global_right_bottom.row, 0.0f);
				glTexCoord2d(local_left_bottom.col, local_left_bottom.row);	glVertex3d(global_left_bottom.col, -1*global_left_bottom.row, 0.0f);
				glTexCoord2d(local_left_top.col, local_left_top.row);glVertex3d(global_left_top.col, -1*global_left_top.row, 0.0f);
			glEnd();
			
		}
	}
	glutSwapBuffers();
}
void printMat(CVMat mat, int prec) 
{  
    for(int i=0; i<mat.size().height; i++) 
    { 
     cout << "["; 
     for(int j=0; j<mat.size().width; j++) 
     { 
      cout << setprecision(prec) << mat.at<double>(i,j); 
      if(j != mat.size().width-1) 
       cout << ", "; 
      else 
       cout << "]" << endl; 
     } 
    } 
} 
int main(int argc, char* argv[]) {
	cout << "Begin the program, please input the file name of panoramic image:";
	string s,filePath;
	cin >> s;
	filePath = "C:\\users\\33712\\Desktop\\MyRectangle\\MyRectangle\\testimg\\" + s;
	//��ȡͼƬ
	img = cv::imread(filePath);
	//��¼��ʼʱ��
	double Time = (double)cvGetTickCount();
	//cv::resize(img, img, cv::Size(0, 0), 0.5, 0.5);
	//��¼���ź��ͼ��
	CVMat scaled_img;
	//��ͼ���������
	cv::resize(img, scaled_img, cv::Size(0, 0), 1, 1);
	cout << "Pic Size:  Height:" << scaled_img.rows << "  Width:" << scaled_img.cols << endl;
	//��¼���ź�ͼ����������Լ���������
	Config config(scaled_img.rows,scaled_img.cols,20,20);
	//��ȡ����ͼ������
	CVMat mask = Mask_contour(scaled_img);
	CVMat tmpmask;
	mask.copyTo(tmpmask);
	//**�������ͼ����Ϣ
	//printMat(tmpmask, 3);//���ͼ��ľ�����Ϣ
    //namedWindow("MyTempMask", CV_WINDOW_AUTOSIZE);//����һ������ΪMyWindow�Ĵ���
    //imshow("MyTempMask", tmpmask);//��MyWindow�Ĵ�������ʾ�洢��img�е�ͼƬ
    //waitKey(0);//�ȴ�ֱ���м�����
    //destroyWindow("MyTempMask");//����MyWindow�Ĵ���
	
	CVMat wrapped_img = CVMat::zeros(scaled_img.size(), CV_8UC3);
	//*****����Local Warpping��ͬʱ��ȡ�����յ�ƫ���������⽫����warp back�������displacementMap��¼��ÿ�����ص��ƫ��
	vector<vector<Coordinate>> displacementMap = Local_wrap(scaled_img,wrapped_img,tmpmask);
	//**���seam carvingͼ����Ϣ
	//namedWindow("MyWarppedImage", CV_WINDOW_AUTOSIZE);//����һ������ΪMyWindow�Ĵ���
    //imshow("MyWarppedImage", wrapped_img);//��MyWindow�Ĵ�������ʾ�洢��img�е�ͼƬ
    //waitKey(0);//�ȴ�ֱ���м�����
    //destroyWindow("MyWarppedImage");//����MyWindow�Ĵ���
	
	//��ȡ����local warpping���ÿ������������
	mesh = get_rectangle_mesh(scaled_img,config);
	
	//drawmesh(wrapped_img, mesh, config);//����������
	//system("pasue");
	
	//����warp back���õ�ԭʼͼ����������Ϣ����¼��mesh��
	wrap_mesh_back(mesh,displacementMap,config);
	//drawmesh(scaled_img, mesh, config);//����������
	cout << "Finish wrap back. Begin global warpping."<<endl;

	//����ÿ��������shape energy���ԣ������뵽shape_energy��
	SpareseMatrixD_Row shape_energy = get_shape_mat(mesh,config);
	cout << "Finish get shape energy."<<endl;
	//8*2�����������1
	SpareseMatrixD_Row Q = get_vertex_to_shape_mat(mesh,config);
	//����Border Constraint���ԣ�����¼���������border_pos_val��
	//��ϵĵ�һ��Ԫ��Ϊ2n*2n��Ϊ1��ʾ����������ڱ߽��
	//��ϵĵڶ���Ԫ��Ϊ2n*1��һά������ʾ��������ȡֵ��ȡֵ�����˵��������������ĸ��߽磩
	pair<SpareseMatrixD_Row, VectorXd> border_pos_val = get_boundary_mat(scaled_img, mesh, config);
	cout << "Finish get border constraint" << endl;
	
	vector<pair<int, double>>id_theta;//��¼��ÿ���߶ζ�Ӧ��bin������Ƕ�theta�Ķ�Ӧֵ
	vector < LineD > line_flatten;//�洢���ֺ��һά���߶�
	vector<double> rotate_theta;//��¼ÿ���߶ε���ת��
	//�ҵ�ͼ���е��������������������зָLineSeg��¼��ÿ�������е��߶�
	vector<vector<vector<LineD>>> LineSeg = init_line_seg(scaled_img, mask, config, line_flatten, mesh, id_theta,rotate_theta);
	double Time2 = (double)cvGetTickCount()- Time;
	//�����������ᵽ�ģ�����ʮ�ֵ���
	//cout << "Begin iteration..."<<endl;
	int itNum = 10;
	cout << "Begin iteration...Please input the number of iteration:";
	cin >> itNum;
	Time = (double)cvGetTickCount();

	for (int iter = 1; iter <= itNum; iter++) {
		cout << iter << endl;
		int Nl = 0;//�߶�����
		vector<pair<MatrixXd, MatrixXd>> BilinearVec;//need to update
		vector<bool> bad;
		//��ȡline energy�������Ĺ�ʽ5��6
		SpareseMatrixD_Row line_energy = get_line_mat(scaled_img, mask, mesh, rotate_theta, LineSeg, BilinearVec, config, Nl, bad);
		cout << "Finish get line energy.";
		//combine
		double Nq = sqrt(config.meshQuadRow*config.meshQuadCol);//�ܵ�������
		double lambdaB = INF;//boundary energy��Ȩ��
		double lambdaL = sqrt(100);//line energy��Ȩ�أ��������ģ�����Ϊ100
		Nl = sqrt(Nl);//****ע�⣬����ļ���sqrt�����ܹ��Ż����
		SpareseMatrixD_Row shape = (1 / Nq)*(shape_energy*Q);//��״���������Ĺ�ʽ2
		SpareseMatrixD_Row boundary = lambdaB * border_pos_val.first;//�߽�����
		SpareseMatrixD_Row line = (lambdaL / Nl)*(line_energy*Q);//�������������Ĺ�ʽ7
		//���������������ƴ������
		SpareseMatrixD_Row K = row_stack(shape, line);
		SpareseMatrixD_Row K2 = row_stack(K, boundary);
		SparseMatrixD K2_trans = K2.transpose();
		SparseMatrixD A = K2_trans * K2;

		VectorXd B = border_pos_val.second;
		VectorXd BA = VectorXd::Zero(K2.rows());
		BA.tail(B.size()) = lambdaB * B;
		VectorXd b = K2_trans * BA;
		
		VectorXd x;
		CSolve *p_A = new CSolve(A);
		x = p_A->solve(b);//���Ax=b�����xΪһά��2n���飬�����µ������λ��
		//update mesh������x��ȡ�µľ������
		outputmesh = vector_to_mesh(x, config);
		//update theta
		int tmplinenum = -1;
		VectorXd thetagroup = VectorXd::Zero(50);//��¼��ǰbin�ĽǶȺ�
		VectorXd thetagroupcnt = VectorXd::Zero(50);//��¼��ǰbin���߶���
		//����ÿ������
		for (int row = 0; row < config.meshQuadRow; row++) {
			for (int col = 0; col < config.meshQuadCol; col++) {
				vector<LineD> linesegInquad = LineSeg[row][col];//��ǰ�����߶�
				int QuadID = row * config.meshQuadCol + col;
				if (linesegInquad.size() == 0) {
					continue;
				}
				else {
					//�����µ����񣬻�ȡ�����������Ϣ���õ�8*1��S����
					VectorXd S = get_vertice(row, col, outputmesh);
					//������ǰ����������߶�
					for (int k = 0; k < linesegInquad.size(); k++) {
						tmplinenum++;
						//cout << tmplinenum<<endl;
						if (bad[tmplinenum] == true) {
							continue;
						}
						//cout << tmplinenum;
						//��ȡ��ǰ�߶ε�˫����Ȩ�ؾ���
						pair<MatrixXd, MatrixXd> Bstartend = BilinearVec[tmplinenum];
						MatrixXd start_W_mat = Bstartend.first;//2*8
						MatrixXd end_W_mat = Bstartend.second;
						Vector2d newstart = start_W_mat * S;// (2*8)  *   (8*1) = (2*1)
						Vector2d newend = end_W_mat * S;
						//�����µ�thetaֵ
						double theta = atan((newstart(1) - newend(1)) / (newstart(0) - newend(0)));
						//�����µ�thetaֵ�;ɵ�thetaֵ�Ĳ�ֵ
						double deltatheta = theta - id_theta[tmplinenum].second;

						if (isnan(id_theta[tmplinenum].second) || isnan(deltatheta)) {
							continue;
						}
						//����Խ�����
						if (deltatheta > (PI / 2)) {
							deltatheta -= PI;
						}
						if (deltatheta < (-PI / 2)) {
							deltatheta += PI;
						}
						//�ۼӵ�ǰbin�ı仯�ĽǶȺ�
						thetagroup(id_theta[tmplinenum].first) += deltatheta;
						//�ۼӵ�ǰbin���߶���
						thetagroupcnt(id_theta[tmplinenum].first) += 1;
						//cout << newstart << endl << endl << newend;
					}
				}
			}
		}

		//���ݽǶȺ��Լ��߶�������ÿ��bin�ĽǶ�ƽ��ֵtheta_mean
		for (int ii = 0; ii < thetagroup.size(); ii++) {
			thetagroup(ii) /= thetagroupcnt(ii);
			//���ÿ��bin��ƽ���Ƕ�
			//cout << thetagroup(ii) << " ";
		}
		//����ÿ���߶ε���ת��theta
		for (int ii = 0; ii < rotate_theta.size(); ii++) {
			rotate_theta[ii] = thetagroup[id_theta[ii].first];//id_theta[ii].firstΪ��ǰ�߶ζ�Ӧ������
			//cout << rotate_theta[ii] << " ";
		}
		cout << endl;
	}//end interator

	/*enlarge_mesh(mesh, 2, config);
	enlarge_mesh(outputmesh, 2, config);*/

	//glut
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(img.cols, img.rows);
	glutCreateWindow(WindowTitle);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);    // ��������
	texGround = matToTexture(img);
	glutDisplayFunc(&display);   //ע�ắ�� 
	Time = (double)cvGetTickCount() - Time + Time2;

	printf("run time = %gms\n", Time / (cvGetTickFrequency() * 1000));//����
	glutMainLoop(); //ѭ������
					//
	system("pause");
	return 0;
}