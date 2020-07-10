#include"Common.h"
#include"LocalWrapping.h"
#include"GlobalWrapping.h"
#include "lsd.h"
#include <GL\freeglut.h>
#include<cmath>

#include <iomanip>
#include <opencv2\highgui\highgui.hpp>
using namespace cv;

#define WindowTitle  "OpenGL纹理测试"
#define GL_BGR_EXT 0x80E0
#define GL_BGRA_EXT 0x80E1
GLuint texGround;
vector<vector<CoordinateDouble>> outputmesh;
vector<vector<CoordinateDouble>> mesh;
CVMat img;//记录图像
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
//
void display(void){
	glLoadIdentity();
	// 清除屏幕
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBindTexture(GL_TEXTURE_2D, texGround);
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
			
			//
			glBegin(GL_QUADS);
				glTexCoord2d(local_right_top.col, local_right_top.row); glVertex3d(global_right_top.col, -1*global_right_top.row, 0.0f);
				glTexCoord2d(local_right_bottom.col, local_right_bottom.row); glVertex3d(global_right_bottom.col, -1*global_right_bottom.row, 0.0f);
				glTexCoord2d(local_left_bottom.col, local_left_bottom.row);	glVertex3d(global_left_bottom.col, -1*global_left_bottom.row, 0.0f);
				glTexCoord2d(local_left_top.col, local_left_top.row);glVertex3d(global_left_top.col, -1*global_left_top.row, 0.0f);
			glEnd();
			
		}
	}
	/*
	int row = 18;
	int col = 18;
	CoordinateDouble local_left_top = mesh[row][col];
	CoordinateDouble local_right_top = mesh[row][col + 1];
	CoordinateDouble local_left_bottom = mesh[row + 1][col];
	CoordinateDouble local_right_bottom = mesh[row + 1][col + 1];


	CoordinateDouble global_left_top = outputmesh[row][col];
	CoordinateDouble global_right_top = outputmesh[row][col + 1];
	CoordinateDouble global_left_bottom = outputmesh[row + 1][col];
	CoordinateDouble global_right_bottom = outputmesh[row + 1][col + 1];
	glBegin(GL_QUADS);
	glTexCoord2d(local_right_top.col, local_right_top.row);
	glVertex3d(1, 0.7, 0.0f);
	glTexCoord2d(local_right_bottom.col, local_right_bottom.row);
	glVertex3d(1, -0.7, 0.0f);
	glTexCoord2d(local_left_bottom.col, local_left_bottom.row); 	
	glVertex3d(-1, -1, 0.0f);
	glTexCoord2d(local_left_top.col, local_left_top.row); 
	glVertex3d(-1, 1, 0.0f);
	glEnd();
	*/
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
	//读取图片
	img = cv::imread("C:\\users\\33712\\Desktop\\MyRectangle\\MyRectangle\\testimg\\2.jpg");
	//记录起始时间
	double Time = (double)cvGetTickCount();
	//cv::resize(img, img, cv::Size(0, 0), 0.5, 0.5);
	//记录缩放后的图像
	CVMat scaled_img;
	//对图像进行缩放
	cv::resize(img, scaled_img, cv::Size(0, 0), 0.5, 0.5);
	//记录缩放后图像的行列数以及网格线数
	Config config(scaled_img.rows,scaled_img.cols,20,20);
	//获取遮罩图像
	CVMat mask = Mask_contour(scaled_img);
	CVMat tmpmask;
	mask.copyTo(tmpmask);
	//**输出遮罩图像信息
	//printMat(tmpmask, 3);//输出图像的矩阵信息
    //namedWindow("MyTempMask", CV_WINDOW_AUTOSIZE);//创建一个名字为MyWindow的窗口
    //imshow("MyTempMask", tmpmask);//在MyWindow的窗中中显示存储在img中的图片
    //waitKey(0);//等待直到有键按下
    //destroyWindow("MyTempMask");//销毁MyWindow的窗口
	
	CVMat wrapped_img = CVMat::zeros(scaled_img.size(), CV_8UC3);
	//*****进行Local Warpping，同时获取到最终的偏移量矩阵，这将用于warp back，这里的displacementMap记录了每个像素点的偏移
	vector<vector<Coordinate>> displacementMap = Local_wrap(scaled_img,wrapped_img,tmpmask);
	//**输出seam carving图像信息
	//namedWindow("MyWarppedImage", CV_WINDOW_AUTOSIZE);//创建一个名字为MyWindow的窗口
    //imshow("MyWarppedImage", wrapped_img);//在MyWindow的窗中中显示存储在img中的图片
    //waitKey(0);//等待直到有键按下
    //destroyWindow("MyWarppedImage");//销毁MyWindow的窗口
	
	//获取经过local warpping后的每个网格点的坐标
	mesh = get_rectangle_mesh(scaled_img,config);
	
	//drawmesh(wrapped_img, mesh, config);
	//system("pasue");
	
	//进行warp back，得到原始图像的网格点信息
	wrap_mesh_back(mesh,displacementMap,config);
	cout << "Finish wrap back. Begin global warpping."<<endl;

	//计算每个网格点的shape energy属性，并存入到shape_energy中
	SpareseMatrixD_Row shape_energy = get_shape_mat(mesh,config);
	cout << "Finish get shape energy."<<endl;
	//8*2
	SpareseMatrixD_Row Q = get_vertex_to_shape_mat(mesh,config);
	//计算Border Constraint属性，并记录在组合容器border_pos_val中
	//组合的第一个元素为2n*2n，表示边界点的某个方向的坐标
	//组合的第二个元素为2n*1（一维），表示边界点某个方向的坐标的取值
	pair<SpareseMatrixD_Row, VectorXd> border_pos_val = get_boundary_mat(scaled_img, mesh, config);
	cout << "Finish get border constraint" << endl;
	
	vector<pair<int, double>>id_theta;//存储bin的编号和theta的值
	vector < LineD > line_flatten;//存储划分后的一维的线段
	vector<double> rotate_theta;
	//找到图像中的线条，按照网格对其进行分割，LineSeg记录了每个容器中的线段,id_theta则记录了容器与角度的对应值
	vector<vector<vector<LineD>>> LineSeg = init_line_seg(scaled_img, mask, config, line_flatten, mesh, id_theta,rotate_theta);
	
	//按照论文中提到的，进行十轮迭代
	cout << "Begin iteration..."<<endl;
	//cout << "Begin iteration...Please"<<endl;
	for (int iter = 1; iter <= 10; iter++) {
		cout << iter << endl;
		int Nl = 0;
		vector<pair<MatrixXd, MatrixXd>> BilinearVec;//need to update
		vector<bool> bad;
		//获取line energy，见论文公式5、6
		SpareseMatrixD_Row line_energy = get_line_mat(scaled_img, mask, mesh, rotate_theta, LineSeg, BilinearVec, config, Nl, bad);
		cout << "Finish get line energy." << "  " << Nl << endl;
		//combine
		double Nq = config.meshQuadRow*config.meshQuadCol;//总的网格数
		double lambdaB = INF;//boundary energy的权重
		double lambdaL = 100;//line energy的权重，按照论文，设置为100
		SpareseMatrixD_Row shape = (1 / Nq)*(shape_energy*Q);//形状能量，论文公式2
		SpareseMatrixD_Row boundary = lambdaB * border_pos_val.first;//边界能量
		SpareseMatrixD_Row line = (lambdaL / Nl)*(line_energy*Q);//线条能量，论文公式7
		//将三个能量矩阵给拼接起来
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
		x = p_A->solve(b);//求解Ax=b，求得x为一维的2n数组
		//update mesh，根据x获取新的矩阵输出
		outputmesh = vector_to_mesh(x, config);
		//update theta
		int tmplinenum = -1;
		VectorXd thetagroup = VectorXd::Zero(50);//记录当前bin的角度和
		VectorXd thetagroupcnt = VectorXd::Zero(50);//记录当前bin的线段数
		//遍历每个网格
		for (int row = 0; row < config.meshQuadRow; row++) {
			for (int col = 0; col < config.meshQuadCol; col++) {
				vector<LineD> linesegInquad = LineSeg[row][col];//当前网格线段
				int QuadID = row * config.meshQuadCol + col;
				if (linesegInquad.size() == 0) {
					continue;
				}
				else {
					//根据新的网格，获取网格点坐标信息，得到8*1的S矩阵
					VectorXd S = get_vertice(row, col, outputmesh);
					//遍历当前网格的所有线段
					for (int k = 0; k < linesegInquad.size(); k++) {
						tmplinenum++;
						//cout << tmplinenum<<endl;
						if (bad[tmplinenum] == true) {
							continue;
						}
						//cout << tmplinenum;
						//获取当前线段的双线性权重矩阵
						pair<MatrixXd, MatrixXd> Bstartend = BilinearVec[tmplinenum];
						MatrixXd start_W_mat = Bstartend.first;//2*8
						MatrixXd end_W_mat = Bstartend.second;
						Vector2d newstart = start_W_mat * S;// (2*8)  *   (8*1) = (2*1)
						Vector2d newend = end_W_mat * S;
						//计算新的theta值
						double theta = atan((newstart(1) - newend(1)) / (newstart(0) - newend(0)));
						//计算新的theta值和旧的theta值的差值
						double deltatheta = theta - id_theta[tmplinenum].second;

						if (isnan(id_theta[tmplinenum].second) || isnan(deltatheta)) {
							continue;
						}
						//考虑越界情况
						if (deltatheta > (PI / 2)) {
							deltatheta -= PI;
						}
						if (deltatheta < (-PI / 2)) {
							deltatheta += PI;
						}
						//累加当前bin的变化的角度和
						thetagroup(id_theta[tmplinenum].first) += deltatheta;
						//累加当前bin的线段数
						thetagroupcnt(id_theta[tmplinenum].first) += 1;
						//cout << newstart << endl << endl << newend;
					}
				}
			}
		}

		//根据角度和以及线段数计算每个bin的角度平均值theta_mean
		for (int ii = 0; ii < thetagroup.size(); ii++) {
			thetagroup(ii) /= thetagroupcnt(ii);

		}
		//更新每条线段的旋转角theta
		for (int ii = 0; ii < rotate_theta.size(); ii++) {
			rotate_theta[ii] = thetagroup[id_theta[ii].first];
		}
	}//end interator
	//cout << x;
	//system("pause");
	//vector<vector<CoordinateDouble>> outputmesh = vector_to_mesh(x,config);
	//drawmesh(scaled_img, outputmesh, config);
	//system("pause");
	enlarge_mesh(mesh, 2, config);
	enlarge_mesh(outputmesh, 2, config);
	//CVMat outputimg = CVMat::zeros(img.size(), CV_32FC3);
	//CVMat ouputcnt = CVMat::zeros(img.size(), CV_32FC3);


	/*
	for (int row = 0; row < config.meshQuadRow; row++) {
		cout << row << endl;
		for (int col = 0; col < config.meshQuadCol; col++) {

			VectorXd Vq = get_vertice(row, col, outputmesh);//x0,y0,x1,y1
			VectorXd Vo = get_vertice(row, col, mesh);//x0,y0
			double col_len = max(Vq(0), max(Vq(2), max(Vq(4), Vq(6)))) - min(Vq(0), min(Vq(2), min(Vq(4), Vq(6))));
			double row_len = max(Vq(1), max(Vq(3), max(Vq(5), Vq(7)))) - min(Vq(1), min(Vq(3), min(Vq(5), Vq(7))));
			double col_step = 1 / (4 * col_len);
			double row_step = 1 / (4 * row_len);
			//system("pause");
			for (double i = 0; i < 1; i += row_step) {
				for (double j = 0; j < 1; j += col_step) {
					double v1w = 1 - i - j + i * j;
					double v2w = j - i * j;
					double v3w = i - i * j;
					double v4w = i * j;
					MatrixXd matt(2, 8);
					matt << v1w, 0, v2w, 0, v3w, 0, v4w, 0,
						0, v1w, 0, v2w, 0, v3w, 0, v4w;
					VectorXd pout = matt * Vq;
					VectorXd pref = matt * Vo;
					if (int(pout(1)) >= 0 && int(pout(0)) >= 0 && int(pout(1)) < img.rows&&int(pout(0)) < img.cols) {
						colorPixel pixel = img.at<colorPixel>(int(pref(1)), int(pref(0)));
						cv::Vec3f pixelf = cv::Vec3f(float(pixel[0]), float(pixel[1]), float(pixel[2]));
						outputimg.at<cv::Vec3f>(int(pout(1)), int(pout(0))) = outputimg.at<cv::Vec3f>(int(pout(1)), int(pout(0))) + pixelf;
						ouputcnt.at<cv::Vec3f>(int(pout(1)), int(pout(0))) += cv::Vec3f(1, 1, 1);
					}
					else {
						//cout << "unfill";
					}
				}
			}
		}
	}
	*/
	//CVMat finaloutput = outputimg / (255 * ouputcnt);
	
	//drawmesh(finaloutput, outputmesh, config);
	//drawmesh(finaloutput, outpu, config);
	//fill_image(finaloutput);
	/*cv::namedWindow("Border", CV_WINDOW_AUTOSIZE);
	cv::imshow("Border", finaloutput);
	cv::waitKey(0);*/


	//glut
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(img.cols, img.rows);
	glutCreateWindow(WindowTitle);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);    // 启用纹理
	texGround = matToTexture(img);
	glutDisplayFunc(&display);   //注册函数 
	Time = (double)cvGetTickCount() - Time;

	printf("run time = %gms\n", Time / (cvGetTickFrequency() * 1000));//毫秒
	glutMainLoop(); //循环调用
					//
	system("pause");
	return 0;
}