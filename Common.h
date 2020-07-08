#pragma once
#ifndef Common_h
#define Common_h

#define INF 1e8   //infinite
#define PI 3.14159265358979323846

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include<iostream>
#include<vector>
#include<algorithm>
#include"lsd.h"
#include <Eigen/Sparse>
#include<cmath>
#include<Eigen/Dense>

typedef cv::Mat CVMat;//CV图像
typedef cv::Vec3b colorPixel;//三色像素

typedef Eigen::SparseMatrix<double> SparseMatrixD;//稀疏矩阵
typedef Eigen::SparseMatrix<double, Eigen::RowMajor> SpareseMatrixD_Row;
typedef Eigen::VectorXd VectorXd;
typedef Eigen::MatrixXd MatrixXd;
typedef Eigen::Vector2d Vector2d;
typedef Eigen::Vector2i Vector2i;
typedef Eigen::MatrixXi MatrixXi;
typedef Eigen::Matrix2d Matrix2d;
typedef Eigen::Triplet<double> T;
typedef Eigen::SimplicialCholesky<SparseMatrixD> CSolve;

using namespace std;

struct Config {//运行前配置
	int rows;//行数
	int cols;//列数
	int meshNumRow;//网格线行数
	int meshNumCol;//网格线列数
	int meshQuadRow;//网格行数
	int meshQuadCol;//网格列数
	double rowPermesh;//每个网格的行数
	double colPermesh;//每个网格的列数
	//构造函数
	Config(int rows, int cols, int meshNumRow, int meshNumCol) {
		this->rows = rows;
		this->cols = cols;
		this->meshNumRow = meshNumRow;
		this->meshNumCol = meshNumCol;
		this->meshQuadCol = meshNumCol - 1;
		this->meshQuadRow = meshNumRow - 1;
		this->rowPermesh = double(rows - 1) / (meshNumRow - 1);
		this->colPermesh = double(cols - 1) / (meshNumCol - 1);
	}
};

//坐标轴（整形）
struct Coordinate {
	int row;//行坐标
	int col;//列坐标

	//重写坐标相等
	bool operator==(const Coordinate& rhs) const {
		return (row == rhs.row && col == rhs.col);
	}
	//重写坐标小于
	bool operator<(const Coordinate& rhs) const {
		// this operator is used to determine equality, so it must use both x and y
		if (row < rhs.row) {
			return true;
		}
		if (row > rhs.row) {
			return false;
		}
		return col < rhs.col;
	}
	//构造函数
	Coordinate() { row = 0; col = 0; };
	Coordinate(int setRow, int setCol) { row = setRow; col = setCol; };
};
//坐标轴（浮点型）
struct CoordinateDouble {
	double row;//行数
	double col;//列数
	//重写坐标相等
	bool operator==(const CoordinateDouble& rhs) const {
		return (row == rhs.row && col == rhs.col);
	}
	//重写坐标小于
	bool operator<(const CoordinateDouble& rhs) const {
		// this operator is used to determine equality, so it must use both x and y
		if (row < rhs.row) {
			return true;
		}
		if (row > rhs.row) {
			return false;
		}
		return col < rhs.col;
	}
	//重写坐标输出
	friend ostream &operator<<(ostream &stream, const CoordinateDouble &p){
		stream << "("<<p.col<<","<<p.row<<")";
		return stream;
	}
	//构造函数
	CoordinateDouble() { row = 0; col = 0; };
	CoordinateDouble(double setRow, double setCol) { row = setRow; col = setCol; };
};

//一条线段
struct LineD {
	//由线段的两个顶点坐标唯一确定该线段
	double row1, col1;//线段顶点一
	double row2, col2;//线段顶点二
	LineD(double row1, double col1, double row2, double col2) {
		this->row1 = row1;
		this->row2 = row2;
		this->col1 = col1;
		this->col2 = col2;
	}
	LineD() { row1 = 0; col1 = 0; row2 = 0; col2 = 0; }
	LineD(CoordinateDouble p1, CoordinateDouble p2) { row1 = p1.row; row2 = p2.row; col1 = p1.col; col2 = p2.col; }
};

CVMat Mask_contour(CVMat src);
VectorXd mesh_to_vector(vector<vector<CoordinateDouble>> mesh, Config config);
vector<vector<CoordinateDouble>> vector_to_mesh(VectorXd x, Config config);
void print_sparse_mat(SparseMatrixD Q);
SpareseMatrixD_Row row_stack(SparseMatrixD origin, SpareseMatrixD_Row diag);
SpareseMatrixD_Row row_stack(SpareseMatrixD_Row origin, SpareseMatrixD_Row diag);
MatrixXd row_stack(MatrixXd mat1, MatrixXd mat2);
MatrixXd col_stack(MatrixXd mat1, MatrixXd mat2);
void DrawLine(CVMat& img, CoordinateDouble coordstart, CoordinateDouble coordend);
void DrawLine(CVMat& img, LineD line);
CVMat drawmesh(CVMat src, vector<vector<CoordinateDouble>> mesh, Config config);
void enlarge_mesh(vector<vector<CoordinateDouble>>& mesh, double enlarge_factor, Config config);

#endif