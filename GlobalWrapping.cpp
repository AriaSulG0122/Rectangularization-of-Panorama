#include"GlobalWrapping.h"


pair<SpareseMatrixD_Row, VectorXd> get_boundary_mat(CVMat src, vector<vector<CoordinateDouble>> mesh, Config config) {
	//Vq=[x0 y0,x1,y1...]
	int rows = config.rows;//行数
	int cols = config.cols;//列数
	int numLineRow = config.meshLineRow;//网格线行数
	int numLineCol = config.meshLineCol;//网格线列数
	int vertexnum = numLineRow * numLineCol;//网格点数
	// *2是代表网格点的(x,y)
	VectorXd dvec = VectorXd::Zero(vertexnum * 2);
	VectorXd Value = VectorXd::Zero(vertexnum * 2);
	//遍历左侧所有点
	for (int i = 0; i < vertexnum * 2; i += numLineCol * 2) {
		//限制x坐标为0
		dvec(i) = 1;
		Value(i) = 0;
	}
	//遍历右侧所有点
	for (int i = numLineCol * 2 - 2; i < vertexnum * 2; i += numLineCol * 2) {
		//限制x坐标为cols-1
		dvec(i) = 1;
		Value(i) = cols - 1;
	}
	//遍历上侧所有点
	for (int i = 1; i < 2 * numLineCol; i += 2) {
		//限制y坐标为0
		dvec(i) = 1;
		Value(i) = 0;
	}
	//遍历下侧所有点
	for (int i = 2 * vertexnum - 2 * numLineCol + 1; i < vertexnum * 2; i += 2) {
		//限制y坐标为rows-1
		dvec(i) = 1;
		Value(i) = rows - 1;
	}

	//diag sparse;
	SpareseMatrixD_Row diag(dvec.size(), dvec.size());
	for (int i = 0; i < dvec.size(); i++) {
		diag.insert(i, i) = dvec(i);
	}
	diag.makeCompressed();
	return make_pair(diag, Value);
};

//获取shape energy
SpareseMatrixD_Row get_shape_mat(vector<vector<CoordinateDouble>> mesh, Config config) {
	int numQuadRow = config.meshQuadRow;//网格行数
	int numQuadCol = config.meshQuadCol;//网格列数
	//*8是代表网格的(x0,y0,x1,y1,x2,y2,x3,y3,x4,y4)
	SpareseMatrixD_Row Shape_energy(8 * numQuadRow*numQuadCol, 8 * numQuadRow*numQuadCol);
	for (int row = 0; row < numQuadRow; row++) {
		for (int col = 0; col < numQuadCol; col++) {
			//读取当前网格的四个点
			CoordinateDouble p0 = mesh[row][col];//左上
			CoordinateDouble p1 = mesh[row][col + 1];//右上
			CoordinateDouble p2 = mesh[row + 1][col];//左下
			CoordinateDouble p3 = mesh[row + 1][col + 1];//右下
			MatrixXd Aq(8, 4);//论文公式(3)中的Aq
			Aq << p0.col, -p0.row, 1, 0,
				p0.row, p0.col, 0, 1,
				p1.col, -p1.row, 1, 0,
				p1.row, p1.col, 0, 1,
				p2.col, -p2.row, 1, 0,
				p2.row, p2.col, 0, 1,
				p3.col, -p3.row, 1, 0,
				p3.row, p3.col, 0, 1;
			MatrixXd Aq_trans = Aq.transpose();//Aq的转置矩阵
			MatrixXd Aq_trans_mul_Aq_reverse = (Aq_trans * Aq).inverse();// ((Aq^T)*Aq)^(-1)
			MatrixXd I = MatrixXd::Identity(8, 8);//8*8的单位矩阵
			MatrixXd coeff = (Aq*(Aq_trans_mul_Aq_reverse)*Aq_trans - I);
			//将当前网格的energy（8*8）放入到Shape_energy中
			int left_top_x = (row*numQuadCol + col) * 8;
			for (int i = 0; i < 8; i++) {
				for (int j = 0; j < 8; j++) {
					Shape_energy.insert(left_top_x + i, left_top_x + j) = coeff(i, j);
				}
			}
		}
	}
	Shape_energy.makeCompressed();
	return Shape_energy;
}

SpareseMatrixD_Row get_vertex_to_shape_mat(vector<vector<CoordinateDouble>> mesh, Config config) {
	int numLineRow = config.meshLineRow;
	int numLineCol = config.meshLineCol;
	int numQuadRow = config.meshQuadRow;
	int numQuadCol = config.meshQuadCol;
	SpareseMatrixD_Row Q(8 * numQuadRow*numQuadCol, 2 * numLineRow*numLineCol);
	//遍历每个网格
	for (int row = 0; row < numQuadRow; row++) {
		for (int col = 0; col < numQuadCol; col++) {
			int quadid = 8 * (row*numQuadCol + col);//当前网格编号
			int topleftvertexId = 2 * (row*numLineCol + col);//当前网格左上的网格点的编号
			Q.insert(quadid, topleftvertexId) = 1;
			Q.insert(quadid + 1, topleftvertexId + 1) = 1;
			Q.insert(quadid + 2, topleftvertexId + 2) = 1;
			Q.insert(quadid + 3, topleftvertexId + 3) = 1;
			Q.insert(quadid + 4, topleftvertexId + 2 * numLineCol) = 1;
			Q.insert(quadid + 5, topleftvertexId + 2 * numLineCol + 1) = 1;
			Q.insert(quadid + 6, topleftvertexId + 2 * numLineCol + 2) = 1;
			Q.insert(quadid + 7, topleftvertexId + 2 * numLineCol + 3) = 1;
		}
	}
	Q.makeCompressed();
	return Q;
}

bool between(double a, double X0, double X1) {
	double temp1 = a - X0;
	double temp2 = a - X1;
	if ((temp1<1e-8 && temp2>-1e-8) || (temp2<1e-6 && temp1>-1e-8)) {
		return true;
	}
	else {
		return false;
	}
}

Vector2d detectIntersect(Matrix2d line1, Matrix2d line2, bool& isintersection) {
	double line_x = 0, line_y = 0; //交点  
	double p1_x = line1(0, 1), p1_y = line1(0, 0), p2_x = line1(1, 1), p2_y = line1(1, 0);
	double p3_x = line2(0, 1), p3_y = line2(0, 0), p4_x = line2(1, 1), p4_y = line2(1, 0);
	if ((fabs(p1_x - p2_x) < 1e-6) && (fabs(p3_x - p4_x) < 1e-6)) {
		isintersection = false;
	}
	else if ((fabs(p1_x - p2_x) < 1e-6)) { //如果直线段p1p2垂直与y轴  
		if (between(p1_x, p3_x, p4_x)) {
			double k = (p4_y - p3_y) / (p4_x - p3_x);
			line_x = p1_x;
			line_y = k * (line_x - p3_x) + p3_y;

			if (between(line_y, p1_y, p2_y)) {
				isintersection = true;
			}
			else {
				isintersection = false;
			}
		}
		else {
			isintersection = false;
		}
	}
	else if ((fabs(p3_x - p4_x) < 1e-6)) { //如果直线段p3p4垂直与y轴  
		if (between(p3_x, p1_x, p2_x)) {
			double k = (p2_y - p1_y) / (p2_x - p1_x);
			line_x = p3_x;
			line_y = k * (line_x - p2_x) + p2_y;
			if (between(line_y, p3_y, p4_y)) {
				isintersection = true;
			}
			else {
				isintersection = false;
			}
		}
		else {
			isintersection = false;
		}
	}
	else {
		double k1 = (p2_y - p1_y) / (p2_x - p1_x);
		double k2 = (p4_y - p3_y) / (p4_x - p3_x);

		if (fabs(k1 - k2) < 1e-6) {
			isintersection = false;
		}
		else {
			line_x = ((p3_y - p1_y) - (k2*p3_x - k1 * p1_x)) / (k1 - k2);
			line_y = k1 * (line_x - p1_x) + p1_y;
		}

		if (between(line_x, p1_x, p2_x) && between(line_x, p3_x, p4_x)) {
			isintersection = true;
		}
		else {
			isintersection = false;
		}
	}
	Vector2d p;
	p << line_y, line_x;
	return p;
}

//不关注边缘线的检测
void revise_mask_for_lines(CVMat &mask) {
	int rows = mask.rows;
	int cols = mask.cols;
	//将边缘线都置白
	for (int row = 0; row < rows; row++) {
		mask.at<uchar>(row, 0) = 255;
		mask.at<uchar>(row, cols - 1) = 255;
	}
	for (int col = 0; col < cols; col++) {
		mask.at<uchar>(0, col) = 255;
		mask.at<uchar>(rows - 1, col) = 255;
	}
	CVMat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(8, 8));
	//对mask腐蚀
	cv::dilate(mask, mask, element);
	cv::dilate(mask, mask, element);
}

//判断点是否位于网格内
bool is_in_quad(CoordinateDouble point, CoordinateDouble topLeft, CoordinateDouble topRight,
	CoordinateDouble bottomLeft, CoordinateDouble bottomRight) {
	//点必须位于左网格线的右侧，上网格线的下侧，底网格线的上侧，左右格线的左侧

	//左网格线的右侧
	if (topLeft.col == bottomLeft.col) {//网格线平
		if (point.col < topLeft.col) {
			return false;
		}
	}
	else {//网格线斜
		double leftSlope = (topLeft.row - bottomLeft.row) / (topLeft.col - bottomLeft.col);//计算斜率
		double leftIntersect = topLeft.row - leftSlope * topLeft.col;//左截距
		double yOnLineX = (point.row - leftIntersect) / leftSlope;//计算网格线上point.row行的列坐标
		if (point.col < yOnLineX) {//点的列坐标必须在网格线point.row行的列坐标右侧
			return false;
		}
	}
	//右网格线的左侧
	if (topRight.col == bottomRight.col) {
		if (point.col > topRight.col) {
			return false;
		}
	}
	else {
		double rightSlope = (topRight.row - bottomRight.row) / (topRight.col - bottomRight.col);
		double rightIntersect = topRight.row - rightSlope * topRight.col;
		double yOnLineX = (point.row - rightIntersect) / rightSlope;
		if (point.col > yOnLineX) {
			return false;
		}
	}
	//上网格线的下侧
	if (topLeft.row == topRight.row) {
		if (point.row < topLeft.row) {
			return false;
		}
	}
	else {
		double topSlope = (topRight.row - topLeft.row) / (topRight.col - topLeft.col);
		double topIntersect = topLeft.row - topSlope * topLeft.col;
		double xOnLineY = topSlope * point.col + topIntersect;
		if (point.row < xOnLineY) {
			return false;
		}
	}
	//下网格线的上侧
	if (bottomLeft.row == bottomRight.row) {
		if (point.row > bottomLeft.row) {
			return false;
		}
	}
	else {
		double bottomSlope = (bottomRight.row - bottomLeft.row) / (bottomRight.col - bottomLeft.col);
		double bottomIntersect = bottomLeft.row - bottomSlope * bottomLeft.col;
		double xOnLineY = bottomSlope * point.col + bottomIntersect;
		if (point.row > xOnLineY) {
			return false;
		}
	}
	//如果符合上述所有条件，则该点在网格线内
	return true;
}

//判断线段两端是否位于遮罩的有效区域
bool line_in_mask(CVMat mask, LineD line) {
	if (mask.at<uchar>(line.row1, line.col1) == 0 && mask.at<uchar>(line.row2, line.col2) == 0) {
		return true;
	}
	return false;
}

//通过lsd来检测线段，并返回线段容器
vector<LineD> lsd_detect(CVMat src, CVMat mask) {
	//CVMat src2;
	//src.copyTo(src2);
	int rows = src.rows;//行数
	int cols = src.cols;//列数
	CVMat gray_img;
	cv::cvtColor(src, gray_img, CV_BGR2GRAY);//转换为灰度图
	double *image = new double[gray_img.rows*gray_img.cols];
	for (int row = 0; row < gray_img.rows; row++) {
		for (int col = 0; col < gray_img.cols; col++) {
			image[row*gray_img.cols + col] = gray_img.at<uchar>(row, col);
		}
	}
	vector<LineD> lines;
	double * out;
	int num_lines;
	//通过lsd进行线段检测，得到线段数num_lines以及每条线段对应的顶点坐标
	out = lsd(&num_lines, image, gray_img.cols, gray_img.rows);
	//遍历每条线段
	for (int i = 0; i < num_lines; i++) {
		//创建当前线段的对象
		LineD line(out[i * 7 + 1], out[i * 7 + 0], out[i * 7 + 3], out[i * 7 + 2]);
		//CoordinateDouble start(out[i * 7 + 1], out[i * 7 + 0]);
		//CoordinateDouble end(out[i * 7 + 3], out[i * 7 + 2]);
		if (line_in_mask(mask, line)) {
			lines.push_back(line);//线段有效，则放入容器中
		}

		//DrawLine(src, start, end);
		/*DrawLine(src, start, end);
		cv::namedWindow("Border", CV_WINDOW_AUTOSIZE);
		cv::imshow("Border", src);
		cv::waitKey(0);*/
	}

	/*cv::namedWindow("Border", CV_WINDOW_AUTOSIZE);
	cv::imshow("Border", src);
	cv::waitKey(0);
	*/
	return lines;
}

//计算线段和网格线的交点
bool does_segment_intersect_line(LineD lineSegment, double slope, double intersect,
	bool vertical, CoordinateDouble& intersectPoint) {
	// calculate line segment m and b
	double lineSegmentSlope = INF;
	//计算线段斜率
	if (lineSegment.col1 != lineSegment.col2) {
		lineSegmentSlope = (lineSegment.row2 - lineSegment.row1) / (lineSegment.col2 - lineSegment.col1);
	}
	//计算线段的截距
	double lineSegmentIntersect = lineSegment.row1 - lineSegmentSlope * lineSegment.col1;

	//如果线段斜率和方格线斜率相等，即两线平行
	if (lineSegmentSlope == slope) {
		//如果连截距也相等，则为同一条线
		if (lineSegmentIntersect == intersect) {
			intersectPoint.col = lineSegment.col1;
			intersectPoint.row = lineSegment.row1;
			return true;
		}
		else {//否则永远不会相交
			return false;
		}
	}
	//斜率不相等，则需要计算交点
	//交点的X坐标
	double intersectX = (intersect - lineSegmentIntersect) / (lineSegmentSlope - slope);
	//交点的Y坐标
	double intersectY = lineSegmentSlope * intersectX + lineSegmentIntersect;
	//检查交点是否在线段上（确保交点不是在线段的延长线上）
	if (vertical) {
		if ((intersectY <= lineSegment.row1 && intersectY >= lineSegment.row2) ||
			(intersectY <= lineSegment.row2 && intersectY >= lineSegment.row1)) {
			intersectPoint.col = intersectX;
			intersectPoint.row = intersectY;
			return true;
		}
		else {
			return false;
		}
	}
	else {
		if ((intersectX <= lineSegment.col1 && intersectX >= lineSegment.col2) ||
			(intersectX <= lineSegment.col2 && intersectX >= lineSegment.col1)) {
			intersectPoint.col = intersectX;
			intersectPoint.row = intersectY;
			return true;
		}
		else {
			return false;
		}
	}
}

//计算线段和网格的交点（为0个或1个或2个）
vector<CoordinateDouble> intersections_with_quad(LineD lineSegment, CoordinateDouble topLeft,
	CoordinateDouble topRight, CoordinateDouble bottomLeft,
	CoordinateDouble bottomRight) {

	vector<CoordinateDouble> intersections;//记录交点信息
	//和网格左线的交点
	double leftSlope = INF;
	if (topLeft.col != bottomLeft.col) {
		//计算网格左线斜率
		leftSlope = (topLeft.row - bottomLeft.row) / (topLeft.col - bottomLeft.col);
	}
	//左截距
	double leftIntersect = topLeft.row - leftSlope * topLeft.col;
	CoordinateDouble leftIntersectPoint;
	//计算网格左线（包含延长线）和线段的交点
	if (does_segment_intersect_line(lineSegment, leftSlope, leftIntersect, true, leftIntersectPoint)) {
		//存在交点，则检查交点是否在网格左线段内部（确保交点不是在网格左线段的延长线上）
		if (leftIntersectPoint.row >= topLeft.row && leftIntersectPoint.row <= bottomLeft.row) {
			intersections.push_back(leftIntersectPoint);//检查完毕，为两线段交点，则推入容器
		}
	}

	//和网格右线的交点
	double rightSlope = INF;
	if (topRight.col != bottomRight.col) {
		rightSlope = (topRight.row - bottomRight.row) / (topRight.col - bottomRight.col);
	}
	double rightIntersect = topRight.row - rightSlope * topRight.col;
	CoordinateDouble rightIntersectPoint;
	if (does_segment_intersect_line(lineSegment, rightSlope, rightIntersect, true, rightIntersectPoint)) {
		if (rightIntersectPoint.row >= topRight.row && rightIntersectPoint.row <= bottomRight.row) {
			intersections.push_back(rightIntersectPoint);
		}
	}

	//和网格上线的交点
	double topSlope = INF;
	if (topLeft.col != topRight.col) {
		topSlope = (topRight.row - topLeft.row) / (topRight.col - topLeft.col);
	}
	double topIntersect = topLeft.row - topSlope * topLeft.col;
	// check
	CoordinateDouble topIntersectPoint;
	if (does_segment_intersect_line(lineSegment, topSlope, topIntersect, false, topIntersectPoint)) {
		if (topIntersectPoint.col >= topLeft.col && topIntersectPoint.col <= topRight.col) {
			intersections.push_back(topIntersectPoint);
		}
	}

	//和网格下线的交点
	double bottomSlope = INF;
	if (bottomLeft.col != bottomRight.col) {
		bottomSlope = (bottomRight.row - bottomLeft.row) / (bottomRight.col - bottomLeft.col);
	}
	double bottomIntersect = bottomLeft.row - bottomSlope * bottomLeft.col;
	CoordinateDouble bottomIntersectPoint;
	if (does_segment_intersect_line(lineSegment, bottomSlope, bottomIntersect, false, bottomIntersectPoint)) {
		if (bottomIntersectPoint.col >= bottomLeft.col && bottomIntersectPoint.col <= bottomRight.col) {
			intersections.push_back(bottomIntersectPoint);
		}
	}

	return intersections;
}

//将线分段，置于每个网格中
vector<vector<vector<LineD>>> segment_line_in_quad(CVMat src, vector<LineD> lines, vector<vector<CoordinateDouble>> mesh, Config config) {
	//网格行数
	int QuadnumRow = config.meshQuadRow;
	//网格列数
	int QuadnumCol = config.meshQuadCol;
	vector<vector<vector<LineD>>> quad_line_seg;
	CVMat src2;
	src.copyTo(src2);
	//遍历每个网格
	for (int row = 0; row < QuadnumRow; row++) {
		vector<vector<LineD>> vec_row;
		for (int col = 0; col < QuadnumCol; col++) {
			//获取当前网格的四个顶点
			CoordinateDouble lefttop = mesh[row][col];
			CoordinateDouble righttop = mesh[row][col + 1];
			CoordinateDouble leftbottom = mesh[row + 1][col];
			CoordinateDouble rightbottom = mesh[row + 1][col + 1];

			vector<LineD> lineInQuad;
			//遍历每条线段
			for (int i = 0; i < lines.size(); i++) {
				LineD line = lines[i];
				//获取当前线段的两个顶点
				CoordinateDouble point1(line.row1, line.col1);
				CoordinateDouble point2(line.row2, line.col2);
				//判断当前线段的两个顶点是否在当前网格内
				bool p1InQuad = is_in_quad(point1, lefttop, righttop, leftbottom, rightbottom);
				bool p2InQuad = is_in_quad(point2, lefttop, righttop, leftbottom, rightbottom);
				//如果两个顶点都在网格内
				if (p1InQuad && p2InQuad) {
					lineInQuad.push_back(line);//直接推入容器
				}
				else if (p1InQuad) {//只有p1在网格内，则将线段进行拆分
					vector<CoordinateDouble> intersections = intersections_with_quad(line, lefttop, righttop, leftbottom, rightbottom);
					if (intersections.size() != 0) {//如果存在交点
						LineD cutLine(point1, intersections[0]);//则划分线段
						lineInQuad.push_back(cutLine);//将划分后的线段推入容器
					}
				}
				else if (p2InQuad) {
					vector<CoordinateDouble> intersections = intersections_with_quad(line, lefttop, righttop, leftbottom, rightbottom);
					if (intersections.size() != 0) {
						LineD cutLine(point2, intersections[0]);
						lineInQuad.push_back(cutLine);
					}
				}
				else {//p1和p2都不在网格内，则要么不交，要么跨越
					vector<CoordinateDouble> intersections = intersections_with_quad(line, lefttop, righttop, leftbottom, rightbottom);
					if (intersections.size() == 2) {
						LineD cutLine(intersections[0], intersections[1]);
						lineInQuad.push_back(cutLine);
					}
				}
			}
			//将当前网格内部的线段放入到当前行网格的容器中
			vec_row.push_back(lineInQuad);
			//TEST
			//DRAW BORDER
			//DrawLine(src, lefttop, righttop);
			//DrawLine(src, lefttop, leftbottom);
			//DrawLine(src, righttop, rightbottom);
			//DrawLine(src, leftbottom, rightbottom);
			/*for (int i = 0; i < lineInQuad.size(); i++) {
				LineD line = lineInQuad[i];
				DrawLine(src2, line);
			}
			cv::namedWindow("quad", CV_WINDOW_AUTOSIZE);
			cv::imshow("quad", src);
			cv::namedWindow("line", CV_WINDOW_AUTOSIZE);
			cv::imshow("line", src2);

			cv::waitKey(0);
			*/
		}
		//得到全部网格的线段容器
		quad_line_seg.push_back(vec_row);
	}
	return quad_line_seg;
}

//对线段容器进行降维，将存放在每个网格中的线段放入到一维容器中
void flatten(vector<vector<vector<LineD>>> lineSeg, vector<LineD>& line_vec, Config config) {
	int numQuadRow = config.meshQuadRow;
	int numQuadCol = config.meshQuadCol;
	for (int row = 0; row < numQuadRow; row++) {
		for (int col = 0; col < numQuadCol; col++) {
			for (int k = 0; k < lineSeg[row][col].size(); k++) {
				line_vec.push_back(lineSeg[row][col][k]);
			}
		}
	}
}

//初始化线段
vector<vector<vector<LineD>>> init_line_seg(CVMat src, CVMat mask, Config config, vector < LineD > &lineSeg_flatten,
	vector<vector<CoordinateDouble>> mesh, vector<pair<int, double>>&id_theta, vector<double> &rotate_theta) {
	double thetaPerbin = PI / 49;//每个容器都占一个转角
	//不检测边缘线
	revise_mask_for_lines(mask);
	//检测线段，并放入容器lines中
	vector<LineD> lines = lsd_detect(src, mask);
	//将容器内的线段进行拆分，分别放入到每个网格中，并存到容器lineSeg中
	vector<vector<vector<LineD>>> lineSeg = segment_line_in_quad(src, lines, mesh, config);
	//对线段容器进行降维，将存放在每个网格中的线段放入到一维容器中
	flatten(lineSeg, lineSeg_flatten, config);
	//遍历每条线段
	for (int i = 0; i < lineSeg_flatten.size(); i++) {
		LineD line = lineSeg_flatten[i];
		//计算反正切函数
		double theta = atan((line.row1 - line.row2) / (line.col1 - line.col2));
		//计算当前线段所属的bin编号
		int lineSegmentBucket = (int)round((theta + PI / 2) / thetaPerbin);
		assert(lineSegmentBucket < 50);//bin编号小于50
		//将对应的bin编号和theta值作为组合放入到id_theta中
		id_theta.push_back(make_pair(lineSegmentBucket, theta));
		rotate_theta.push_back(0);
	}
	return lineSeg;
}

//在原始矩阵origin的基础上再往下连接矩阵addin
SpareseMatrixD_Row block_diag(SpareseMatrixD_Row origin, MatrixXd addin, int QuadID, Config config) {
	int cols_total = 8 * config.meshQuadRow*config.meshQuadCol;
	SpareseMatrixD_Row res(origin.rows() + addin.rows(), cols_total);
	res.topRows(origin.rows()) = origin;

	int lefttop_row = origin.rows();
	int lefttop_col = 8 * QuadID;
	for (int row = 0; row < addin.rows(); row++) {
		for (int col = 0; col < addin.cols(); col++) {
			res.insert(lefttop_row + row, lefttop_col + col) = addin(row, col);
		}
	}
	res.makeCompressed();
	return res;
}

//获取当前网格的四个顶点的坐标值，转化为8*1的矩阵
VectorXd get_vertice(int row, int col, vector<vector<CoordinateDouble>> mesh) {//y0,x0,y1,x1...
	VectorXd Vq = VectorXd::Zero(8);
	CoordinateDouble p0 = mesh[row][col];//左上
	CoordinateDouble p1 = mesh[row][col + 1];//右上
	CoordinateDouble p2 = mesh[row + 1][col];//左下
	CoordinateDouble p3 = mesh[row + 1][col + 1];//右下
	Vq << p0.col, p0.row, p1.col, p1.row, p2.col, p2.row, p3.col, p3.row;
	return Vq;
}

//将权重转化为2*8的矩阵
MatrixXd BilinearWeightsToMatrix(BilinearWeights w) {
	MatrixXd mat(2, 8);
	double v1w = 1 - w.s - w.t + w.s*w.t;
	double v2w = w.s - w.s*w.t;
	double v3w = w.t - w.s*w.t;
	double v4w = w.s*w.t;
	mat << v1w, 0, v2w, 0, v3w, 0, v4w, 0,
		0, v1w, 0, v2w, 0, v3w, 0, v4w;
	return mat;
}

BilinearWeights get_bilinear_weights(CoordinateDouble point, Coordinate upperLeftIndices, vector<vector<CoordinateDouble>> mesh) {
	//获取当前网格的四个顶点坐标
	CoordinateDouble p1 = mesh[upperLeftIndices.row][upperLeftIndices.col]; // topLeft
	CoordinateDouble p2 = mesh[upperLeftIndices.row][upperLeftIndices.col + 1]; // topRight
	CoordinateDouble p3 = mesh[upperLeftIndices.row + 1][upperLeftIndices.col]; // bottomLeft
	CoordinateDouble p4 = mesh[upperLeftIndices.row + 1][upperLeftIndices.col + 1]; // bottomRight

	//test
	//cout << p1 << " " << p2 << endl << p3 << " " << p4 << endl<<point;

	//上网格线斜率
	double slopeTop = (p2.row - p1.row) / (p2.col - p1.col);
	//下网格线斜率
	double slopeBottom = (p4.row - p3.row) / (p4.col - p3.col);
	//左网格线斜率
	double slopeLeft = (p1.row - p3.row) / (p1.col - p3.col);
	//右网格线斜率
	double slopeRight = (p2.row - p4.row) / (p2.col - p4.col);

	double quadraticEpsilon = 0.01;
	//如果是长方形网格
	if (slopeTop == slopeBottom && slopeLeft == slopeRight) {

		//// method 3
		Matrix2d mat1;
		mat1 << p2.col - p1.col, p3.col - p1.col,
			p2.row - p1.row, p3.row - p1.row;

		MatrixXd mat2(2, 1);
		mat2 << point.col - p1.col, point.row - p1.row;

		MatrixXd matsolution = mat1.inverse()*mat2;
		BilinearWeights weights;
		weights.s = matsolution(0, 0);//获取matsolution(0,0)位置的值
		weights.t = matsolution(1, 0);//获取matsolution(1,0)位置的值
		return weights;
	}
	else if (slopeLeft == slopeRight) {//左右网格线平行
		// method 2
		double a = (p2.col - p1.col)*(p4.row - p3.row) - (p2.row - p1.row)*(p4.col - p3.col);
		double b = point.row*((p4.col - p3.col) - (p2.col - p1.col)) - point.col*((p4.row - p3.row) - (p2.row - p1.row)) + p1.col*(p4.row - p3.row) - p1.row*(p4.col - p3.col) + (p2.col - p1.col)*(p3.row) - (p2.row - p1.row)*(p3.col);
		double c = point.row*(p3.col - p1.col) - point.col*(p3.row - p1.row) + p1.col*p3.row - p3.col*p1.row;

		double s1 = (-1 * b + sqrt(b*b - 4 * a*c)) / (2 * a);
		double s2 = (-1 * b - sqrt(b*b - 4 * a*c)) / (2 * a);
		double s;
		if (s1 >= 0 && s1 <= 1) {
			s = s1;
		}
		else if (s2 >= 0 && s2 <= 1) {
			s = s2;
		}
		else {

			if ((s1 > 1 && s1 - quadraticEpsilon < 1) ||
				(s2 > 1 && s2 - quadraticEpsilon < 1)) {
				s = 1;
			}
			else if ((s1 < 0 && s1 + quadraticEpsilon > 0) ||
				(s2 < 0 && s2 + quadraticEpsilon > 0)) {
				s = 0;
			}
			else {
				// this case should not happen
				cerr << "   Could not interpolate s weight for coordinate (" << point.col << "," << point.row << ")." << endl;
				s = 0;
			}
		}

		double val = (p3.row + (p4.row - p3.row)*s - p1.row - (p2.row - p1.row)*s);
		double t = (point.row - p1.row - (p2.row - p1.row)*s) / val;
		double valEpsilon = 0.1; // 0.1 and 0.01 appear identical
		if (fabs(val) < valEpsilon) {
			// Py ~= Cy because Dy - Cy ~= 0. So, instead of interpolating with y, we use x.
			t = (point.col - p1.col - (p2.col - p1.col)*s) / (p3.col + (p4.col - p3.col)*s - p1.col - (p2.col - p1.col)*s);
		}

		BilinearWeights weights;
		weights.s = s;
		weights.t = t;
		return weights;
	}
	else {

		// method 1
		double a = (p3.col - p1.col)*(p4.row - p2.row) - (p3.row - p1.row)*(p4.col - p2.col);
		double b = point.row*((p4.col - p2.col) - (p3.col - p1.col)) - point.col*((p4.row - p2.row) - (p3.row - p1.row)) + (p3.col - p1.col)*(p2.row) - (p3.row - p1.row)*(p2.col) + (p1.col)*(p4.row - p2.row) - (p1.row)*(p4.col - p2.col);
		double c = point.row*(p2.col - p1.col) - (point.col)*(p2.row - p1.row) + p1.col*p2.row - p2.col*p1.row;

		double t1 = (-1 * b + sqrt(b*b - 4 * a*c)) / (2 * a);
		double t2 = (-1 * b - sqrt(b*b - 4 * a*c)) / (2 * a);
		double t;
		if (t1 >= 0 && t1 <= 1) {
			t = t1;
		}
		else if (t2 >= 0 && t2 <= 1) {
			t = t2;
		}
		else {
			if ((t1 > 1 && t1 - quadraticEpsilon < 1) ||
				(t2 > 1 && t2 - quadraticEpsilon < 1)) {
				t = 1;
			}
			else if ((t1 < 0 && t1 + quadraticEpsilon > 0) ||
				(t2 < 0 && t2 + quadraticEpsilon > 0)) {
				t = 0;
			}
			else {
				// this case should not happen
				cerr << "   Could not interpolate t weight for coordinate (" << point.col << "," << point.row << ")." << endl;
				t = 0;
			}
		}

		double val = (p2.row + (p4.row - p2.row)*t - p1.row - (p3.row - p1.row)*t);
		double s = (point.row - p1.row - (p3.row - p1.row)*t) / val;
		double valEpsilon = 0.1; // 0.1 and 0.01 appear identical
		if (fabs(val) < valEpsilon) {
			// Py ~= Ay because By - Ay ~= 0. So, instead of interpolating with y, we use x.
			s = (point.col - p1.col - (p3.col - p1.col)*t) / (p2.col + (p4.col - p2.col)*t - p1.col - (p3.col - p1.col)*t);
		}

		BilinearWeights weights;
		weights.s = clamp(s, 0, 1);
		weights.t = clamp(t, 0, 1);
		return weights;
	}
}

//获取line energy
SpareseMatrixD_Row get_line_mat(CVMat src, CVMat mask, vector<vector<CoordinateDouble>> mesh,
	vector<double>rotate_theta, vector<vector<vector<LineD>>> lineSeg, vector<pair<MatrixXd, MatrixXd>>& BilinearVec,
	Config config, int &linenum, vector<bool>& bad) {

	int linetmpnum = -1;
	int rows = config.rows;//行数
	int cols = config.cols;//列数
	int QuadnumRow = config.meshQuadRow;//网格行数
	int QuadnumCol = config.meshQuadCol;//网格列数

	//记录线段能量
	SpareseMatrixD_Row energy_line;
	//遍历每个网格
	for (int row = 0; row < QuadnumRow; row++) {
		for (int col = 0; col < QuadnumCol; col++) {
			//获取当前网格内的所有线段
			vector<LineD> linesegInquad = lineSeg[row][col];

			//记录当前网格ID
			int QuadID = row * QuadnumCol + col;
			if (linesegInquad.size() == 0) {
				continue;//网格内没线段，则跳过该网格
			}
			else {
				Coordinate topleft(row, col);
				MatrixXd C_row_stack(0, 8);
				/*if (linesegInquad.size() > 2) {
					cout << endl<<QuadID<<" "<< linesegInquad.size();
					system("pause");
				}*/
				//遍历当前网格内的所有线段
				for (int k = 0; k < linesegInquad.size(); k++) {
					linetmpnum++;
					LineD line = linesegInquad[k];

					CoordinateDouble linestart(line.row1, line.col1);//记录当前线段的起点
					CoordinateDouble lineend(line.row2, line.col2);//记录当前线段的终点

					//test
					//获取双线性插值权重
					BilinearWeights startWeight = get_bilinear_weights(linestart, topleft, mesh);//s t2n t t1n
					//将权重转化为矩阵
					MatrixXd start_W_mat = BilinearWeightsToMatrix(startWeight);//2*8
					BilinearWeights endWeight = get_bilinear_weights(lineend, topleft, mesh);
					MatrixXd end_W_mat = BilinearWeightsToMatrix(endWeight);
					//cout << startWeight.s << " " << startWeight.t << endl;//test
					//test
					VectorXd S = get_vertice(row, col, mesh);//8*1
					Vector2d ans = start_W_mat * S - Vector2d(linestart.col, linestart.row);//2*1
					Vector2d ans2 = end_W_mat * S - Vector2d(lineend.col, lineend.row);
					//计算矩阵范数
					if (ans2.norm() >= 0.0001 || ans.norm() >= 0.0001) {//error case
						bad.push_back(true);
						BilinearVec.push_back(make_pair(MatrixXd::Zero(2, 8), MatrixXd::Zero(2, 8)));
						continue;
					}
					assert(ans.norm() < 0.0001);
					assert(ans2.norm() < 0.0001);
					bad.push_back(false);
					//end test
					//system("pause");

					//获取当前线段的旋转角
					double theta = rotate_theta[linetmpnum];
					BilinearVec.push_back(make_pair(start_W_mat, end_W_mat));
					//论文中的旋转矩阵
					Matrix2d R;
					R << cos(theta), -sin(theta),
						sin(theta), cos(theta);
					//**输入的方向向量矩阵，input orientation vector of this line segment
					MatrixXd ehat(2, 1);
					ehat << line.col1 - line.col2, line.row1 - line.row2;
					MatrixXd tmp = (ehat.transpose()*ehat).inverse();
					Matrix2d I = Matrix2d::Identity();//单位矩阵
					MatrixXd C = R * ehat*tmp*(ehat.transpose())*(R.transpose()) - I;//论文中的C
					MatrixXd CT = C * (start_W_mat - end_W_mat);//将C乘上权重矩阵，当前CT为2*8矩阵
					C_row_stack = row_stack(C_row_stack, CT);//将CT连接上去，C_row_stack会记录当前网格下的所有线段的权重矩阵
				}
				energy_line = block_diag(energy_line, C_row_stack, QuadID, config);//往energy_line上不断连接C_row_stack，形成所有网格的线段的权重矩阵
			}
		}
	}
	linenum = linetmpnum;
	return energy_line;
}

vector < vector<vector<pair<int, double>>>> cal_theta(vector<vector<vector<Line_rotate>>> lineSeg, Config config) {
	vector < vector<vector<pair<int, double>>>> lineGroup;
	for (int row = 0; row < config.meshQuadRow; row++) {
		vector<vector<pair<int, double>>> row_vec;
		for (int col = 0; col < config.meshQuadCol; col++) {
			vector<pair<int, double>> vec;
			row_vec.push_back(vec);
		}
		lineGroup.push_back(row_vec);
	}
	double qstep = PI / 49;
	for (int row = 0; row < config.meshQuadRow; row++) {
		for (int col = 0; col < config.meshQuadCol; col++) {
			vector<Line_rotate> linevec = lineSeg[row][col];
			int linenum = linevec.size();
			for (int i = 0; i < linenum; i++) {
				Line_rotate line = linevec[i];
				Vector2d pstart = line.pstart;//y x
				Vector2d pend = line.pend;
				double theta = atan((pstart(0) - pend(0)) / (pstart(1) - pend(1)));
				int groupid = (int)(round((theta + PI / 2) / qstep) + 1);
				lineGroup[row][col].push_back(make_pair(groupid, theta));
			}
		}
	}
	return lineGroup;
}