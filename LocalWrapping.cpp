#include"LocalWrapping.h"

bool cmp(const pair<int, float> a, const pair<int, float> b) {
	return a.second < b.second;
}

//判断遮罩中的某点是否非黑
bool Is_transparent(CVMat mask, int row, int col) {
	if (mask.at<uchar>(row, col) == 0) {
		return false;//等于0，为黑色，不是
	}
	else {
		return true;//否则表明该点非黑
	}
}
//初始化偏移数组容器
void init_displacement(vector<vector<Coordinate>>& displacement, int rows, int cols) {
	for (int row = 0; row < rows; row++) {
		vector<Coordinate> displacement_row;
		for (int col = 0; col < cols; col++) {
			Coordinate c;
			displacement_row.push_back(c);
		}
		displacement.push_back(displacement_row);
	}
}
//计算各个像素的能量值
CVMat Sobel_img(CVMat src) {
	CVMat gray;
	cv::cvtColor(src, gray, CV_BGR2GRAY);//将BGR图转为灰度图
	CVMat grad_x, grad_y, dst;
	//根据Sobel因子计算灰度图能量值
	cv::Sobel(gray, grad_x, CV_8U, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::Sobel(gray, grad_y, CV_8U, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
	//考虑权重
	addWeighted(grad_x, 0.5, grad_y, 0.5, 0, dst);
	return dst;
}

//选择最长的边界空缺，并返回最长border的[begin，end]
pair<int, int> Choose_longest_border(CVMat src, CVMat mask, Border& direction) {

	int maxLength = 0;
	int rows = src.rows;//图像行数
	int cols = src.cols;//图像列数

	int final_startIndex = 0;
	int final_endIndex = 0;


	//left
	int tmp_maxLength, tmp_startIndex, tmp_endIndex;
	tmp_maxLength = tmp_startIndex = tmp_endIndex = 0;
	bool isCounting = false;
	for (int row = 0; row < rows; row++) {
		colorPixel point = src.at<colorPixel>(row, 0);
		//如果碰到遮罩中的黑点或者已经一条搜索到头了，则停止搜索
		if (!Is_transparent(mask, row, 0) || row == rows - 1) {//rows-1为最左边
			if (isCounting) {//如果还在计数
				if (Is_transparent(mask, row, 0)) {
					tmp_endIndex++;
					tmp_maxLength++;
					isCounting = true;
				}
				//超过了目前的最长长度，重新记录
				if (tmp_maxLength > maxLength) {
					maxLength = tmp_maxLength;
					final_startIndex = tmp_startIndex;
					final_endIndex = tmp_endIndex;
					direction = BORDER_LEFT;
				}
			}
			isCounting = false;
			tmp_startIndex = tmp_endIndex = row;
			tmp_maxLength = 0;
		}
		else {//该点透明，开始计数
			tmp_endIndex++;
			tmp_maxLength++;
			isCounting = true;
		}
	}
	//cout << maxLength << endl;

	//right
	tmp_maxLength = tmp_startIndex = tmp_endIndex = 0;
	isCounting = false;
	for (int row = 0; row < rows; row++) {
		colorPixel point = src.at<colorPixel>(row, cols - 1);//cols-1为最右边
		//如果碰到遮罩中的黑点或者已经一条搜索到头了，则停止搜索
		if (!Is_transparent(mask, row, cols - 1) || row == rows - 1) {
			if (isCounting) {//如果还在计数
				if (Is_transparent(mask, row, cols - 1)) {
					tmp_endIndex++;
					tmp_maxLength++;
					isCounting = true;
				}
				if (tmp_maxLength > maxLength) {//目前为最长，进行更新
					maxLength = tmp_maxLength;
					final_startIndex = tmp_startIndex;
					final_endIndex = tmp_endIndex;
					direction = BORDER_RIGHT;
				}
			}
			//停止计数
			isCounting = false;
			tmp_startIndex = tmp_endIndex = row;
			tmp_maxLength = 0;
		}
		else {//该点非黑，开始计数
			tmp_endIndex++;
			tmp_maxLength++;
			isCounting = true;
		}
	}
	//cout << maxLength << endl;

	//top
	tmp_maxLength = tmp_startIndex = tmp_endIndex = 0;
	isCounting = false;
	for (int col = 0; col < cols; col++) {
		colorPixel point = src.at<colorPixel>(0, col);
		//cout << col << " " << Is_transparent(point) << endl;
		if (!Is_transparent(mask, 0, col) || col == cols - 1) {
			if (isCounting) {//如果还在计数
				if (Is_transparent(mask, 0, col)) {
					tmp_endIndex++;
					tmp_maxLength++;
					isCounting = true;
				}
				if (tmp_maxLength > maxLength) {
					maxLength = tmp_maxLength;
					final_startIndex = tmp_startIndex;
					final_endIndex = tmp_endIndex;
					direction = BORDER_TOP;
				}
			}
			isCounting = false;
			tmp_startIndex = tmp_endIndex = col;
			tmp_maxLength = 0;
		}
		else {//该点透明，开始计数
			tmp_endIndex++;
			tmp_maxLength++;
			isCounting = true;
		}
	}
	//cout << maxLength << endl;

	//bottom
	tmp_maxLength = tmp_startIndex = tmp_endIndex = 0;
	isCounting = false;
	for (int col = 0; col < cols; col++) {
		colorPixel point = src.at<colorPixel>(rows - 1, col);
		if (!Is_transparent(mask, rows - 1, col) || col == cols - 1) {
			if (isCounting) {//如果还在计数
				if (Is_transparent(mask, rows - 1, col)) {
					tmp_endIndex++;
					tmp_maxLength++;
					isCounting = true;
				}
				if (tmp_maxLength > maxLength) {
					maxLength = tmp_maxLength;
					final_startIndex = tmp_startIndex;
					final_endIndex = tmp_endIndex;
					direction = BORDER_BOTTOM;
				}
			}
			isCounting = false;
			tmp_startIndex = tmp_endIndex = col;
			tmp_maxLength = 0;
		}
		else {//该点透明，开始计数
			tmp_endIndex++;
			tmp_maxLength++;
			isCounting = true;
		}
	}
	//cout << maxLength << endl;

	//cout << Is_transparent(src.at<Vec3b>(0,final_endIndex));
	//system("pause");
	//返回找到的最长值
	return make_pair(final_startIndex, final_endIndex - 1);
}

CVMat Show_longest_border(CVMat src, pair<int, int>begin_end, Border direction) {
	CVMat tmpsrc;
	src.copyTo(tmpsrc);
	int rows = src.rows;
	int cols = src.cols;
	switch (direction) {
	case BORDER_LEFT:
		for (int row = begin_end.first; row < begin_end.second; row++) {
			tmpsrc.at<colorPixel>(row, 0) = colorPixel(0, 0, 255);
		}
		break;
	case BORDER_RIGHT:
		for (int row = begin_end.first; row < begin_end.second; row++) {
			tmpsrc.at<colorPixel>(row, cols - 1) = colorPixel(0, 0, 255);
		}
		break;
	case BORDER_TOP:
		for (int col = begin_end.first; col < begin_end.second; col++) {
			tmpsrc.at<colorPixel>(0, col) = colorPixel(0, 0, 255);
		}
		break;
	case BORDER_BOTTOM:
		for (int col = begin_end.first; col < begin_end.second; col++) {
			tmpsrc.at<colorPixel>(rows - 1, col) = colorPixel(0, 0, 255);
		}
		break;
	default:
		break;
	}

	cv::namedWindow("Border", CV_WINDOW_AUTOSIZE);
	cv::imshow("Border", tmpsrc);
	cv::waitKey(0);

	return tmpsrc;
}

vector<vector<Coordinate>> Local_wrap(CVMat src, CVMat& wrap_img, CVMat mask) {
	//进行局部的Seam Carving算法，同时得到最终的偏移量矩阵
	vector<vector<Coordinate>> displacementMap = Get_Local_wrap_displacement(src, mask);
	for (int row = 0; row < src.rows; row++) {
		for (int col = 0; col < src.cols; col++) {
			//获取某个像素的位置偏移
			Coordinate displacement = displacementMap[row][col];
			//获取偏移的像素
			colorPixel pixel = src.at<colorPixel>(row + displacement.row, col + displacement.col);
			//将该像素放到对应位置
			wrap_img.at<colorPixel>(row, col) = pixel;
		}
	}
	return displacementMap;
}

//获取偏移量矩阵
vector<vector<Coordinate>> Get_Local_wrap_displacement(CVMat src, CVMat mask) {

	int rows = src.rows;//行数
	int cols = src.cols;//列数
	//记录偏移的数组容器
	vector<vector<Coordinate>> displacementMap;
	vector<vector<Coordinate>> finaldisplacementMap;
	//初始化数组容器
	init_displacement(finaldisplacementMap, rows, cols);
	init_displacement(displacementMap, rows, cols);
	//int cnt=0;

	while (true) {
		Border direction;
		//获取一条最长空缺，并记录其在四边的位置
		pair<int, int> begin_end = Choose_longest_border(src, mask, direction);
		//cout << direction<<endl;
		//test1 show longest border
		//Show_longest_border(src, begin_end, direction);

		//如果不存在最长空缺了，则返回记录偏移的数组容器
		if (begin_end.first == begin_end.second) {
			/*cv::namedWindow("Border", CV_WINDOW_AUTOSIZE);
			cv::imshow("Border", src);
			cv::waitKey(0);*/
			return displacementMap;
		}
		else {//循环进行local warpping
			bool shift_to_end = false;
			SeamDirection seamdirection;
			switch (direction) {
				//如果空缺线在左边或者右边，则缝线是垂直的
			case BORDER_LEFT:
				seamdirection = SEAM_VERTICAL;
				shift_to_end = false;
				break;
			case BORDER_RIGHT:
				seamdirection = SEAM_VERTICAL;
				shift_to_end = true;
				break;
				//如果空缺线在上边或者下边，则缝线是水平的
			case BORDER_TOP:
				seamdirection = SEAM_HORIZONTAL;
				shift_to_end = false;
				break;
			case BORDER_BOTTOM:
				seamdirection = SEAM_HORIZONTAL;
				shift_to_end = true;
				break;
			default:
				break;
			}
			//cout << seamdirection << endl;
			//通过最小化能量值，获取一条最小能量线
			int* seam = Get_local_seam(src, mask, seamdirection, begin_end);
			//cout << seamdirection << endl;
			//插入最小能量线，同时对需要移位的像素进行移位
			src = Insert_local_seam(src, mask, seam, seamdirection, begin_end, shift_to_end);


			//更新偏移矩阵
			for (int row = 0; row < rows; row++) {
				for (int col = 0; col < cols; col++) {
					Coordinate tmpdisplacement;
					//空缺线在左右且目前位于局部图内
					if (seamdirection == SEAM_VERTICAL && row >= begin_end.first&&row <= begin_end.second) {
						int local_row = row - begin_end.first;
						//如果空缺线在右侧，则能量线右侧的像素集体右移，记录偏移为-1，这在论文里体现为Figure 3(ii)
						if (col > seam[local_row] && shift_to_end) {
							tmpdisplacement.col = -1;
						}
						//如果空缺线在左侧，则能量线左侧的像素集体左移，记录偏移为1
						else {
							if (col < seam[local_row] && !shift_to_end) {
								tmpdisplacement.col = 1;
							}
						}
					}
					//上下同理
					else {
						if (seamdirection == SEAM_HORIZONTAL && col >= begin_end.first&&col <= begin_end.second) {
							int local_col = col - begin_end.first;
							if (row > seam[local_col] && shift_to_end) {
								tmpdisplacement.row = -1;
							}
							else {
								if (row < seam[local_col] && !shift_to_end) {
									tmpdisplacement.row = 1;
								}
							}
						}
					}
					Coordinate &finaldisplacement = finaldisplacementMap[row][col];
					//获取当前像素偏移后的坐标
					int tmpdisplace_row = row + tmpdisplacement.row;
					int tmpdisplace_col = col + tmpdisplacement.col;
					//根据偏移后的坐标，获取到该坐标目前已经偏移的量
					Coordinate displacementOftarget = displacementMap[tmpdisplace_row][tmpdisplace_col];
					//根据目前的偏移量，获取该坐标的原始行列坐标
					int rowInOrigin = tmpdisplace_row + displacementOftarget.row;
					int colInOrigin = tmpdisplace_col + displacementOftarget.col;
					//根据原始行列坐标，计算该像素最终的偏移结果
					finaldisplacement.row = rowInOrigin - row;
					finaldisplacement.col = colInOrigin - col;
				}
			}
			//将finalDisplacement的内容赋值给displacement，完成对displacement偏移矩阵的更新
			for (int row = 0; row < rows; row++) {
				for (int col = 0; col < cols; col++) {
					Coordinate &displacement = displacementMap[row][col];
					Coordinate finalDisplacement = finaldisplacementMap[row][col];
					displacement.row = finalDisplacement.row;
					displacement.col = finalDisplacement.col;
				}
			}//end for
		}//end else
	}
	//返回最终的偏移矩阵
	return displacementMap;
}

//插入最小能量线
CVMat Insert_local_seam(CVMat src, CVMat& mask, int* seam, SeamDirection seamdirection, pair<int, int> begin_end, bool shiftToend) {
	//同样先进行方向的转换，都看成竖的能量线来处理
	if (seamdirection == SEAM_HORIZONTAL) {
		transpose(src, src);
		transpose(mask, mask);
		//system("pause");
	}

	CVMat resimg;
	src.copyTo(resimg);

	int begin = begin_end.first;//起始行坐标
	int end = begin_end.second;//终止行坐标
	/*
	if(seamdirection=)
	for (int row = begin; row <= end; row++) {
	int local_row = row - begin;
	src.at<Vec3b>(row, seam[local_row]) = Vec3b(0, 0, 255);
	}
	namedWindow("insert_seam", CV_WINDOW_AUTOSIZE);
	imshow("insert_seam", src);
	waitKey(0);
	*/
	int rows = src.rows;//行数
	int cols = src.cols;//列数
	//遍历局部图
	for (int row = begin; row <= end; row++) {
		int local_row = row - begin;//局部图的行数
		//如果空缺在左或者上，在能量线左侧的像素集体左移
		for (int col = 0; col < seam[local_row]; col++) {
			if (!shiftToend) {
				colorPixel pixel = src.at<colorPixel>(row, col + 1);
				resimg.at<colorPixel>(row, col) = pixel;
				mask.at<uchar>(row, col) = mask.at<uchar>(row, col + 1);
			}
		}
		//如果空缺在右或者下，在能量线右侧的像素集体右移
		for (int col = cols - 1; col > seam[local_row]; col--) {

			if (shiftToend) {
				colorPixel pixel = src.at<colorPixel>(row, col - 1);
				resimg.at<colorPixel>(row, col) = pixel;
				mask.at<uchar>(row, col) = mask.at<uchar>(row, col - 1);
			}
		}
		//对能量线的像素进行赋值
		mask.at<uchar>(row, seam[local_row]) = 0;
		//如果在两端，则直接取邻位像素的值
		if (seam[local_row] == 0) {
			resimg.at<colorPixel>(row, seam[local_row]) = src.at<colorPixel>(row, seam[local_row] + 1);
		}
		else {
			if (seam[local_row] == cols - 1) {
				resimg.at<colorPixel>(row, seam[local_row]) = src.at<colorPixel>(row, seam[local_row] - 1);
			}
			//如果在中间，则取两侧邻位像素的平均值，起到平缓过渡的作用
			else {
				colorPixel pixel1 = src.at<colorPixel>(row, seam[local_row] + 1);
				colorPixel pixel2 = src.at<colorPixel>(row, seam[local_row] - 1);
				resimg.at<colorPixel>(row, seam[local_row]) = pixel1 / 2 + pixel2 / 2;
			}
		}
	}
	//刚才转过来了，由于最后插入完毕的resimg需要传出，因此转回去
	if (seamdirection == SEAM_HORIZONTAL) {
		cv::transpose(resimg, resimg);
		cv::transpose(mask, mask);
	}/*
	 else {
	 namedWindow("insert_seam3", CV_WINDOW_AUTOSIZE);
	 imshow("insert_seam3", mask);
	 waitKey(0);
	 }*/
	return resimg;
}

//获取一条最小能量线
int* Get_local_seam(CVMat src, CVMat mask, SeamDirection seamdirection, pair<int, int> begin_end) {
	//如果是水平的，则给图像换向，换向后，统一寻找竖直的seam
	if (seamdirection == SEAM_HORIZONTAL) {
		cv::transpose(src, src);
		cv::transpose(mask, mask);
	}

	int rows = src.rows;//行数
	int cols = src.cols;//列数

	int row_start = begin_end.first;//起始行坐标
	int row_end = begin_end.second;//终止行坐标

	int range = row_end - row_start + 1;//seam线像素长度

	int col_start = 0;//列首坐标
	int col_end = cols - 1;//列尾坐标

	int outputWidth = cols;//输出宽度
	int outputHeight = range;//输出高度


	CVMat displayimg;
	src.copyTo(displayimg);
	//根据seam线的高和宽，获取局部图以及局部遮罩图
	CVMat local_img = displayimg(cv::Range::Range(row_start, row_end + 1), cv::Range::Range(col_start, col_end + 1));
	CVMat local_mask = mask(cv::Range::Range(row_start, row_end + 1), cv::Range::Range(col_start, col_end + 1));//Range 左闭右开
	//根据Sobel算子计算局部图的能量
	CVMat local_energy = Sobel_img(local_img);
	CVMat local_energy_32f;
	local_energy.convertTo(local_energy_32f, CV_32F);
	//Mat local_energy_32f = get_energy(src);
	/*
	for (int row = 0; row < rows; row++) {
	for (int col = 0; col < cols; col++) {
	cout << local_energy_32f.at<float>(row, col)<<" ";
	}
	cout << endl;
	}*/
	//system("pause");
	//论文中提到，在选择seam线时，对于空缺部分的能量设置为无穷，避免seam线选择到空缺部分
	for (int row = 0; row < range; row++) {
		for (int col = col_start; col <= col_end; col++) {
			if ((int)local_mask.at<uchar>(row, col) == 255) {
				local_energy_32f.at<float>(row, col) = INF;
			}
		}
	}/*
	 namedWindow("local_seam2", CV_WINDOW_AUTOSIZE);
	 imshow("local_seam2", local_energy);
	 waitKey(0);
	 *//*
	 for (int row = 0; row < range; row++) {
	 for (int col = col_start; col <= col_end; col++) {
	 if (Is_transparent(local_img.at<Vec3b>(row, col)) == true) {
	 local_energy_32f.at<float>(row, col) = INF;
	 }
	 }
	 }*/


	CVMat tmpenergy;//记录能量图
	local_energy_32f.copyTo(tmpenergy);
	//在子图中进行动态规划(Dynamic Programming)，获取最小能量线
	for (int row = 1; row < range; row++) {
		for (int col = col_start; col <= col_end; col++) {
			//第一列和最后一列考虑特殊情况
			if (col == col_start) {
				tmpenergy.at<float>(row, col) += min(tmpenergy.at<float>(row - 1, col), tmpenergy.at<float>(row - 1, col + 1));
			}
			else {
				if (col == col_end) {
					tmpenergy.at<float>(row, col) += min(tmpenergy.at<float>(row - 1, col - 1), tmpenergy.at<float>(row - 1, col));
				}
				else {
					tmpenergy.at<float>(row, col) += min(tmpenergy.at<float>(row - 1, col), min(tmpenergy.at<float>(row - 1, col - 1), tmpenergy.at<float>(row - 1, col + 1)));
				}
			}
		}
	}
	//获取最后一行的能量值
	vector<pair<int, float>> last_row;
	for (int col = col_start; col <= col_end; col++) {
		last_row.push_back(make_pair(col, tmpenergy.at<float>(range - 1, col)));
	}
	//对最后一行的能量值进行排序
	sort(last_row.begin(), last_row.end(), cmp);
	int *seam = new int[range];
	//得到能量线在最后一行的像素
	seam[range - 1] = last_row[0].first;
	//逆序往前，找能量线的像素，只需要逆序找最小的就行了
	for (int row = range - 2; row >= 0; row--) {
		//列首和列尾特殊考虑
		if (seam[row + 1] == col_start) {
			if (tmpenergy.at<float>(row, seam[row + 1] + 1) < tmpenergy.at<float>(row, seam[row + 1])) {
				seam[row] = seam[row + 1] + 1;
			}
			else {
				seam[row] = seam[row + 1];
			}
		}
		else {
			if (seam[row + 1] == col_end) {
				if (tmpenergy.at<float>(row, seam[row + 1] - 1) < tmpenergy.at<float>(row, seam[row + 1])) {
					seam[row] = seam[row + 1] - 1;
				}
				else {
					seam[row] = seam[row + 1];
				}
			}
			//列中
			else {
				float min_energy = min(tmpenergy.at<float>(row, seam[row + 1] - 1), min(tmpenergy.at<float>(row, seam[row + 1]), tmpenergy.at<float>(row, seam[row + 1] + 1)));
				if (min_energy == tmpenergy.at<float>(row, seam[row + 1] - 1)) {
					seam[row] = seam[row + 1] - 1;
				}
				else {
					if (min_energy == tmpenergy.at<float>(row, seam[row + 1] + 1)) {
						seam[row] = seam[row + 1] + 1;
					}
					else {
						seam[row] = seam[row + 1];
					}
				}
			}
		}
	}
	/*
	for (int row = 0; row < range; row++) {
		local_energy_32f.at<float>(row, seam[row]) = INF;
	}*/
	//给最小能量线上的像素点赋值
	for (int row = 0; row < range; row++) {
		local_img.at<colorPixel>(row, seam[row]) = colorPixel(255, 255, 0);
	}


	/*
	cv::namedWindow("local_seam", CV_WINDOW_AUTOSIZE);
	cv::imshow("local_seam", local_img);
	//cv::namedWindow("local_seam2", CV_WINDOW_AUTOSIZE);
	//cv::imshow("local_seam2", mask);
	cv::waitKey(0);
	//system("pause");
	*/
	return seam;
}

//获取每个网格点的坐标
vector<vector<CoordinateDouble>> get_rectangle_mesh(CVMat src, Config config) {
	//获取网格线的行和列
	int meshnum_row = config.meshNumRow;
	int meshnum_col = config.meshNumCol;
	//获取每个网格占有图像的行数和列数
	double row_per_mesh = config.rowPermesh;
	double col_per_mesh = config.colPermesh;
	vector<vector<CoordinateDouble>> mesh;
	for (int row_mesh = 0; row_mesh < meshnum_row; row_mesh++) {
		vector<CoordinateDouble> meshrow;
		for (int col_mesh = 0; col_mesh < meshnum_col; col_mesh++) {
			//获取网格点的坐标
			CoordinateDouble coord;
			coord.row = row_mesh * row_per_mesh;
			coord.col = col_mesh * col_per_mesh;
			meshrow.push_back(coord);
		}
		mesh.push_back(meshrow);
	}
	return mesh;
}

//进行warp back
void wrap_mesh_back(vector<vector<CoordinateDouble>>& mesh, vector<vector<Coordinate>> displacementMap, Config config) {
	//读取网格的行数与列数
	int meshnum_row = config.meshNumRow;
	int meshnum_col = config.meshNumCol;
	//遍历每个网格点
	for (int row_mesh = 0; row_mesh < meshnum_row; row_mesh++) {
		for (int col_mesh = 0; col_mesh < meshnum_col; col_mesh++) {
			//若为最后一行或最后一列网格线
			if (row_mesh == meshnum_row - 1 && col_mesh == meshnum_col - 1) {
				CoordinateDouble& meshVertexCoord = mesh[row_mesh][col_mesh];
				Coordinate vertexDisplacement = displacementMap[floor(meshVertexCoord.row) - 1][floor(meshVertexCoord.col) - 1];
				meshVertexCoord.row += vertexDisplacement.row;
				meshVertexCoord.col += vertexDisplacement.col;
			}
			//获取当前的网格点信息
			CoordinateDouble& meshVertexCoord = mesh[row_mesh][col_mesh];
			//获取当前网格点的偏移信息
			Coordinate vertexDisplacement = displacementMap[(int)floor(meshVertexCoord.row)][(int)floor(meshVertexCoord.col)];
			//根据当前网格点的偏移信息进行warp back回退，得到原始图像的网格点信息
			meshVertexCoord.row += vertexDisplacement.row;
			meshVertexCoord.col += vertexDisplacement.col;
		}
	}
}