#include"LocalWrapping.h"

bool cmp(const pair<int, float> a, const pair<int, float> b) {
	return a.second < b.second;
}

//�ж������е�ĳ���Ƿ�Ǻ�
bool Is_transparent(CVMat mask, int row, int col) {
	if (mask.at<uchar>(row, col) == 0) {
		return false;//����0��Ϊ��ɫ������
	}
	else {
		return true;//��������õ�Ǻ�
	}
}
//��ʼ��ƫ����������
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
//����������ص�����ֵ
CVMat Sobel_img(CVMat src) {
	CVMat gray;
	cv::cvtColor(src, gray, CV_BGR2GRAY);//��BGRͼתΪ�Ҷ�ͼ
	CVMat grad_x, grad_y, dst;
	//����Sobel���Ӽ���Ҷ�ͼ����ֵ
	cv::Sobel(gray, grad_x, CV_8U, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
	cv::Sobel(gray, grad_y, CV_8U, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT);
	//����Ȩ��
	addWeighted(grad_x, 0.5, grad_y, 0.5, 0, dst);
	return dst;
}

//ѡ����ı߽��ȱ���������border��[begin��end]
pair<int, int> Choose_longest_border(CVMat src, CVMat mask, Border& direction) {

	int maxLength = 0;
	int rows = src.rows;//ͼ������
	int cols = src.cols;//ͼ������

	int final_startIndex = 0;
	int final_endIndex = 0;


	//left
	int tmp_maxLength, tmp_startIndex, tmp_endIndex;
	tmp_maxLength = tmp_startIndex = tmp_endIndex = 0;
	bool isCounting = false;
	for (int row = 0; row < rows; row++) {
		colorPixel point = src.at<colorPixel>(row, 0);
		//������������еĺڵ�����Ѿ�һ��������ͷ�ˣ���ֹͣ����
		if (!Is_transparent(mask, row, 0) || row == rows - 1) {//rows-1Ϊ�����
			if (isCounting) {//������ڼ���
				if (Is_transparent(mask, row, 0)) {
					tmp_endIndex++;
					tmp_maxLength++;
					isCounting = true;
				}
				//������Ŀǰ������ȣ����¼�¼
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
		else {//�õ�͸������ʼ����
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
		colorPixel point = src.at<colorPixel>(row, cols - 1);//cols-1Ϊ���ұ�
		//������������еĺڵ�����Ѿ�һ��������ͷ�ˣ���ֹͣ����
		if (!Is_transparent(mask, row, cols - 1) || row == rows - 1) {
			if (isCounting) {//������ڼ���
				if (Is_transparent(mask, row, cols - 1)) {
					tmp_endIndex++;
					tmp_maxLength++;
					isCounting = true;
				}
				if (tmp_maxLength > maxLength) {//ĿǰΪ������и���
					maxLength = tmp_maxLength;
					final_startIndex = tmp_startIndex;
					final_endIndex = tmp_endIndex;
					direction = BORDER_RIGHT;
				}
			}
			//ֹͣ����
			isCounting = false;
			tmp_startIndex = tmp_endIndex = row;
			tmp_maxLength = 0;
		}
		else {//�õ�Ǻڣ���ʼ����
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
			if (isCounting) {//������ڼ���
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
		else {//�õ�͸������ʼ����
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
			if (isCounting) {//������ڼ���
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
		else {//�õ�͸������ʼ����
			tmp_endIndex++;
			tmp_maxLength++;
			isCounting = true;
		}
	}
	//cout << maxLength << endl;

	//cout << Is_transparent(src.at<Vec3b>(0,final_endIndex));
	//system("pause");
	//�����ҵ����ֵ
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
	//���оֲ���Seam Carving�㷨��ͬʱ�õ����յ�ƫ��������
	vector<vector<Coordinate>> displacementMap = Get_Local_wrap_displacement(src, mask);
	for (int row = 0; row < src.rows; row++) {
		for (int col = 0; col < src.cols; col++) {
			//��ȡĳ�����ص�λ��ƫ��
			Coordinate displacement = displacementMap[row][col];
			//��ȡƫ�Ƶ�����
			colorPixel pixel = src.at<colorPixel>(row + displacement.row, col + displacement.col);
			//�������طŵ���Ӧλ��
			wrap_img.at<colorPixel>(row, col) = pixel;
		}
	}
	return displacementMap;
}

//��ȡƫ��������
vector<vector<Coordinate>> Get_Local_wrap_displacement(CVMat src, CVMat mask) {

	int rows = src.rows;//����
	int cols = src.cols;//����
	//��¼ƫ�Ƶ���������
	vector<vector<Coordinate>> displacementMap;
	vector<vector<Coordinate>> finaldisplacementMap;
	//��ʼ����������
	init_displacement(finaldisplacementMap, rows, cols);
	init_displacement(displacementMap, rows, cols);
	//int cnt=0;

	while (true) {
		Border direction;
		//��ȡһ�����ȱ������¼�����ıߵ�λ��
		pair<int, int> begin_end = Choose_longest_border(src, mask, direction);
		//cout << direction<<endl;
		//test1 show longest border
		//Show_longest_border(src, begin_end, direction);

		//������������ȱ�ˣ��򷵻ؼ�¼ƫ�Ƶ���������
		if (begin_end.first == begin_end.second) {
			/*cv::namedWindow("Border", CV_WINDOW_AUTOSIZE);
			cv::imshow("Border", src);
			cv::waitKey(0);*/
			return displacementMap;
		}
		else {//ѭ������local warpping
			bool shift_to_end = false;
			SeamDirection seamdirection;
			switch (direction) {
				//�����ȱ������߻����ұߣ�������Ǵ�ֱ��
			case BORDER_LEFT:
				seamdirection = SEAM_VERTICAL;
				shift_to_end = false;
				break;
			case BORDER_RIGHT:
				seamdirection = SEAM_VERTICAL;
				shift_to_end = true;
				break;
				//�����ȱ�����ϱ߻����±ߣ��������ˮƽ��
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
			//ͨ����С������ֵ����ȡһ����С������
			int* seam = Get_local_seam(src, mask, seamdirection, begin_end);
			//cout << seamdirection << endl;
			//������С�����ߣ�ͬʱ����Ҫ��λ�����ؽ�����λ
			src = Insert_local_seam(src, mask, seam, seamdirection, begin_end, shift_to_end);


			//����ƫ�ƾ���
			for (int row = 0; row < rows; row++) {
				for (int col = 0; col < cols; col++) {
					Coordinate tmpdisplacement;
					//��ȱ����������Ŀǰλ�ھֲ�ͼ��
					if (seamdirection == SEAM_VERTICAL && row >= begin_end.first&&row <= begin_end.second) {
						int local_row = row - begin_end.first;
						//�����ȱ�����Ҳ࣬���������Ҳ�����ؼ������ƣ���¼ƫ��Ϊ-1����������������ΪFigure 3(ii)
						if (col > seam[local_row] && shift_to_end) {
							tmpdisplacement.col = -1;
						}
						//�����ȱ������࣬���������������ؼ������ƣ���¼ƫ��Ϊ1
						else {
							if (col < seam[local_row] && !shift_to_end) {
								tmpdisplacement.col = 1;
							}
						}
					}
					//����ͬ��
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
					//��ȡ��ǰ����ƫ�ƺ������
					int tmpdisplace_row = row + tmpdisplacement.row;
					int tmpdisplace_col = col + tmpdisplacement.col;
					//����ƫ�ƺ�����꣬��ȡ��������Ŀǰ�Ѿ�ƫ�Ƶ���
					Coordinate displacementOftarget = displacementMap[tmpdisplace_row][tmpdisplace_col];
					//����Ŀǰ��ƫ��������ȡ�������ԭʼ��������
					int rowInOrigin = tmpdisplace_row + displacementOftarget.row;
					int colInOrigin = tmpdisplace_col + displacementOftarget.col;
					//����ԭʼ�������꣬������������յ�ƫ�ƽ��
					finaldisplacement.row = rowInOrigin - row;
					finaldisplacement.col = colInOrigin - col;
				}
			}
			//��finalDisplacement�����ݸ�ֵ��displacement����ɶ�displacementƫ�ƾ���ĸ���
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
	//�������յ�ƫ�ƾ���
	return displacementMap;
}

//������С������
CVMat Insert_local_seam(CVMat src, CVMat& mask, int* seam, SeamDirection seamdirection, pair<int, int> begin_end, bool shiftToend) {
	//ͬ���Ƚ��з����ת��������������������������
	if (seamdirection == SEAM_HORIZONTAL) {
		transpose(src, src);
		transpose(mask, mask);
		//system("pause");
	}

	CVMat resimg;
	src.copyTo(resimg);

	int begin = begin_end.first;//��ʼ������
	int end = begin_end.second;//��ֹ������
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
	int rows = src.rows;//����
	int cols = src.cols;//����
	//�����ֲ�ͼ
	for (int row = begin; row <= end; row++) {
		int local_row = row - begin;//�ֲ�ͼ������
		//�����ȱ��������ϣ����������������ؼ�������
		for (int col = 0; col < seam[local_row]; col++) {
			if (!shiftToend) {
				colorPixel pixel = src.at<colorPixel>(row, col + 1);
				resimg.at<colorPixel>(row, col) = pixel;
				mask.at<uchar>(row, col) = mask.at<uchar>(row, col + 1);
			}
		}
		//�����ȱ���һ����£����������Ҳ�����ؼ�������
		for (int col = cols - 1; col > seam[local_row]; col--) {

			if (shiftToend) {
				colorPixel pixel = src.at<colorPixel>(row, col - 1);
				resimg.at<colorPixel>(row, col) = pixel;
				mask.at<uchar>(row, col) = mask.at<uchar>(row, col - 1);
			}
		}
		//�������ߵ����ؽ��и�ֵ
		mask.at<uchar>(row, seam[local_row]) = 0;
		//��������ˣ���ֱ��ȡ��λ���ص�ֵ
		if (seam[local_row] == 0) {
			resimg.at<colorPixel>(row, seam[local_row]) = src.at<colorPixel>(row, seam[local_row] + 1);
		}
		else {
			if (seam[local_row] == cols - 1) {
				resimg.at<colorPixel>(row, seam[local_row]) = src.at<colorPixel>(row, seam[local_row] - 1);
			}
			//������м䣬��ȡ������λ���ص�ƽ��ֵ����ƽ�����ɵ�����
			else {
				colorPixel pixel1 = src.at<colorPixel>(row, seam[local_row] + 1);
				colorPixel pixel2 = src.at<colorPixel>(row, seam[local_row] - 1);
				resimg.at<colorPixel>(row, seam[local_row]) = pixel1 / 2 + pixel2 / 2;
			}
		}
	}
	//�ղ�ת�����ˣ�������������ϵ�resimg��Ҫ���������ת��ȥ
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

//��ȡһ����С������
int* Get_local_seam(CVMat src, CVMat mask, SeamDirection seamdirection, pair<int, int> begin_end) {
	//�����ˮƽ�ģ����ͼ���򣬻����ͳһѰ����ֱ��seam
	if (seamdirection == SEAM_HORIZONTAL) {
		cv::transpose(src, src);
		cv::transpose(mask, mask);
	}

	int rows = src.rows;//����
	int cols = src.cols;//����

	int row_start = begin_end.first;//��ʼ������
	int row_end = begin_end.second;//��ֹ������

	int range = row_end - row_start + 1;//seam�����س���

	int col_start = 0;//��������
	int col_end = cols - 1;//��β����

	int outputWidth = cols;//������
	int outputHeight = range;//����߶�


	CVMat displayimg;
	src.copyTo(displayimg);
	//����seam�ߵĸߺͿ���ȡ�ֲ�ͼ�Լ��ֲ�����ͼ
	CVMat local_img = displayimg(cv::Range::Range(row_start, row_end + 1), cv::Range::Range(col_start, col_end + 1));
	CVMat local_mask = mask(cv::Range::Range(row_start, row_end + 1), cv::Range::Range(col_start, col_end + 1));//Range ����ҿ�
	//����Sobel���Ӽ���ֲ�ͼ������
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
	//�������ᵽ����ѡ��seam��ʱ�����ڿ�ȱ���ֵ���������Ϊ�������seam��ѡ�񵽿�ȱ����
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


	CVMat tmpenergy;//��¼����ͼ
	local_energy_32f.copyTo(tmpenergy);
	//����ͼ�н��ж�̬�滮(Dynamic Programming)����ȡ��С������
	for (int row = 1; row < range; row++) {
		for (int col = col_start; col <= col_end; col++) {
			//��һ�к����һ�п����������
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
	//��ȡ���һ�е�����ֵ
	vector<pair<int, float>> last_row;
	for (int col = col_start; col <= col_end; col++) {
		last_row.push_back(make_pair(col, tmpenergy.at<float>(range - 1, col)));
	}
	//�����һ�е�����ֵ��������
	sort(last_row.begin(), last_row.end(), cmp);
	int *seam = new int[range];
	//�õ������������һ�е�����
	seam[range - 1] = last_row[0].first;
	//������ǰ���������ߵ����أ�ֻ��Ҫ��������С�ľ�����
	for (int row = range - 2; row >= 0; row--) {
		//���׺���β���⿼��
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
			//����
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
	//����С�������ϵ����ص㸳ֵ
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

//��ȡÿ������������
vector<vector<CoordinateDouble>> get_rectangle_mesh(CVMat src, Config config) {
	//��ȡ�����ߵ��к���
	int meshnum_row = config.meshNumRow;
	int meshnum_col = config.meshNumCol;
	//��ȡÿ������ռ��ͼ�������������
	double row_per_mesh = config.rowPermesh;
	double col_per_mesh = config.colPermesh;
	vector<vector<CoordinateDouble>> mesh;
	for (int row_mesh = 0; row_mesh < meshnum_row; row_mesh++) {
		vector<CoordinateDouble> meshrow;
		for (int col_mesh = 0; col_mesh < meshnum_col; col_mesh++) {
			//��ȡ����������
			CoordinateDouble coord;
			coord.row = row_mesh * row_per_mesh;
			coord.col = col_mesh * col_per_mesh;
			meshrow.push_back(coord);
		}
		mesh.push_back(meshrow);
	}
	return mesh;
}

//����warp back
void wrap_mesh_back(vector<vector<CoordinateDouble>>& mesh, vector<vector<Coordinate>> displacementMap, Config config) {
	//��ȡ���������������
	int meshnum_row = config.meshNumRow;
	int meshnum_col = config.meshNumCol;
	//����ÿ�������
	for (int row_mesh = 0; row_mesh < meshnum_row; row_mesh++) {
		for (int col_mesh = 0; col_mesh < meshnum_col; col_mesh++) {
			//��Ϊ���һ�л����һ��������
			if (row_mesh == meshnum_row - 1 && col_mesh == meshnum_col - 1) {
				CoordinateDouble& meshVertexCoord = mesh[row_mesh][col_mesh];
				Coordinate vertexDisplacement = displacementMap[floor(meshVertexCoord.row) - 1][floor(meshVertexCoord.col) - 1];
				meshVertexCoord.row += vertexDisplacement.row;
				meshVertexCoord.col += vertexDisplacement.col;
			}
			//��ȡ��ǰ���������Ϣ
			CoordinateDouble& meshVertexCoord = mesh[row_mesh][col_mesh];
			//��ȡ��ǰ������ƫ����Ϣ
			Coordinate vertexDisplacement = displacementMap[(int)floor(meshVertexCoord.row)][(int)floor(meshVertexCoord.col)];
			//���ݵ�ǰ������ƫ����Ϣ����warp back���ˣ��õ�ԭʼͼ����������Ϣ
			meshVertexCoord.row += vertexDisplacement.row;
			meshVertexCoord.col += vertexDisplacement.col;
		}
	}
}