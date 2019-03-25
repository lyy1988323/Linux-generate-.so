#include"machine_vision_process_v6_Liunx.h"


typedef struct Match {
	bool Successflag;
	Mat match_H;
	Mat drawImg;
	vector<Point2f> objPoints;
}Matches;


//���ʶ�Ӧˮƽ�ӽ��б�
float fHorizonTable[33] = { 30.1, 28.6, 27.2, 25.9, 24.75, 23.44, 22.3, 21.13, 19.95, 18.95, 17.94, 16.91, 15.88, 14.83, 13.88, 12.82,
11.96, 11.1, 10.23, 9.36, 8.48, 7.82, 7.05, 6.38, 5.71, 5.16, 4.6, 4.04, 3.59, 3.26, 2.81, 2.36, 1.85 };

/**************************************************************************
*  ��������: Pshitf_angle
*  ����˵����������̨�Ƕ��뱶�ʹ�ϵ
*  �� �� �ߣ�����Ӣ
*  �� �� ֵ: ��
*  ��    ��: ����,ˮƽƫ��,��ֱƫ��,ˮƽ�Ƕȣ���ֱ�Ƕ�
**************************************************************************/
void Pshitf_angle(int nZoom, int offest_X, int offest_Y, float &Pshift_H, float &Pshift_V)
{
	nZoom = nZoom < 0 ? 0 : nZoom;
	nZoom = nZoom > 0x4000 ? 0x4000 : nZoom;  //16384

	double nZoomIndex = nZoom / 512.0;
	double dAngleH;
	if (nZoomIndex >= 32)
	{
		dAngleH = fHorizonTable[32];
	}
	else
	{
		int leftIndex = int(nZoomIndex);
		int rightIndex = int(nZoomIndex) + 1;
		dAngleH = fHorizonTable[leftIndex] + (nZoomIndex - int(nZoomIndex))*(fHorizonTable[rightIndex] - fHorizonTable[leftIndex]);
	}
	//cout << "dAngleH = " << dAngleH << endl;

	//ˮƽ����ľ�ƫ�Ƕ�
	float Ave_angleH = dAngleH / 960;
	Pshift_H = Ave_angleH * offest_X; //ˮƽ�����ƫ�Ƕ�
									  //Anlgle_H/960��ˮƽ����ÿ��������ռ�ĽǶ�ֵ
									  //cout << "Pshift_H = "<< Pshift_H << endl;

									  // fVerticalHorizonRatio����߱� 1080 �� 1920 = 0.5625
	double dAngleV = (dAngleH * 0.5625);
	//cout << "dAngleV = " << dAngleV << endl;

	//��ֱ����ľ�ƫ�Ƕ�
	float Ave_angleV = dAngleV / 540;
	Pshift_V = Ave_angleV * offest_Y;  //��ֱ�����ƫ�Ƕ�
									   //Anlgle_V/540����ֱ����ÿ��������ռ�ĽǶ�ֵ
									   //cout << "Pshift_V = "<< Pshift_V << endl;
}


//��ȡԭʼλ������
Point Get_OrigionPos_center(Mat temp_Img, Mat src_Img)
{
	int resultImg_cols = src_Img.cols - temp_Img.cols + 1;
	int resultImg_rows = src_Img.rows - temp_Img.rows + 1;
	Mat resultImg;
	double minValue, maxValue;
	Point minLoc, maxLoc;
	resultImg = Mat::zeros(resultImg_cols, resultImg_rows, CV_32FC1);   //��ʼ����
	matchTemplate(src_Img, temp_Img, resultImg, TM_CCOEFF_NORMED);
	minMaxLoc(resultImg, &minValue, &maxValue, &minLoc, &maxLoc);

	//ʶ����������
	Point matchCenter;
	matchCenter.x = maxLoc.x + temp_Img.cols / 2;
	matchCenter.y = maxLoc.y + temp_Img.rows / 2;
	rectangle(src_Img, Point(maxLoc.x, maxLoc.y), Point(maxLoc.x + temp_Img.cols, maxLoc.y + temp_Img.rows), Scalar(0, 255, 0), 2);

	/*imshow("matchTemplate", src_Img);*/
	return matchCenter;
}


/**************************************************************************
*  ��������: featureMatch
*  ����˵����match_Feedback�Ӻ��� ͼ��ƥ�� ��ȡ��Ӧ����
*  �� �� �ߣ�����Ӣ
*  �޸�����: 7��9��
*  �� �� ֵ: H����,��ʾͼ��,�Ƿ�ƥ��ɹ�flag
*  ��    ��: Ŀ��ͼ,����ͼ,Hessianֵ,H����,��ʾͼ��,�Ƿ�ƥ��ɹ�flag
**************************************************************************/
Matches featureMatch(Mat objectImg, Mat sceneImg, int Hessian, Matches matches)
{
	Matches Match = matches;
	//�ؼ�����
	Ptr<SurfFeatureDetector> detector = SurfFeatureDetector::create(100);
	vector<KeyPoint> keypoints_object, keypoints_scene;
	detector->detect(objectImg, keypoints_object);
	detector->detect(sceneImg, keypoints_scene);

	if (keypoints_scene.size()<10 || keypoints_object.size()<10)
	{
		Match.Successflag = false;
		return Match;
	}

	//���������ɣ�DLCO��
	//Ptr<VGG> vgg_descriptor = VGG::create();
	//Mat descriptors_object, descriptors_scene;
	//vgg_descriptor->compute(objectImg, keypoints_object, descriptors_object);
	//vgg_descriptor->compute(sceneImg, keypoints_scene, descriptors_scene);
	/*imshow("objectImg",objectImg);
	imshow("sceneImg",sceneImg);
	waitKey(0);*/

	//����detect��������SIFT�����ؼ��㣬������vector������
	Mat descriptors_object, descriptors_scene;
	detector->detectAndCompute(objectImg, Mat(), keypoints_object, descriptors_object);
	detector->detectAndCompute(sceneImg, Mat(), keypoints_scene, descriptors_scene);
	/*cout << "keypoint_object size:" << keypoints_object.size() << endl;
	cout << "keypoints_scene size:" << keypoints_scene.size() << endl;*/

	//ʹ��FLANNƥ�����ӽ���ƥ��
	FlannBasedMatcher matcher;
	vector<DMatch> mach;
	matcher.match(descriptors_object, descriptors_scene, mach);;

	//������ؼ���֮���������ֵ����Сֵ
	double Max_dist = 0.0;
	double Min_dist = 100000.0;
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		double dist = mach[i].distance;
		if (dist < Min_dist)Min_dist = dist;
		if (dist > Max_dist)Max_dist = dist;
	}

	int Min_dist_times = 2;         //��С���뱶������ʼ��Ϊ2��
	int Angle_filter_number = 0;    //��¼������˺�����Ŀ
	int Distance_filter_number = 0; //��¼������˺�����Ŀ
	while (Min_dist_times <= 5)
	{
		//������ؼ���֮���������ֵ����Сֵ
		vector<DMatch>good_matches;
		for (int i = 0; i < descriptors_object.rows; i++)
		{
			if (mach[i].distance < Min_dist_times * Min_dist)
				good_matches.push_back(mach[i]);
		}
		Mat img_maches;
		drawMatches(objectImg, keypoints_object, sceneImg, keypoints_scene, good_matches, img_maches);
		/*cout << " good_matches :" << good_matches.size() <<endl;*/
		/*imshow("img_maches",img_maches);
		waitKey(0);*/

		//����ƥ���ԵĽǶȺ;���
		vector<int> Keypoints_angle;
		vector<int> Keypoints_distance;
		for (unsigned int i = 0; i < good_matches.size(); i++)
		{
			Keypoints_angle.push_back(atan((keypoints_object[good_matches[i].queryIdx].pt.y - keypoints_scene[good_matches[i].trainIdx].pt.y)
				/ (keypoints_object[good_matches[i].queryIdx].pt.x - keypoints_scene[good_matches[i].trainIdx].pt.x)) * 180 / CV_PI + 0.5);

			Keypoints_distance.push_back(sqrt(pow(abs(keypoints_object[good_matches[i].queryIdx].pt.y - keypoints_scene[good_matches[i].trainIdx].pt.y), 2)
				+ pow(abs(keypoints_object[good_matches[i].queryIdx].pt.x - keypoints_scene[good_matches[i].trainIdx].pt.x), 2)) + 0.5);
		}
		/*imshow("img_maches", img_maches);
		waitKey(0);*/
		//���ݣ���ֹ��������֮��indexֵ����ң�
		vector<int> Keypoints_angle_colne = Keypoints_angle;
		vector<int> Keypoints_distance_colne = Keypoints_distance;

		//�ҳ�Ԫ��������ĽǶ�ֵ����Ŀ
		sort(Keypoints_angle.begin(), Keypoints_angle.end());
		int value_angle = 0;
		int value_angle_pre = 0;
		int angle_value = 0;
		int count_angle = 0;
		int min_value = 0;
		for (int i = 0; i < Keypoints_angle.size(); i++)
		{
			value_angle = Keypoints_angle[i];
			//cout << "value_angle = " << value_angle << endl;
			if (value_angle == value_angle_pre)
			{
				count_angle++;
				if (count_angle > min_value)
				{
					min_value = count_angle;
					angle_value = Keypoints_angle[i];
				}
			}
			else {
				count_angle = 0;
			}
			value_angle_pre = value_angle;
		}

		/* vector<int>::iterator it;
		for(it=Keypoints_angle.begin();it!=Keypoints_angle.end();it++)
		cout<<"it_angle:"<<*it<<endl;

		vector<int>::iterator distance_it;
		for(distance_it=Keypoints_distance.begin();distance_it!=Keypoints_distance.end();distance_it++)
		cout<<"it_distance:"<< *distance_it <<endl;;*/

		//cout<<"/******************************************/"<<endl;
		//cout << "angle_value = " << angle_value << endl;

		//�Ƕȹ��˺�����
		Angle_filter_number = 0;
		int angle_thresh = 2;  //�Ƕȷ�Χֵ
		vector<int>good_Keypoints_distance;
		vector<int>good_Keypoints_index;
		for (unsigned int i = 0; i < good_matches.size(); i++)
		{
			if ((Keypoints_angle_colne[i] >(angle_value - angle_thresh)) &&
				(Keypoints_angle_colne[i] < (angle_value + angle_thresh))) //ѡ���Ƕ���ȶ�Ӧ�ľ���
			{
				good_Keypoints_distance.push_back(Keypoints_distance_colne[i]);
				Angle_filter_number++;
				good_Keypoints_index.push_back(i);
				//cout << "it_angle"<<i<<":"<< Keypoints_angle_colne[i] <<endl;
				//cout << "Keypoints_distance_colne:" << Keypoints_distance_colne[i] << endl;
			}
		}
		if (Angle_filter_number < 12)
		{
			Min_dist_times++;
			Match.Successflag = false;
			continue;
		}
		//cout << "Angle_filter_number: " << Angle_filter_number <<endl;

		//����ȡ��
		sort(good_Keypoints_distance.begin(), good_Keypoints_distance.end());
		vector<int>good_Keypoints_distance_int;
		for (int i = 0; i < good_Keypoints_distance.size(); i++)
		{
			//cout << "good_Keypoints_distance : " << good_Keypoints_distance[i] << endl;
			good_Keypoints_distance_int.push_back(int(good_Keypoints_distance[i] / 10) * 10);
		}
		//ɸѡ�ľ�����д�С����
		sort(good_Keypoints_distance_int.begin(), good_Keypoints_distance_int.end());
		/*for (int i = 0; i < good_Keypoints_distance_int.size(); i++)
		{
		cout << "good_Keypoints_distance_int " <<good_Keypoints_distance_int[i]<<endl;
		}*/

		//ɸѡ���������ľ���ֵ
		int value_distance = 0;
		int value_distance_pre = 0;
		int distance_value = 0;
		int count_distance = 0;
		int min_distance_value = 0;
		int distanceIndex = 0;
		for (int i = 0; i < good_Keypoints_distance_int.size(); i++)
		{
			value_distance = good_Keypoints_distance_int[i];
			//cout << "value_distance = " << value_distance << endl;
			if (value_distance == value_distance_pre)
			{
				count_distance++;
				if (count_distance > min_distance_value)
				{
					min_distance_value = count_distance;
					distance_value = good_Keypoints_distance_int[i];
					distanceIndex = i;
				}
			}
			else {
				count_distance = 0;
			}
			value_distance_pre = value_distance;
		}

		//ѡ�����������Ŀ�����м�ֵ
		int middle_distance = good_Keypoints_distance[distanceIndex - (min_distance_value / 2.0)];
		//cout<<"middle_distance : " << good_Keypoints_distance[distanceIndex - (min_distance_value/2.0)] << endl;
		/*cout<<"min_distance_value = " << min_distance_value << endl;
		cout << "value_distance = " << distance_value << endl;*/
		//waitKey(0);

		//vector<int>::iterator distance_it;
		//for(distance_it=good_Keypoints_distance.begin();distance_it!=good_Keypoints_distance.end();distance_it++)
		//	cout<<"it_distance:"<< *distance_it <<endl;
		////���˾���
		//int middle_distance = good_Keypoints_distance[(int)(good_Keypoints_distance.size() / 2)]; //��ȡ������м�ֵ
		//cout << "middle_distance : " << middle_distance << endl;
		/*middle_distance = 460;*/
		//waitKey(0);
		int distance_thresh = 18;   //���뷶Χֵ
		vector<Point2f> obj;
		vector<Point2f> scene;
		vector<DMatch> best_matches;
		Distance_filter_number = 0;
		for (unsigned int i = 0; i < good_Keypoints_index.size(); i++)
		{
			if ((Keypoints_distance_colne[good_Keypoints_index[i]] >(middle_distance - distance_thresh)) &&
				(Keypoints_distance_colne[good_Keypoints_index[i]] < (middle_distance + distance_thresh)))
			{
				//cout << "it_diatance"<<Distance_filter_number<<":"<< Keypoints_distance_colne[good_Keypoints_index[i]] << endl;
				obj.push_back(keypoints_object[good_matches[good_Keypoints_index[i]].queryIdx].pt);
				scene.push_back(keypoints_scene[good_matches[good_Keypoints_index[i]].trainIdx].pt);
				best_matches.push_back(good_matches[good_Keypoints_index[i]]);
				Distance_filter_number++;
				/*cout << "Keypoints_distance_colne:" << Keypoints_distance_colne[i] << endl;*/
			}
		}
		Match.objPoints = obj;
		//cout << "Distance_filter_number: " << Distance_filter_number <<endl;

		//���㳡��ͼ��������ͬ�����������ͬ������ͬ����ͼ��ƥ�䲻�ϣ�
		int maxCnt = 1;
		for (int i = 0; i < scene.size(); i++) {
			int cnt = count(scene.begin(), scene.end(), scene[i]);
			if (cnt > maxCnt) {
				maxCnt = cnt;
			}
		}
		/*cout << "maxCnt : " << maxCnt << endl;*/
		int samePoint_scale = scene.size() / maxCnt;
		/*cout << "Same Point:" << samePoint_scale << endl;*/
		//С��12�������㷵��
		if (Distance_filter_number < 10 || samePoint_scale < 3)
		{
			Min_dist_times++;
			Match.Successflag = false;
			continue;
		}
		else
		{
			Match.Successflag = true;
			Match.match_H = findHomography(obj, scene, RANSAC);
			drawMatches(objectImg, keypoints_object, sceneImg, keypoints_scene, best_matches, Match.drawImg,
				Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			break;
		}
	}
	/*cout << "Min_dist_times=" << Min_dist_times << endl;
	cout << "Angle_filter_number:" << Angle_filter_number << endl;
	cout << "Distance_filter_number:" << Distance_filter_number << endl;*/
	/*imshow("Match.drawImg", Match.drawImg);
	waitKey(0);*/
	return Match;
}

/**************************************************************************
*  ��������: match_Feedback
*  ����˵������ȡƥ������
*  �� �� �ߣ�����Ӣ
*  �� �� ֵ: ƥ��ɹ�����1,���򷵻�-1
*  ��    ��: Ŀ��ͼ,����ͼ,ƥ�����ģ��ļ�Ŀ¼
**************************************************************************/
void match_Feedback(Mat &temp_Img, Mat &src_Img, Point &matchCenter)
{
	/************������ƥ��***********/
	Mat temp_Img_resize;
	Mat src_Img_resize;
	Matches temp_src_match;      //��ʼ��

	Mat temp_Img_clone = temp_Img.clone();
	Mat src_Img_clone = src_Img.clone();

	//ͼ�񽵲���
	resize(temp_Img, temp_Img_resize, Size(temp_Img.cols / 2, temp_Img.rows / 2));
	resize(src_Img, src_Img_resize, Size(src_Img.cols / 2, src_Img.rows / 2));
	temp_src_match = featureMatch(temp_Img_resize, src_Img_resize, 20, temp_src_match);
	//cout << "Match flag:" << temp_src_match.Successflag << endl;

	if (temp_src_match.Successflag && !temp_src_match.match_H.empty())   //����ƥ��ɹ�
	{
		//ģ��ͼ����
		int src_center_x = temp_Img_resize.cols / 2;
		int src_center_y = temp_Img_resize.rows / 2;

		//����objͼ����������
		long int objPoint_total_X = 0;
		long int objPoint_total_Y = 0;
		for (int i = 0; i < temp_src_match.objPoints.size(); i++)
		{
			objPoint_total_X += temp_src_match.objPoints[i].x;
			objPoint_total_Y += temp_src_match.objPoints[i].y;
		}
		Point2f objCenter_Keypoint;
		objCenter_Keypoint.x = objPoint_total_X / temp_src_match.objPoints.size();
		objCenter_Keypoint.y = objPoint_total_Y / temp_src_match.objPoints.size();

		//ģ��ͼ������objͼ���������ľ���
		Point2f srcCenter_objCenter_distance;
		srcCenter_objCenter_distance.x = src_center_x - objCenter_Keypoint.x;
		srcCenter_objCenter_distance.y = src_center_y - objCenter_Keypoint.y;

		//У����λ
		vector<Point2f> obj_corners(5);
		obj_corners[0] = Point(0, 0);
		obj_corners[1] = Point(temp_Img_resize.cols, 0);
		obj_corners[2] = Point(temp_Img_resize.cols, temp_Img_resize.rows);
		obj_corners[3] = Point(0, temp_Img_resize.rows);
		obj_corners[4] = objCenter_Keypoint;
		vector<Point2f> scene_corners(5);

		//����͸�ӱ任
		perspectiveTransform(obj_corners, scene_corners, temp_src_match.match_H);

		//���Ƴ��ǵ�֮���ֱ��
		Mat last_maches = temp_src_match.drawImg;
		Mat Keypoints_maches = last_maches.clone();
		line(Keypoints_maches, scene_corners[0] + Point2f(static_cast<float>(temp_Img_resize.cols), 0), scene_corners[1] + Point2f(static_cast<float>(temp_Img_resize.cols), 0), Scalar(255, 0, 0), 2);
		line(Keypoints_maches, scene_corners[1] + Point2f(static_cast<float>(temp_Img_resize.cols), 0), scene_corners[2] + Point2f(static_cast<float>(temp_Img_resize.cols), 0), Scalar(255, 0, 0), 2);
		line(Keypoints_maches, scene_corners[2] + Point2f(static_cast<float>(temp_Img_resize.cols), 0), scene_corners[3] + Point2f(static_cast<float>(temp_Img_resize.cols), 0), Scalar(255, 0, 0), 2);
		line(Keypoints_maches, scene_corners[3] + Point2f(static_cast<float>(temp_Img_resize.cols), 0), scene_corners[0] + Point2f(static_cast<float>(temp_Img_resize.cols), 0), Scalar(255, 0, 0), 2);
		//circle(last_maches, (scene_corners[4] + Point2f(static_cast<float>(temp_Img_resize.cols), 0)), 3, Scalar(255, 255, 0), -1);
		circle(last_maches, (scene_corners[4] + Point2f(static_cast<float>(temp_Img_resize.cols), 0)), 3, Scalar(0, 0, 255), -1);
		circle(last_maches, (scene_corners[4] + Point2f(static_cast<float>(temp_Img_resize.cols), 0) + srcCenter_objCenter_distance), 3, Scalar(255, 0, 0), -1);
		//circle(last_maches, obj_corners[4], 3, Scalar(255, 255, 0), -1);
		circle(last_maches, objCenter_Keypoint, 3, Scalar(0, 0, 255), -1);
		circle(last_maches, objCenter_Keypoint + srcCenter_objCenter_distance, 3, Scalar(255, 0, 0), -1);

		//imshow("Keypoints_maches", Keypoints_maches);

		//rectangle(last_maches, Point(scene_corners[4].x + src_center_x, scene_corners[4].y - src_center_y),
		//Point(scene_corners[4].x + 3*src_center_x, scene_corners[4].y + src_center_y), Scalar(255, 0, 255), 2, 8);

		//ƥ������(����ǰ�潵����һ������Ҫ��ԭ��������*2)
		matchCenter.x = (scene_corners[4].x + srcCenter_objCenter_distance.x) * 2;
		matchCenter.y = (scene_corners[4].y + srcCenter_objCenter_distance.y) * 2;

		//drawCross(last_maches, (scene_corners[4] + Point2f(static_cast<float>(temp_Img_resize.cols), 0) + srcCenter_objCenter_distance), Scalar(0, 255, 0), 30, 1);
		//͸�Ӻ�����
		rectangle(last_maches, Point(scene_corners[4].x + srcCenter_objCenter_distance.x + src_center_x, scene_corners[4].y + srcCenter_objCenter_distance.y - src_center_y),
			Point(scene_corners[4].x + srcCenter_objCenter_distance.x + 3 * src_center_x, scene_corners[4].y + srcCenter_objCenter_distance.y + src_center_y), Scalar(255, 0, 0), 2, 8);
		//imshow("last_macheImg22",last_maches);

		//drawCross(draw, Point(nObjCenterX, nObjCenterY), Scalar(0, 0, 255), 30, 1); //���ͼƥ������
		//drawCross(src_Img_clone, Point(Outdoor_PublicModels_XML.focusX0, Outdoor_PublicModels_XML.focusY0), Scalar(0, 255, 255), 30, 1); //ԭͼ����
		/*imshow("src_Img_clone", src_Img_clone);
		cout << "matchCenter.x : " << matchCenter.x << endl;
		cout << "matchCenter.y : " << matchCenter.y << endl;
		fprintf(fp,"->sift_Match matchCenter: [%d, %d]\n", matchCenter.x, matchCenter.y);*/

		/*	waitKey(0);
		waitKey(0);
		waitKey(0);
		waitKey(0);
		waitKey(0);
		waitKey(0);
		waitKey(0);
		waitKey(0);
		waitKey(0);*/
	}
	else  //ģ��ƥ��
	{
		int resultImg_cols = src_Img.cols - temp_Img.cols + 1;
		int resultImg_rows = src_Img.rows - temp_Img.rows + 1;
		Mat resultImg;
		double minValue, maxValue;
		Point minLoc, maxLoc;
		resultImg = Mat::zeros(resultImg_cols, resultImg_rows, CV_32FC1);   //��ʼ����
		matchTemplate(src_Img, temp_Img, resultImg, TM_CCOEFF_NORMED);
		minMaxLoc(resultImg, &minValue, &maxValue, &minLoc, &maxLoc);

		cout << "maxValue : " << maxValue << endl;
		//if (maxValue < 0.18)
		//	return -1;

		//ʶ����������
		matchCenter.x = maxLoc.x + temp_Img.cols / 2;
		matchCenter.y = maxLoc.y + temp_Img.rows / 2;

		//drawCross(src_Img_clone, matchCenter, Scalar(0, 0, 255), 30, 2);
		rectangle(src_Img_clone, Point(maxLoc.x, maxLoc.y), Point(maxLoc.x + temp_Img.cols, maxLoc.y + temp_Img.rows), Scalar(0, 255, 0), 2);
		/*imshow("matchImage222",src_Img_clone);*/

		resultImg.release();
	}
}

// �ѻ��㷨��ƫ����
//[in]szScriptPath:�ű�·��
//[in]szPicPath : ͼƬ·��
//[in]Zoom : ��һ���������
//[in|out]pX : ˮƽ����ƫ��ֵ
//[in|out]pY : ��ֱ����ƫ��ֵ
// ����ֵ
// �ɹ� 0
// ʧ�� - 1

int AdjustAngle(char *szScriptPath, char *szPicPath, int Zoom, float *pX, float *pY)
{
	*pX = 0;
	*pY = 0;
	char temp_file[256];
	sprintf(temp_file, "%s/target_0.bmp", szScriptPath);
	Mat temp_Img = imread(temp_file);
	if (temp_Img.empty()){
		cout << "temp_Img is empty!!!" << endl;
		return -1;
	}	

	//��ȡԭʼ����ֵ
	char source_file[256];
	sprintf(source_file, "%s/source_0.bmp", szScriptPath);
	Mat source_Img = imread(source_file);
	if (source_Img.empty()) {
		cout << "source_Img is empty!!!" << endl;
		return -1;
	}
	Point OrigionCenter; //ԭʼλ������
	OrigionCenter = Get_OrigionPos_center(temp_Img, source_Img);
	cout << "OrigionCenter : " << OrigionCenter << endl;
	
	//����ƥ������
	Mat src_Img = imread(szPicPath);
	if (src_Img.empty()){
		cout << "src_Img is empty!!!" << endl;
		return -1;
	}
	Point matchCenter;     //ƥ������
	match_Feedback(temp_Img, src_Img, matchCenter);
	cout << "matchCenter : " << matchCenter << endl;

	//������̨ƫ��Ƕ�
	//float Pshift_H, Pshift_V;
	int offest_X = matchCenter.x - OrigionCenter.x;
	int offest_Y = OrigionCenter.y - matchCenter.y;
	Pshitf_angle(Zoom, offest_X, offest_Y, *pX, *pY);

	cout << "------*pX : " << *pX << endl;
	cout << "------*pY : " << *pY << endl;

	return 0;
}


int main()
{
	
	char* tempImg_Path = "form4__0_2019-3-20-16-57-26";
	char* suorceImg_Path = "form4__0_2019-3-20-16-57-26/srcImg.jpg";
	float shift_pX = 0, shift_pY = 0;
	int flag = -1;
	flag = AdjustAngle(tempImg_Path, suorceImg_Path, 8192, &shift_pX, &shift_pY);
	cout << "flag : " << flag << endl;
	cout << "shift_pX : " << shift_pX << endl;
	cout << "shift_pY : " << shift_pY << endl;
	

	waitKey(0);
}
