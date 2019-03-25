
#include "AdjustAngle.h"
int main()
{

	char* tempImg_Path = "/home/bubble/Desktop/work_20190321/source_pic";
	char* suorceImg_Path = "/home/bubble/Desktop/work_20190321/source_pic/srcImg.jpg";
	float shift_pX = 0, shift_pY = 0;
	int flag = -1;
	flag = AdjustAngle(tempImg_Path, suorceImg_Path, 5423, &shift_pX, &shift_pY);
	cout << "flag : " << flag << endl;
	cout << "shift_pX : " << shift_pX << endl;
	cout << "shift_pY : " << shift_pY << endl;


	waitKey(0);
}
