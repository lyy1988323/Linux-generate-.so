#include <iostream>
#include "machine_vision_process_v6_Liunx.h"
using namespace std; 
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
	return 0;
}

