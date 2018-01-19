#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "unistd.h"
#include <iostream>
using namespace std;

int main(void){
   cv::Mat src_img;
   src_img = cv::imread("../../picture/ubuntu-logo.png", 1);
   
   cv::Mat hoge;
   hoge = cv::imread("../../picture/linux_dekiru.jpeg", 1);
   
   // 画像が読み込まれなかったらプログラム終了
   if(src_img.empty() || hoge.empty()){
      cout << "image load error" << endl;
      return -1;
   }
   
   // 結果画像表示
   cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

   for(int i = 0; i < 10; i++){
      if(i <= 4){
	 cv::imshow("Image", src_img);
	 cv::waitKey(1);
	 //break;
      }else{
	 cv::imshow("Image", hoge);
	 cv::waitKey(1);
	 //break;
      }
      cout << "time = "<< i << endl;
      sleep(1);
   }
   return 0;
}
