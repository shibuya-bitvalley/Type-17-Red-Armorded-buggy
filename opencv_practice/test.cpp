#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "unistd.h"
#include <iostream>
using namespace std;
using namespace cv;

int main(void){
   cv::Mat src_img;
   cv::Mat first;
   src_img = cv::imread("picture/ubuntu-logo.png", 1);
   
   cv::Mat hoge;
   hoge = cv::imread("picture/linux_dekiru.jpeg", 1);
   
   // 画像が読み込まれなかったらプログラム終了
   if(src_img.empty() || hoge.empty()){
      cout << "image load error" << endl;
      return -1;
   }
   
   // 結果画像表示
   cv::namedWindow("Image", CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);

   //for(int i = 0; i < 10; i++){
      //if(i <= 4){
	 cv::resize(src_img, first, Size(), 0.5, 0.5);
         cv::imshow("Image", first);
         //cv::imshow("Image", src_img);
	 cv::waitKey(0);
      //}else{
	 //cv::imshow("Image", hoge);
	 //cv::waitKey(1);
      //}
      //cout << "time = "<< i << endl;
      //sleep(1);
   //}
   return 0;
}
