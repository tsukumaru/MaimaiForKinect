#include "Kinect.h"

#include <iostream>
#include <sstream>

#include <Windows.h>
#include <NuiApi.h>
#include <stdint.h>
#include <time.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <fstream>
#include <list>

#define _USE_MATH_DEFINES
#include <math.h>



/*#define ERROR_CHECK(ret)\
if (ret != S_OK){\
	std::stringstream ss; \
	ss << "failed " #ret "" << std::hex << ret << std::endl; \
	throw std::runtime_error(ss.str().c_str());\
}*/

#define TAP 1
#define EACH 2
#define HOLD 3
#define SLIDE 4

/*const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

INuiSensor *kinect;

HANDLE imageStreamHandle = INVALID_HANDLE_VALUE;
HANDLE depthStreamHandle = INVALID_HANDLE_VALUE;

HANDLE streamEvent;

DWORD width;
DWORD height;

const int minDepth = NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
const int maxDepth = NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT;*/

enum{
	NONE,
	GET_HANDS,
	GAME_START
};
int processFlag;

//データを扱う行列
cv::Mat rgbIm;
cv::Mat playerIm;
cv::Mat skeletonIm;
cv::Mat maimaiIm;

int scale;
//全体のフレーム数
int maiframe=0;
int loadflag = 1;
int maiWidth, maiHeight, radius;

//配列の要素数を求めるテンプレート関数
template
<
	typename TYPE,
	std::size_t SIZE
>
std::size_t array_length(const TYPE(&)[SIZE]){
	return SIZE;
}


/*class Kinect{
public:
	Kinect(){

	}
	void initKinect(){
		int count = 0;
		ERROR_CHECK(::NuiGetSensorCount(&count));
		if (count == 0){
			throw std::runtime_error("Kinectを接続してください");
		}

		//最初のインスタンス作成
		ERROR_CHECK(::NuiCreateSensorByIndex(0, &kinect));

		//Kinectの状態を取得
		HRESULT status = kinect->NuiStatus();
		if (status != S_OK){
			throw std::runtime_error("Kinectが利用可能ではありません");
		}

		//スケルトンデータを取得ために、NuiInitializeのフラグを追加しましょう！
		ERROR_CHECK(kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON));

		//RGB初期化
		ERROR_CHECK(kinect->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_COLOR, CAMERA_RESOLUTION, 0, 2, 0, &imageStreamHandle));

		//Depth初期化
		ERROR_CHECK(kinect->NuiImageStreamOpen(
			NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, CAMERA_RESOLUTION, 0, 2, 0, &depthStreamHandle));
		ERROR_CHECK(kinect->NuiImageStreamSetImageFrameFlags(depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE));

		//skeleton初期化
		//NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT を入れると、上半身のみ読み込む
		kinect->NuiSkeletonTrackingEnable(0,
			NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE);// | NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);

		//フレーム更新イベントの作成
		streamEvent = ::CreateEventA(0, TRUE, FALSE, 0);
		ERROR_CHECK(kinect->NuiSetFrameEndEvent(streamEvent, 0));

		::NuiImageResolutionToSize(CAMERA_RESOLUTION, width, height);
	}

	void setRgbImage()
	{
		// RGBカメラのフレームデータを取得する
		NUI_IMAGE_FRAME imageFrame = { 0 };

		ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(
			imageStreamHandle, 0, &imageFrame));

		//画像取得
		NUI_LOCKED_RECT colorData;
		imageFrame.pFrameTexture->LockRect(0, &colorData, 0, 0);

		rgbIm = cv::Mat(height, width, CV_8UC4, colorData.pBits);
		imageFrame.pFrameTexture->UnlockRect(0);

		//フレーム解放
		ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(
			imageStreamHandle, &imageFrame));
	}


	void setDepthImage()
	{
		/* 処理を記述 
		NUI_IMAGE_FRAME depthFrame = { 0 };
		ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame));

		//距離画像取得
		NUI_LOCKED_RECT depthData = { 0 };
		depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

		USHORT* depth = (USHORT*)depthData.pBits;
		playerIm = cv::Mat::zeros(height, width, CV_8UC1);
		LONG colorX = 0;
		LONG colorY = 0;
		for (LONG depthY = 0; depthY < height; depthY++) {
			for (LONG depthX = 0; depthX < width; depthX++) {

				//i番目の画素の距離値を取り出す
				//USHORT distance = ::NuiDepthPixelToDepth(*depth);

				//i番目の画素の人物情報を取り出す
				USHORT player = ::NuiDepthPixelToPlayerIndex(*depth);

				kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, depthX, depthY, *depth, &colorX, &colorY);
				colorX = (colorX >= width) ? width - 1 : (colorX<0 ? 0 : colorX);
				colorY = (colorY >= height) ? height - 1 : (colorY<0 ? 0 : colorY);
				playerIm.at<UCHAR>(colorY, colorX) = (player != 0 ? 255 : 0);

				depth++;
			}
		}
		depthFrame.pFrameTexture->UnlockRect(0);
		ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame));

		cv::dilate(playerIm, playerIm, cv::Mat(), cv::Point(-1, -1), 3);
		cv::erode(playerIm, playerIm, cv::Mat(), cv::Point(-1, -1), 3);
	}

	void setSkeletonImage(){
		skeletonIm = cv::Mat::zeros(height, width, CV_8UC3);
		cv::cvtColor(rgbIm, rgbIm, CV_BGRA2BGR);
		rgbIm.copyTo(skeletonIm, playerIm);
	}

	void drawPoint(cv::Mat& image, Vector4 position, int scale)
	{
		// ３次元の位置から距離画像での位置に変換
		FLOAT depthX = 0, depthY = 0;
		NuiTransformSkeletonToDepthImage(position, &depthX, &depthY, CAMERA_RESOLUTION);

		// 距離画像での位置からRGB画像での位置に変換
		LONG colorX = 0;
		LONG colorY = 0;
		kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
			CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0,
			(LONG)depthX, (LONG)depthY, 0, &colorX, &colorY);

		// RGB画像での位置に丸を描画
		cv::circle(image, cv::Point(colorX*scale, colorY*scale), 60, cv::Scalar(0, 255, 0), 5);
	}
};*/


class Effect{
private:
	int kind;
	int location[10][10]; //x,y
	int frame;//始まりから終わりまでのフレーム数
	int judge = 0;
public:
	int getJudge(){
		return judge;
	}

	void judgeInc(){
		judge++;
	}

	void draw(){
		if (0 <= judge && judge <= 5)
			std::cout << "FAST!!!!" << std::endl;
		else if (6 <= judge && judge <= 11)
			std::cout << "Perfect!!!" << std::endl;
		else if (12 <= judge && judge <= 17)
			std::cout << "Late!!!!" << std::endl;
		else
			std::cout << "MISS!!!!" << std::endl;
	}
};

class Note{
private:
	int kind; //ノートは4種類
	int dir; //方向
	int frame; //タイミング
	int noteframe=0;
	int tapradius=30;
	int drawflag = 1;
	double location[2]; //x,y
	Effect e;
public:
	void setKind(int k){
		kind = k;
	}
	void setDir(int d){
		dir = d;
	}
	void setFrame(int f){
		frame = f;
	}

	int getKind(){
		return kind;
	}
	int getDir(){
		return dir;
	}

	int getLocation(int index){
		return location[index];
	}

	boolean canmove(){
		if (drawflag)
			return this->frame <= maiframe;
		else
			return false;
	}
	//消していいかどうか
	boolean isDeleted(){
		return false;
	}

	void draw(){
		location[0] = maiWidth / 2 + (noteframe * 10)*cos(dir* M_PI / 4 - M_PI / 2 - M_PI / 8);
		location[1] = maiHeight / 2 + (noteframe * 10)*sin(dir*M_PI / 4 - M_PI / 2 - M_PI / 8);
		switch (kind){
		case TAP:
			cv::circle(maimaiIm, cv::Point(location[0], location[1]), tapradius, cv::Scalar(255,20,255), 6, 4);
			break;
		case EACH:
			printf("error\n");
			break;
		case HOLD:
			printf("error\n");
			break;
		case SLIDE:
			printf("error\n");
			break;
		}

		noteframe++;
	}

	void drawCansel(){
		drawflag = 0;
		location[0] = 1000;
		location[1] = 1000;
	}

	void countEffect(){
		e.judgeInc();
	}

	void drawEffect(){
		e.draw();
	}

	Effect getEffect(){
		return e;
	}
};

class Scene{
private:
	int speed;
	int notenum;
	int smallCircleRad=10;
	int Effect[4]; //エフェクトはPerfect,Great,Good,TooLateの4つ
	int circlePos[9][2];
	//譜面を読み込む
	Note notes; 
	Note score[100];
	Vector4 pos[2];//左右の手の座標
public :
	void setScene(){
		scale = 2;
		cv::resize(rgbIm, maimaiIm, cv::Size(), scale, scale, cv::INTER_NEAREST);
		setCircumCircle();
	}
	void setCircumCircle(){
		maiWidth = width * scale;
		maiHeight = height * scale;
		radius;
		if (maiWidth / 2 >= maiHeight / 2) radius = maiHeight / 2 - 50;
		else radius = maiWidth / 2 - 50;
		cv::circle(maimaiIm, cv::Point(maiWidth / 2, maiHeight / 2), radius, cv::Scalar(150, 0, 150), 6, 4);
		//小円の表示
		for (int i = 1; i <= 8; i++){
			//8つの小さい円を表示させる
			cv::circle(maimaiIm, cv::Point(maiWidth / 2 + (radius)*cos(i*M_PI / 4 -M_PI/2 - M_PI / 8), maiHeight / 2 + (radius)*sin(i*M_PI / 4 -M_PI/2 - M_PI / 8)), smallCircleRad, cv::Scalar(150, 0, 150), 6, 4);
			circlePos[i][0] = maiWidth / 2 + radius*cos(i*M_PI / 4 - M_PI / 2 - M_PI / 8);
			circlePos[i][1] = maiHeight / 2 + radius*sin(i*M_PI / 4 - M_PI / 2 - M_PI / 8);
		}
	}

	void createNote(Note *score,int notenum){
		//std::cout << sizeof score / sizeof score[0] << std::endl;
		for (int i = 0; i < notenum; i++){
			if (score[i].canmove()){
				//moveNote(score[i], score[i].getKind());
				score[i].draw();
			}
		}
	}

	Vector4 getHandsPosition(int index){
		// ３次元の位置から距離画像での位置に変換
		FLOAT depthX = 0, depthY = 0;
		NuiTransformSkeletonToDepthImage(pos[index], &depthX, &depthY, CAMERA_RESOLUTION);

		// 距離画像での位置からRGB画像での位置に変換
		LONG colorX = 0;
		LONG colorY = 0;
		kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
			CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0,
			(LONG)depthX, (LONG)depthY, 0, &colorX, &colorY);
		Vector4 p;
		p.x = colorX * 2;
		p.y = colorY * 2;
		return p;
	}

	void getHands(){
		Kinect k;
		NUI_SKELETON_FRAME skeletonFrame = { 0 };
		kinect->NuiSkeletonGetNextFrame(0, &skeletonFrame);

		for (int i = 0; i < NUI_SKELETON_COUNT; i++){
			const NUI_SKELETON_DATA &skeleton = skeletonFrame.SkeletonData[i];

			switch (skeleton.eTrackingState)
			{
			case NUI_SKELETON_TRACKED: //詳細スケルトンデータを得られる
			{
										   //if (processFlag == GET_HANDS) {
										   int handsPosition[] = { NUI_SKELETON_POSITION_HAND_LEFT, NUI_SKELETON_POSITION_HAND_RIGHT };
										   int handsPosSize = sizeof(handsPosition) / sizeof(handsPosition[0]);
										   for (int j = 0; j < handsPosSize; j++){
											   if (skeleton.eSkeletonPositionTrackingState[handsPosition[j]] == NUI_SKELETON_POSITION_TRACKED) {
												   pos[j] = skeleton.SkeletonPositions[handsPosition[j]];
												   k.drawPoint(maimaiIm, pos[j], scale);
												   //drawPoint(rgbIm, pos,1);
											   }
										   }
										   //}
										   break;
			}
			case NUI_SKELETON_POSITION_ONLY:
				k.drawPoint(skeletonIm, skeleton.Position, 1);
				break;
			}
		}
	}

	boolean isTouched(int dir){
		//std::cout << "x"<<pos[0].x <<"y"<<pos[0].y<< std::endl;

		boolean bleft = pow((60 + 20), 2) >= pow((getHandsPosition(0).x - circlePos[dir][0]), 2) + pow((getHandsPosition(0).y - circlePos[dir][1]), 2);
		boolean bright = pow((60 + 20), 2) >= pow((getHandsPosition(1).x - circlePos[dir][0]), 2) + pow((getHandsPosition(1).y - circlePos[dir][1]), 2);
		//return bleft || bright;
		if (bleft) return bleft;
		if (bright) return bright;
		return false;
	}

	void loadScore(){
		int k, d, f;

		std::ifstream ifs("Notes.txt");
		std::string str;

		if (ifs.fail()){
			std::cerr << "File don't exist.\n";
			//exit(0);
		}
		int i = 0;
		while (std::getline(ifs, str)){
			k = 0; d = 0; f = 0;
			sscanf_s(str.data(), "%d %d %d", &k, &d, &f); //引数に文字列入れるときは、次の引数にバッファサイズを指定（sizeof(dst)）
			std::cout << k << d << f << std::endl;
			notes.setKind(k);
			notes.setDir(d);
			notes.setFrame(f);
			score[i] = notes;
			i++;
		}
		notenum = i;
	}

	void doEffect(){

	}

	void draw(){
		Kinect k;
		getHands();
		if (loadflag){
			loadScore();
			loadflag = 0;
		}
		createNote(score, notenum);
		//touchの判定
		for (int i = 0; i < array_length(score); i++){
			if ((pow(score[i].getLocation(0) - maiWidth / 2, 2) + pow(score[i].getLocation(1) - maiHeight / 2, 2) >= pow(radius - 100, 2)) &&
				(pow(score[i].getLocation(0) - maiWidth / 2, 2) + pow(score[i].getLocation(1) - maiHeight / 2, 2) <= pow(radius + 150, 2))){
				if (isTouched(score[i].getDir())){
					touch(score[i]);
					score[i].drawCansel();
				}
				score[i].countEffect();
				//std::cout << score[i].getEffect().getJudge() << std::endl;
			}
			/*else if ((pow(score[i].getLocation(0) - maiWidth / 2, 2) + pow(score[i].getLocation(1) - maiHeight / 2, 2)) >= pow(radius + 50, 2)){
				score[i].drawEffect();
			}*/
		}
	}

	void playSound(){

	}

	void showEffect(){
		std::cout << "touched!!" << std::endl;
		//cv::imwrite("sample" + std::to_string(maiframe) + ".png", maimaiIm);
	}

	void touch(Note note){
		//showEffect();
		note.drawEffect();
		switch (note.getKind()){
		case TAP:
			break;
		}

	}
};

class Tap : public Note{
private:
	//int leaveFrame; //タイミング
};

class Hold : public Note{

};

class Slide : public Note{

};

int main(){
	/*************************
	* メインループ
	**************************/

	processFlag = NONE;
	Scene scene;
	Kinect k;
	k.initKinect();
	while (1){
		//更新待ち
		DWORD ret = ::WaitForSingleObject(streamEvent, INFINITE);
		::ResetEvent(streamEvent);
		k.setRgbImage();
		k.setDepthImage();
		k.setSkeletonImage();
		scene.setScene();

		//キーウェイト(qが押されたらループを抜ける)
		int key = cv::waitKey(10);
		if (key == 'q') {
			break;
		}
		else {
			switch (key) {
			case 'h':
				processFlag = GET_HANDS;
				
				break;
			case 's':
				processFlag = GAME_START;
				std::cout << "start" << std::endl;
				//setNotes();
				break;
			case 'e':
				processFlag = NONE;
				loadflag = 1;
				maiframe = 0;
				break;
			}
		}

		//フラグによって処理する
		switch (processFlag){
		case GET_HANDS:
			scene.getHands();
			break;
		case GAME_START:
			scene.draw();//譜面の描画
			maiframe++;
			break;
		}

		cv::imshow("RGB Image", rgbIm);
		cv::imshow("Player Image", playerIm);
		cv::imshow("Skeleton", skeletonIm);
		cv::imshow("maimai", maimaiIm);
	}

	/*************************
	* 終了処理
	*************************/

	if (kinect != 0){
		kinect->NuiShutdown();
		kinect->Release();
	}

}