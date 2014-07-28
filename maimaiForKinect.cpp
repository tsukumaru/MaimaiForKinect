/*#include "Kinect.h"
#include "Effect.h"
#include "Note.h"
#include "Scene.h"*/

#include <iostream>
#include <opencv2/opencv.hpp>

#include <sstream>
#include <Windows.h>
#include <NuiApi.h>
#include <stdint.h>
#include <time.h>
#include <mmsystem.h>

#include <string>
#include <fstream>
#include <list>

#define _USE_MATH_DEFINES
#include <math.h>



#define ERROR_CHECK(ret)\
if (ret != S_OK){
\
	std::stringstream ss; \
	ss << "failed " #ret "" << std::hex << ret << std::endl; \
	throw std::runtime_error(ss.str().c_str()); \
}

#define TAP 1
#define EACH 2
#define HOLD 3
#define SLIDE 4

#define POSE_NUM 4

#define FAST 0
#define PERFECT 1
#define LATE 2
#define MISS 3

const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

INuiSensor *kinect;

HANDLE imageStreamHandle = INVALID_HANDLE_VALUE;
HANDLE depthStreamHandle = INVALID_HANDLE_VALUE;

HANDLE streamEvent;

DWORD width;
DWORD height;

const int minDepth = NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
const int maxDepth = NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

enum{
	NONE,
	GET_HANDS,
	SAVE_POSE_1,
	SAVE_POSE_2,
	SAVE_POSE_3,
	SAVE_POSE_4,
	GAME_START,
	RESULT
};
int processFlag;

//データを扱う行列
cv::Mat rgbIm;
cv::Mat playerIm;
cv::Mat skeletonIm;
cv::Mat maimaiIm;

int scale;
//全体のフレーム数
int maiframe = 0;
int loadflag = 1;
int maiWidth, maiHeight, radius;

cv::Mat savedData;
cv::Mat loadedData[4];

NUI_SKELETON_DATA trackedSkeleton;
const int skippedFrame = 20;
const int savedFrameMax = 20;
int savedFrameIdx = 0;
int skippedFrameIdx = 0;

const int positionList[10] = { NUI_SKELETON_POSITION_HEAD,
NUI_SKELETON_POSITION_SHOULDER_CENTER,
NUI_SKELETON_POSITION_SHOULDER_RIGHT,
NUI_SKELETON_POSITION_ELBOW_RIGHT,
NUI_SKELETON_POSITION_WRIST_RIGHT,
NUI_SKELETON_POSITION_HAND_RIGHT,
NUI_SKELETON_POSITION_SHOULDER_LEFT,
NUI_SKELETON_POSITION_ELBOW_LEFT,
NUI_SKELETON_POSITION_WRIST_LEFT,
NUI_SKELETON_POSITION_HAND_LEFT };


//配列の要素数を求めるテンプレート関数
template
<
typename TYPE,
std::size_t SIZE
>
std::size_t array_length(const TYPE(&)[SIZE]){
	return SIZE;
}


class Kinect{
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
			NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE | NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);

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
		/* 処理を記述*/
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
				USHORT distance = ::NuiDepthPixelToDepth(*depth);

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
		/*skeletonIm = cv::Mat::zeros(height, width, CV_8UC3);
		cv::cvtColor(rgbIm, rgbIm, CV_BGRA2BGR);
		rgbIm.copyTo(skeletonIm, playerIm);*/
		NUI_SKELETON_FRAME skeletonFrame = { 0 };
		kinect->NuiSkeletonGetNextFrame(0, &skeletonFrame);

		skeletonIm = cv::Mat::zeros(height, width, CV_8UC3);
		cv::cvtColor(rgbIm, rgbIm, CV_BGRA2BGR);
		rgbIm.copyTo(skeletonIm, playerIm);
		for (int i = 0; i < NUI_SKELETON_COUNT; i++){
			const NUI_SKELETON_DATA &skeleton = skeletonFrame.SkeletonData[i];

			switch (skeleton.eTrackingState)
			{
			case NUI_SKELETON_TRACKED: //詳細スケルトンデータを得られる
				drawTrackedSkeleton(skeletonIm, skeleton);
				trackedSkeleton = skeleton;
				break;

			case NUI_SKELETON_POSITION_ONLY: //重心だけ
				drawPoint(skeletonIm, skeleton.Position, 10);
				break;
			}
		}
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

	void drawLine(cv::Mat& image, Vector4 pos1, Vector4 pos2)
	{
		// ３次元の位置から距離画像での位置に変換
		FLOAT depthX1 = 0, depthY1 = 0;
		FLOAT depthX2 = 0, depthY2 = 0;

		NuiTransformSkeletonToDepthImage(pos1, &depthX1, &depthY1, CAMERA_RESOLUTION);
		NuiTransformSkeletonToDepthImage(pos2, &depthX2, &depthY2, CAMERA_RESOLUTION);

		// 距離画像での位置からRGB画像での位置に変換
		LONG colorX1 = 0, colorY1 = 0;
		LONG colorX2 = 0, colorY2 = 0;
		kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
			CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, (LONG)depthX1, (LONG)depthY1, 0, &colorX1, &colorY1);
		kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
			CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, (LONG)depthX2, (LONG)depthY2, 0, &colorX2, &colorY2);

		// RGB画像での位置に線分を描画
		cv::line(image, cv::Point(colorX1, colorY1), cv::Point(colorX2, colorY2), cv::Scalar(50, 255, 50), 5);
	}


	void drawBone(cv::Mat& image, const NUI_SKELETON_DATA & skeleton, NUI_SKELETON_POSITION_INDEX jointFrom, NUI_SKELETON_POSITION_INDEX jointTo)
	{
		NUI_SKELETON_POSITION_TRACKING_STATE jointFromState = skeleton.eSkeletonPositionTrackingState[jointFrom];
		NUI_SKELETON_POSITION_TRACKING_STATE jointToState = skeleton.eSkeletonPositionTrackingState[jointTo];

		// Draw only if one of the points is inferred OR both points are tracked
		if ((jointFromState == NUI_SKELETON_POSITION_INFERRED || jointToState == NUI_SKELETON_POSITION_INFERRED) ||
			(jointFromState == NUI_SKELETON_POSITION_TRACKED && jointToState == NUI_SKELETON_POSITION_TRACKED))
		{
			const Vector4 jointFromPosition = skeleton.SkeletonPositions[jointFrom];
			const Vector4 jointToPosition = skeleton.SkeletonPositions[jointTo];
			drawLine(image, jointFromPosition, jointToPosition);
		}
	}

	void drawTrackedSkeleton(cv::Mat& image, const NUI_SKELETON_DATA& skeleton)
	{
		// 胴体
		drawBone(image, skeleton, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER);
		drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT);
		drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
		drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE);
		drawBone(image, skeleton, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER);

		drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT);
		drawBone(image, skeleton, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
		drawBone(image, skeleton, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);
		drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT);
		drawBone(image, skeleton, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
		drawBone(image, skeleton, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);

		// 足などの描画
		// .........
	}

	bool doSaveFrame(const char filename[], cv::Mat &savedData)
	{
		if (trackedSkeleton.eTrackingState != NUI_SKELETON_TRACKED) {
			//std::cout << "bbbb" << std::endl;
			return false;
		}

		skippedFrameIdx++;
		if (skippedFrameIdx < skippedFrame) {
			char txt[255];
			sprintf_s(txt, 255, "Prepare count down %d", (skippedFrame - skippedFrameIdx));
			cv::putText(rgbIm, txt, cvPoint(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cvScalar(0, 0, 0, 255), 2);
			cv::putText(rgbIm, txt, cvPoint(29, 29), cv::FONT_HERSHEY_SIMPLEX, 1, cvScalar(255, 255, 255, 255), 2);
			std::cout << txt << std::endl;
			return false;
		}

		//NUI_SKELETON_POSITION_SHOULDER_CENTERに対して相対座標
		//std::cout << "hey" << std::endl;
		for (int i = 0; i < 10; i++) {
			savedData.at<FLOAT>(i * 3, savedFrameIdx) = trackedSkeleton.SkeletonPositions[positionList[i]].x -
				trackedSkeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].x;
			savedData.at<FLOAT>(i * 3 + 1, savedFrameIdx) = trackedSkeleton.SkeletonPositions[positionList[i]].y -
				trackedSkeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].y;
			savedData.at<FLOAT>(i * 3 + 2, savedFrameIdx) = trackedSkeleton.SkeletonPositions[positionList[i]].z -
				trackedSkeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].z;
		}
		savedFrameIdx++;
		if (savedFrameIdx < savedFrameMax) {
			char txt[255];
			sprintf_s(txt, 255, "%s Frame %d", filename, savedFrameIdx);
			cv::putText(rgbIm, txt, cvPoint(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cvScalar(0, 0, 0, 255), 2);
			cv::putText(rgbIm, txt, cvPoint(29, 29), cv::FONT_HERSHEY_SIMPLEX, 1, cvScalar(255, 255, 255, 255), 2);
			std::cout << txt << std::endl;
			return false;
		}

		cv::FileStorage cfs(filename, cv::FileStorage::WRITE);
		cfs << "root" << savedData;
		cfs.release();
		std::cout << "Saving " << filename << std::endl;
		return true;
	}
};


class Effect{
private:
	int kind;
	int location[10][10]; //x,y
	int frame = 1000000;//始まりから終わりまでのフレーム数
	int count = 0;
	int judge = 0;

public:
	static int judgeflag;
	static int efftime;

public:
	int getJudgeFlag(){
		return judgeflag;
	}

	int getJudge(){
		return judge;
	}

	void judgeInc(){
		judge++;
	}

	void perfectJudge(){
		if (efftime < 10)
			cv::putText(maimaiIm, "PERFECT!!!", cvPoint(maiWidth / 5, maiHeight / 2), cv::FONT_HERSHEY_SIMPLEX, 6, cvScalar(0, 100, 0, 255), 4);
		efftime++;
	}
	void draw(){
		switch (judgeflag){
		case 0:
			std::cout << "FAST!!!!" << std::endl;
			drawJudge("FAST!!!");
			break;
		case 1:
			std::cout << "PERFECT!!!!" << std::endl;
			drawJudge("PERFECT!!!");
			break;
		case 2:
			std::cout << "LATE!!!!" << std::endl;
			drawJudge("LATE!!!");
			break;
		case 3:
			std::cout << "MISS!!!!" << std::endl;
			drawJudge("MISS!!!");
			break;
		case 100:
			std::cout << "PERFECT!!!!" << std::endl;
			drawJudge("PERFECT!!!!");
			break;
		}
	}

	void drawJudge(char *txt){
		FLOAT depthX1 = 0, depthY1 = 0;
		NuiTransformSkeletonToDepthImage(trackedSkeleton.SkeletonPositions[NUI_SKELETON_POSITION_HEAD],
			&depthX1, &depthY1, CAMERA_RESOLUTION);

		LONG colorX1 = 0, colorY1 = 0;
		kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
			CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, (LONG)depthX1, (LONG)depthY1, 0, &colorX1, &colorY1);
		colorX1 *= 2;
		colorY1 *= 2;
		colorX1 = colorX1 - 60;
		if (colorX1 < 0) colorX1 = 0;
		if (efftime < 10){
			if (judgeflag == 100)
				cv::putText(maimaiIm, "PERFECT!!!", cvPoint(maiWidth / 5, maiHeight / 2), cv::FONT_HERSHEY_SIMPLEX, 6, cvScalar(0, 100, 0, 255), 4);
			else
				cv::putText(maimaiIm, txt, cvPoint(colorX1, colorY1), cv::FONT_HERSHEY_SIMPLEX, 3, cvScalar(0, 100, 0, 255), 4);
		}
		efftime++;
	}
};

int Effect::judgeflag = -1;
int Effect::efftime = 0;

class Note{
private:
	int kind; //ノートは4種類
	int dir; //方向
	int frame; //タイミング
	int noteframe = 0;
	int tapradius = 30;
	int drawflag = 1;
	double location[2]; //x,y
	Effect e;

public:
	static int result[4];

public:
	Note(){
		for (int i = 0; i < 4; i++){
			result[i] = 0;
		}
	}

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

	int *getResult(){
		return result;
	}

	bool canmove(){
		if (drawflag)
			return this->frame <= maiframe;
		else
			return false;
	}
	//消していいかどうか
	bool isDeleted(){
		return false;
	}

	void draw(){
		location[0] = maiWidth / 2 + (noteframe * 10)*cos(dir* M_PI / 4 - M_PI / 2 - M_PI / 8);
		location[1] = maiHeight / 2 + (noteframe * 10)*sin(dir*M_PI / 4 - M_PI / 2 - M_PI / 8);
		switch (kind){
		case TAP:
			cv::circle(maimaiIm, cv::Point(location[0], location[1]), tapradius, cv::Scalar(255, 20, 255), 6, 4);
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
		Effect::efftime = 0;
		if (2 <= e.getJudge() && e.getJudge() <= 5){
			Effect::judgeflag = 0;
			result[FAST]++;
		}
		else if (6 <= e.getJudge() && e.getJudge() <= 9){
			Effect::judgeflag = 1;
			result[PERFECT]++;
		}
		else if (10 <= e.getJudge() && e.getJudge() <= 13){
			Effect::judgeflag = 2;
			result[LATE]++;
		}
		else{
			Effect::judgeflag = 3;
			result[MISS]++;
		}
	}

	Effect getEffect(){
		return e;
	}

	void casePerfect(){
		Effect::efftime = 0;
		Effect::judgeflag = 100;
	}
};

int Note::result[4];

class Scene{
private:
	int speed;
	int notenum;
	int smallCircleRad = 10;
	int effect[4]; //エフェクトはPerfect,Great,Good,TooLateの4つ
	int circlePos[9][2];
	int rec;
	//譜面を読み込む
	Note notes;
	Note score[100];
	Vector4 pos[2];//左右の手の座標
	Effect e;
public:
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
			cv::circle(maimaiIm, cv::Point(maiWidth / 2 + (radius)*cos(i*M_PI / 4 - M_PI / 2 - M_PI / 8), maiHeight / 2 + (radius)*sin(i*M_PI / 4 - M_PI / 2 - M_PI / 8)), smallCircleRad, cv::Scalar(150, 0, 150), 6, 4);
			circlePos[i][0] = maiWidth / 2 + radius*cos(i*M_PI / 4 - M_PI / 2 - M_PI / 8);
			circlePos[i][1] = maiHeight / 2 + radius*sin(i*M_PI / 4 - M_PI / 2 - M_PI / 8);
		}
	}

	void createNote(Note *score, int notenum){
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
												   // k.drawPoint(maimaiIm, pos[j], scale);
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

	bool isTouched(int dir){
		//std::cout << "x"<<pos[0].x <<"y"<<pos[0].y<< std::endl;

		bool bleft = pow((60 + 20 + 20), 2) >= pow((getHandsPosition(0).x - circlePos[dir][0]), 2) + pow((getHandsPosition(0).y - circlePos[dir][1]), 2);
		bool bright = pow((60 + 20 + 20), 2) >= pow((getHandsPosition(1).x - circlePos[dir][0]), 2) + pow((getHandsPosition(1).y - circlePos[dir][1]), 2);
		//return bleft || bright;
		if (rec == 1) return true;
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

	void doLoadFrame()
	{
		for (int i = 0; i < POSE_NUM; i++) {
			char filename[255];
			sprintf_s(filename, 255, "pose%d.xml", i + 1);
			cv::FileStorage cfs(filename, cv::FileStorage::READ);
			cfs["root"] >> loadedData[i];
			cfs.release();
			std::cout << "Loading " << filename << ", matrix size: " << loadedData[i].size().height << " by " << loadedData[i].size().width << std::endl;
		}
	}

	int doRecognition()
	{
		Kinect k;
		if (trackedSkeleton.eTrackingState != NUI_SKELETON_TRACKED) return -1;
		cv::Mat inputData = cv::Mat::zeros(30, 1, CV_32F);

		for (int i = 0; i < 10; i++) {
			float xx = trackedSkeleton.SkeletonPositions[positionList[i]].x - trackedSkeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].x;
			float yy = trackedSkeleton.SkeletonPositions[positionList[i]].y - trackedSkeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].y;
			float zz = trackedSkeleton.SkeletonPositions[positionList[i]].z - trackedSkeleton.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].z;
			inputData.at<FLOAT>(i * 3, 0) = xx;
			inputData.at<FLOAT>(i * 3 + 1, 0) = yy;
			inputData.at<FLOAT>(i * 3 + 2, 0) = zz;
		}

		//std::cout << "HEAD 距離 = " << trackedSkeleton.SkeletonPositions[positionList[NUI_SKELETON_POSITION_HEAD]].y << std::endl;
		char txt[255];
		/*if (trackedSkeleton.SkeletonPositions[positionList[NUI_SKELETON_POSITION_HEAD]].y < -0.25){
		sprintf_s(txt, 255, "(%d)", 2);
		return 2;
		}*/
		double distance[4] = { -1, -1, -1, -1 };
		int smallestIdx = -1;
		double smallestDistance = 999999;
		for (int i = 0; i<POSE_NUM; i++) {
			double tmp = 0;
			for (int j = 0; j<savedFrameMax; j++) {
				tmp += cv::norm(loadedData[i].colRange(j, j + 1) - inputData);
			}
			distance[i] = tmp / savedFrameMax;
			//std::cout << "distance[" << i << "] = " << distance[i] << std::endl;
			if (smallestDistance > distance[i]) {
				smallestDistance = distance[i];
				//std::cout << smallestDistance << std::endl;
				smallestIdx = i + 1; // pose 1, pose 2, pose 3, pose 4
			}
		}
		//std::cout << "smallestIdx = " << smallestIdx << std::endl;
		if (distance[smallestIdx - 1] > 0.3) smallestIdx = 0;
		//char txt[255];
		sprintf_s(txt, 255, "(%d)", smallestIdx);
		FLOAT depthX1 = 0, depthY1 = 0;
		NuiTransformSkeletonToDepthImage(trackedSkeleton.SkeletonPositions[NUI_SKELETON_POSITION_HEAD],
			&depthX1, &depthY1, CAMERA_RESOLUTION);

		LONG colorX1 = 0, colorY1 = 0;
		kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
			CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, (LONG)depthX1, (LONG)depthY1, 0, &colorX1, &colorY1);
		colorX1 = colorX1 - 60;
		if (colorX1 < 0) colorX1 = 0;
		cv::putText(rgbIm, txt, cvPoint(colorX1, colorY1), cv::FONT_HERSHEY_SIMPLEX, 3, cvScalar(0, 0, 0, 255), 4);
		cv::putText(rgbIm, txt, cvPoint(colorX1 - 1, colorY1 - 1), cv::FONT_HERSHEY_SIMPLEX, 3, cvScalar(255, 255, 255, 255), 4);

		return smallestIdx;
	}

	void draw(){
		static int canselnum = 0;

		Kinect k;

		getHands();
		rec = doRecognition();
		if (loadflag){
			loadScore();
			loadflag = 0;
		}
		createNote(score, notenum);
		//touchの判定
		for (int i = 0; i < array_length(score); i++){
			if ((pow(score[array_length(score) - 1].getLocation(0) - maiWidth / 2, 2) + pow(score[array_length(score) - 1].getLocation(1) - maiHeight / 2, 2) <= pow(radius + 150, 2))
				|| canselnum == array_length(score) - 1){
				processFlag = RESULT;
			}
			if ((pow(score[i].getLocation(0) - maiWidth / 2, 2) + pow(score[i].getLocation(1) - maiHeight / 2, 2) >= pow(radius - 100, 2)) &&
				(pow(score[i].getLocation(0) - maiWidth / 2, 2) + pow(score[i].getLocation(1) - maiHeight / 2, 2) <= pow(radius + 300, 2))){
				//if (rec == 1 && (pow(score[i].getLocation(0) - maiWidth / 2, 2) + pow(score[i].getLocation(1) - maiHeight / 2, 2) == pow(radius, 2))){
				if (rec == 1){
					score[i].casePerfect();
					score[i].drawCansel();
					canselnum++;
				}
				else {
					if (isTouched(score[i].getDir())){
						touch(score[i]);
						score[i].drawCansel();
						canselnum++;
					}
				}
				score[i].countEffect();
			}

			if (maiframe <= 200){
				e.draw();
			}
			else {
				processFlag = RESULT;
			}
		}
	}

	void letsKomaneti(){

	}

	void showEffect(){
		std::cout << "touched!!" << std::endl;
		//cv::imwrite("sample" + std::to_string(maiframe) + ".png", maimaiIm);
	}

	void touch(Note note){
		//showEffect();
		note.drawEffect();
		//e.draw();
		switch (note.getKind()){
		case TAP:
			break;
		}

	}

	void printResult(){
		cv::putText(maimaiIm, "Perfect : " + std::to_string(Note::result[1]), cv::Point(maiWidth / 2 - 30, maiHeight / 5), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255, 255), 4, CV_AA);
		cv::putText(maimaiIm, "FAST : " + std::to_string(Note::result[0]), cv::Point(maiWidth / 2 - 30, maiHeight / 5 * 2), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255, 255), 4, CV_AA);
		cv::putText(maimaiIm, "LATE : " + std::to_string(Note::result[2]), cv::Point(maiWidth / 2 - 30, maiHeight / 5 * 3), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255, 255), 4, CV_AA);
		cv::putText(maimaiIm, "Miss : " + std::to_string(Note::result[3]), cv::Point(maiWidth / 2 - 30, maiHeight / 5 * 4), cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(255, 255, 255, 255), 4, CV_AA);
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
			case '1':
				std::cout << "ポーズ1の辞書データの取得を開始する" << std::endl;
				processFlag = SAVE_POSE_1;
				savedFrameIdx = 0;
				skippedFrameIdx = 0;
				savedData = cv::Mat::zeros(30, savedFrameMax, CV_32F);
				break;
			case '2':
				std::cout << "ポーズ2の辞書データの取得を開始する" << std::endl;
				processFlag = SAVE_POSE_2;
				savedFrameIdx = 0;
				skippedFrameIdx = 0;
				savedData = cv::Mat::zeros(30, savedFrameMax, CV_32F);
				break;
			case '3':
				std::cout << "ポーズ3の辞書データの取得を開始する" << std::endl;
				processFlag = SAVE_POSE_3;
				savedFrameIdx = 0;
				skippedFrameIdx = 0;
				savedData = cv::Mat::zeros(30, savedFrameMax, CV_32F);
				break;
			case '4':
				std::cout << "ポーズ4の辞書データの取得を開始する" << std::endl;
				processFlag = SAVE_POSE_4;
				savedFrameIdx = 0;
				skippedFrameIdx = 0;
				savedData = cv::Mat::zeros(30, savedFrameMax, CV_32F);
				break;
			case 's':
				PlaySound("be1_002.wav", NULL, SND_FILENAME | SND_ASYNC | SND_LOOP);
				scene.doLoadFrame();
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
		case SAVE_POSE_1:
			//std::cout << "aaaaaa" << std::endl;
			if (k.doSaveFrame("pose1.xml", savedData) == true)
				processFlag = NONE;
			break;
		case SAVE_POSE_2:
			if (k.doSaveFrame("pose2.xml", savedData) == true)
				processFlag = NONE;
			break;
		case SAVE_POSE_3:
			if (k.doSaveFrame("pose3.xml", savedData) == true)
				processFlag = NONE;
			break;
		case SAVE_POSE_4:
			if (k.doSaveFrame("pose4.xml", savedData) == true)
				processFlag = NONE;
			break;
		case GET_HANDS:
			scene.getHands();
			break;
		case GAME_START:
			//scene.doRecognition();
			scene.draw();//譜面の描画
			maiframe++;
			break;
		case RESULT:
			scene.printResult();
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