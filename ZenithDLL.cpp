
////////////////////////////////////////////////////////////////////////////////////////////////////
////                                                                                            ////
////   Copyright (c) 2016 - present, UPM                                                        ////
////                                                                                            ////
////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"

#include <opencv\cv.h>
#include <opencv\highgui.h>
#include <opencv2\opencv.hpp>

#include <iostream>
#include <fstream>


using namespace cv;

extern "C"
{

	int frameNumber = 0;
	std::string ipAdress;
	bool compression;
	cv::VideoCapture vcap;
	Mat frame;
	uchar *image_uchar;

	__declspec(dllexport) const void __cdecl InitializeSensorZenith(char *fps, char *ip)
	{
		std::string ipStr(ip);
		std::string cameraURL = ""; // enter the camera URL here (including credentials)
		std::string direccion = cameraURL + ipStr;
		std::stringstream ss0;
		ss0 << direccion << "/" << "cgi-bin/faststream.jpg?stream=full&fps=" << fps << ".0&noaudio&data=v.mjpg";
		ipAdress = ss0.str();
		ss0.str("");
		image_uchar = NULL;
	}

	__declspec(dllexport) bool __cdecl IsSensorAvailableZenith()
	{
		return vcap.open(ipAdress);
	}

	__declspec(dllexport) bool __cdecl OpenSensorZenith() {
		return vcap.open(ipAdress);
	}

	__declspec(dllexport) void __cdecl CloseSensorZenith() {
		vcap.release();
	}

	uchar *MatToBytes(cv::Mat image)
	{
		int image_size = image.total() * image.elemSize();
		if (image_uchar) {
			delete (image_uchar);
			image_uchar = NULL;
		}
		image_uchar = new uchar[image_size];
		std::memcpy(image_uchar, image.data, image_size * sizeof(uchar));
		return image_uchar;
	}

	__declspec(dllexport) uchar* __cdecl GetFrameZenith() {
		if (frame.data) {
			frame.release();
			frame.~Mat();
		}
		vcap >> frame;
		image_uchar = MatToBytes(frame);
		return image_uchar;
	}
	
	__declspec(dllexport) void __cdecl SaveFrameZenith(char  *path) {
		if (frame.data) {
			frame.release();
			frame.~Mat();
		}
		vcap >> frame;
		imwrite(path,frame);
	}

}


