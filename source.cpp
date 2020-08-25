// opencvtest.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

//#include <stdafx.h> 
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <opencv2/photo.hpp>
#include <opencv2/imgcodecs.hpp>

#define recording 1
#define debuggingMode 1
#define isiArray 0
#define printThreshold 0 //print masking

//#define CV_CHAIN_APPROX_NONE cv::CHAIN_APPROX_NONE
//#define CV_RETR_LIST cv::RETR_LIST
//#define CV_AA Core.LINE_AA
//#define CV_FILLED cv::FILLED
//#define CV_NAMEDWINDOW cv::namedWindow
//struct cvScalar;
//void cvdestroyAllWindows();

using namespace cv;
using namespace std;

// ========== VARIABEL FILTER HSV ==========

int HLow = 0;// 230;
int HHigh = 255;
int SLow = 0;
int SHigh = 255;
int VLow = 236;
int VHigh = 255;

//  ========== VARIABEL ROI ==========

Point rook_points[4];

int lebar_atas = 560;// 560;
int lebar_bawah = 640;
int tinggi_bawah = 0;
int tinggi_atas = 480*(0.45);// 200;// 400;

//========== VARIABEL SEARCH LINE ==========
int pixel = 0;
int nilai = 0;
int lastpixel = 0;
int searchblack = 0;
int pixelskip = 20;// 100;

Mat imgOriginal, imgResized;
Mat imgHSV;

int FirstMax_y;

int lebar_garis;
int frame_width = 640;//640; //extra
int frame_height = 480;// 480; //extra

vector<Vec4i> hierarchy;

//========== ERROR ==========
int ref_susur = 0, prevRefSusur = 0;


//==========================================================================================
int c1, c2, c3, c4, c5, c6;

void Trackbar_HSV() {

	namedWindow("Trackbar_HSV", WINDOW_NORMAL);

	createTrackbar("HLow", "Trackbar_HSV", &HLow, 255);
	createTrackbar("HHigh", "Trackbar_HSV", &HHigh, 255);

	createTrackbar("SLow", "Trackbar_HSV", &SLow, 255);
	createTrackbar("SHigh", "Trackbar_HSV", &SHigh, 255);

	createTrackbar("VLow", "Trackbar_HSV", &VLow, 255);
	createTrackbar("VHigh", "Trackbar_HSV", &VHigh, 255);
}

Mat masking(Mat param1, Mat imgPrint, bool print, int lebar_atas, int tinggi_bawah, int tinggi_atas, int lebar_bawah) {

	rook_points[0] = Point(0 + lebar_bawah, imgOriginal.rows - tinggi_bawah);
	rook_points[1] = Point(imgOriginal.cols - lebar_bawah, imgOriginal.rows - tinggi_bawah);
	rook_points[2] = Point(imgOriginal.cols - lebar_atas, imgOriginal.rows - tinggi_atas);
	rook_points[3] = Point(lebar_atas, imgOriginal.rows - tinggi_atas);

	Mat output;
	Mat mask = Mat::zeros(imgPrint.size(), imgPrint.type());

	// Create a binary polygon mask
	fillConvexPoly(mask, rook_points, 4, Scalar(255));

	// Multiply the edges image and the mask to get the output
	bitwise_and(imgPrint, mask, output);

	// Print rook point (Trapesium ukuran ROI)

	if (print) {

		line(imgOriginal, rook_points[0], rook_points[1], Scalar(0, 255, 255), 2);
		line(imgOriginal, rook_points[1], rook_points[2], Scalar(0, 255, 255), 2);
		line(imgOriginal, rook_points[2], rook_points[3], Scalar(0, 255, 250), 2);
		line(imgOriginal, rook_points[3], rook_points[0], Scalar(0, 255, 250), 2);

		putText(imgOriginal, "0", rook_points[0],
			FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0, 255, 255), 2, CV_AA);
		putText(imgOriginal, "1", rook_points[1],
			FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0, 255, 255), 2, CV_AA);
		putText(imgOriginal, "2", rook_points[2],
			FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0, 255, 255), 2, CV_AA);
		putText(imgOriginal, "3", rook_points[3],
			FONT_HERSHEY_COMPLEX_SMALL, 2, cvScalar(0, 255, 255), 2, CV_AA);
	}


	return output;
}

void hitungRegresi(Point line[][8], int index, int min, int max, long double& regA, long double& regB) {

	long totalX = 0, totalY = 0, totalXY = 0, totalXkuadrat = 0, kuadrattotalX = 0;
	int n = max - min + 1;
	totalX = 0;
	totalY = 0; totalXY = 0; totalXkuadrat = 0; kuadrattotalX = 0;
	for (int i = min; i <= max; i++) {
		if (line[i][index] != Point(0, 0)) {
			totalX += line[i][index].x; //1876 benar
			totalY += line[i][index].y; //945 benar
			totalXY += line[i][index].x * line[i][index].y;  //591,111 benar
			totalXkuadrat += pow(line[i][index].x, 2); // 1,173,306 benar
		}
	}
	kuadrattotalX = pow(totalX, 2); //3.319.376

	if (totalXkuadrat <= 10000000) {

		long double regresi_B = ((n * totalXkuadrat) - (kuadrattotalX)); //-1172760
		if (regresi_B != 0)
			regB = ((n * totalXY) - (totalX * totalY)) / regresi_B;
		else
			regB = 0;


		long double regresi_A = ((n * totalXkuadrat) - (kuadrattotalX));
		if (regresi_A != 0)
			regA = ((totalXkuadrat * totalY) - (totalX * totalXY)) / regresi_A;
		else regA = 0;
	}
	else {
		regA = 0;
		regB = 0;
	}
}

// Fungsi Hitung_AB digunakan untuk proses regresi linier
void Hitung_AB(Point& line1, Point& line2, long double& regA, long double& regB)
{
	double totalX, totalY, totalXY, totalXkuadrat, kuadrattotalX;

	totalX = line1.x + line2.x;
	totalY = line1.y + line2.y;
	totalXY = (line1.x * line1.y) + (line2.x * line2.y);

	kuadrattotalX = pow(totalX, 2);
	totalXkuadrat = pow(line1.x, 2) + pow(line2.x, 2);

	if (totalXkuadrat <= 10000000) {

		double regresi_B = ((2 * totalXkuadrat) - (kuadrattotalX));
		if (regresi_B > 0)
			regB = ((2 * totalXY) - (totalX * totalY)) / regresi_B;
		else regB = 0;

		double regresi_A = ((2 * totalXkuadrat) - (kuadrattotalX));
		if (regresi_A > 0)
			regA = ((totalXkuadrat * totalY) - (totalX * totalXY)) / regresi_A;
		else regA = 0;
	}
}

int main() {

	int error = 0, prevError = 0, errorh2 = 0;
	double errorH1Test = 0, lastErrorH1Test=0;
	int lebarJalan = 0, lastLebarJalanH1 = 0;
	float error_f = 0;
	int susurTest = 0;

	//------ akses video ---------

	//VideoCapture cap("23juli/ERROR0_2.mp4"); 
	VideoCapture cap("23juli/jalan_4_lurus.mp4");
	//VideoCapture cap("4agus/maju_lurus_1.mp4");
	//VideoCapture cap("29juli2/jalan2.mp4");//miringkanan_2.mp4"); //Video bagus dari yutup

	// ----- akses video streaming -----//


	//VideoCapture cap(0);

	if (!cap.isOpened()) {
		cout << "Kamera tidak dapat di akses" << endl;
		return -1;
	}



	cap.set(CAP_PROP_FRAME_WIDTH, frame_width);// 640); //TADINYA PAKE CV_ DI DEPANNYA
	cap.set(CAP_PROP_FRAME_HEIGHT, frame_height);// 480);


	// ----- membuat video output ------ //NAMAFILE
	VideoWriter video("4agus.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(frame_width, frame_height));	//extra

	while (1) {

		//----- Akses Video ----
		cap >> imgOriginal;
		//------- Akses Gambar -----

		//imgOriginal = imread("29juli/ERROR-80.jpg");
		//imgOriginal = imread("2line.jpeg");
		//imgOriginal = imread("29juli2/ERROR_+50_1.jpg");
		//imgOriginal = imread("29juli3/0.jpg");
		//imgOriginal = imread("4agus/0_50.jpg");

		// resize gambar agar ukuran 640x480
		resize(imgOriginal, imgOriginal, Size(640, 480));
		//cout << imgOriginal(Range(1, 2), Range(1, 2)) << endl; // utk access data Mat



		//========== VARIABEL IMAGE ==========
		int imgrows = imgOriginal.rows;
		int imgcols = imgOriginal.cols;
		int imgrowshalf = 0.45 * imgOriginal.rows; // 0.61 * imgOriginal.rows; //extra //0.7


		//========== VARIABEL SORTED ARRAY ==========
		int linescan = imgrows / 50;
		int baris_garis = 1;
		int pointcount;
		int Max_pointcount = 0;
		int lastpointcount = 0;
		int FirstMax_pointcount = 0;
		int lastx[8];
		Point point_titik[200];
		int nilai_titik = 0;
		Point array_titik[100][8];
		Point regresi_cek;
		long double line1_regA, line1_regB, line2_regA, line2_regB;


		//========== ERROR ==========
		int temp = 0, temp_belok = 0;
		int temp1 = 0, temp1_belok = 0;
		int colsH[8], colsH2[8];
		double regB[8];
		int kosong[8] = { 0,0,0,0,0 };
		int h = imgOriginal.rows * 0.97; //0.77;// 75;// 0.94;//0.986, 0.88;  //titik ketinggian garis merah varH
		int h2 = imgOriginal.rows * 0.77; //asli 0.65
		int h3 = imgOriginal.rows * 0.6;
		int Last_ref_titik_tengah = 0;
		int ref_titik_tengah = 0;
		int error_sudut = 0;
		colsH[1] = 0, colsH2[2] = 0;
		colsH[2] = 0, colsH2[2] = 0;
		temp = 0;
		int temp_garis = 0, temp_garis2 = 0;
		int tengah_frame = imgOriginal.cols / 2;
		int belokH2 = 0;  // 1:kanan, 2:kiri, 3:per4an


		//========== ZEBRACROSS ==========
		int lebar = 0;
		int zebracross = 0;



		int index1[8] = { 0,0,0,0,0 };
		int indexY[8] = { 0,0,0,0,0 };


		//================================== PRE PROCESSING =========================================

		//Atur parameter HSV dengan taskbar
		Trackbar_HSV();

		//HSV Filter
		Mat imgHSV, hsv1;
		cvtColor(imgOriginal, hsv1, COLOR_BGR2HSV);
		inRange(hsv1, Scalar(HLow, SLow, VLow), Scalar(HHigh, SHigh, VHigh), imgHSV);
		//inRange(imgHSV, Scalar(HLow, SLow, VLow), Scalar(HHigh, SHigh, VHigh), imgOriginal);

		//Mencari Contour		
		std::vector<std::vector<cv::Point>> contours;
		std::vector<std::vector<cv::Point>> contoursFiltered;
		Mat contourOutput = imgHSV.clone();
		findContours(contourOutput, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

		//Moments Contour
		vector<Moments> mu(contours.size());
		for (int i = 0; i < contours.size(); i++) {
			mu[i] = moments(contours[i], false);
		}

		//Draw Contours
		Mat imgResultArea(imgHSV.size(), CV_8UC1, Scalar(0, 0, 0));
		for (int i = 0; i < contours.size(); i++)
		{
			Scalar color = Scalar(255);
			//ukuran minimum dan maksimum "filter area"
			if ((contourArea(contours[i]) > 400) && (contourArea(contours[i]) < 8000)) //500 - 8000 100 - 10000
			{
				drawContours(imgResultArea, contours, i, color, FILLED);
			}
		}

		Mat imgHasil;
		bitwise_and(imgHSV, imgResultArea, imgHasil);

		Mat imgThresholded = Mat::zeros(Size(1080, 720), CV_8UC1);
		imgThresholded = masking(imgThresholded, imgHasil, printThreshold, lebar_atas, tinggi_bawah, tinggi_atas, lebar_bawah);



		//================================== Scan Titik Putih =========================================
		cout << " Mulai  " << endl;
		searchblack = 0;
		for (int y = imgrowshalf; y < imgrows; y += linescan) { //rows
			pointcount = 0;
			//proses scaning titik putih
			for (int x = 0; x < imgcols; x++) { //cols
				lastpixel = pixel;
				pixel = (int)imgThresholded.at<uchar>(y, x);

				if (!searchblack) {
					if (!lastpixel && pixel) { //hitam-putih
						point_titik[nilai_titik] = Point(x, y);
						circle(imgOriginal, point_titik[nilai_titik], 2, Scalar(255, 0, 0), 2);

						pointcount++;

						searchblack = 1;
						x += pixelskip; //jarak x tiap titik
						nilai_titik++;
					}
				}
				else if (!pixel && !lastpixel) // // putih-hitam
					searchblack = 0;

				//Hilangin zebracross
				else { //hitam-hitam / putih-putih
					nilai = 0;
					for (int i = (point_titik[nilai_titik - 1].x); i < imgcols; i++) { // scan lebar garis putih
						nilai = (int)imgThresholded.at<uchar>(y, i);

						if (!nilai) {
							lebar = i - (point_titik[nilai_titik - 1].x);
							break;
						}
					}
					if (lebar > 50) {
						cout << " titik = " << point_titik[nilai_titik - 1] << endl;
						cout << " lebar = " << lebar << endl;
						circle(imgOriginal, point_titik[nilai_titik - 1], 2, Scalar(0, 0, 0), 2);

						array_titik[baris_garis][pointcount] = Point(0, 0);
						pointcount--;
						x = x + lebar - 50;
					}
				}
				array_titik[baris_garis][pointcount] = point_titik[nilai_titik - 1]; //isi koordinat titik pada garis
			}

			//menentukan nilai Maximum kolom
			if (pointcount > lastpointcount) {
				Max_pointcount = pointcount; //index titik pertama terbanyak pada tiap garis
				lastpointcount = pointcount;
				FirstMax_pointcount = baris_garis;

				//cout << "First_Max_pointcount :=" << FirstMax_pointcount << endl;

				FirstMax_y = y;
			}

			if (pointcount > 0) {
				baris_garis++;
			}
		}
		cout << "  " << endl;

		/*
		// Deteksi Zebracross
		if (Max_pointcount > 3) {
			zebracross = 0;
			//Hitung lebar titik putih - zebracross
			for (int k = 1; k < (Max_pointcount + 1); k++) {
				for (int i = array_titik[FirstMax_pointcount][k].x; i < imgcols; i++) {

					nilai = (int)imgThresholded.at<uchar>(array_titik[FirstMax_pointcount][k].y, i);

					if (!nilai) {
						lebar = i - (array_titik[FirstMax_pointcount][k].x);
						cout << "kolom = " << k << "     " << " lebar = " << lebar << endl;
						cout << "x = " << array_titik[FirstMax_pointcount][k] << "     " << " y = " << i << endl;
						break;
					}
				}
				if (lebar > 50) {
					zebracross++;
				}
			}
			if (zebracross > 1) {
				cout << "ada zebracross" << endl;
			}
		}
		*/

		//======================== Mengambil Nilai Array Titik yang Terdeteksi ============================

		if (debuggingMode == 1) { //print inisial array titik 
			cout << "====== ARRAY SEBELUM =====" << endl;

			//Print nilai (x,y) titik yang terdeteksi
			for (int k = 0; k < baris_garis; k++) { //y
				//for (int i = 1; i < (Max_pointcount + 1); i++) {
				for (int i = 1; i < 7; i++) { //x
					cout << array_titik[k][i] << "    ";
				}
				cout << endl;
			}

			cout << "Max_pointcount Sebelum =" << Max_pointcount << endl;
			cout << "First_Max_pointcount Sebelum =" << FirstMax_pointcount << endl;
		}
		//===================================== Verifikasi Array =========================================//

		/*
		Jika kolom berikutnya (max_pointcount+1) ada nilai yang tidak (0,0)
		maka max_pointcount = (max_pointcount + 1)
		*/

		//+++++++++++++ Verifikasi ke atas +++++++++++++
		for (int i = FirstMax_pointcount - 1; i >= 0; i--) {
			for (int y = 1; y < Max_pointcount + 1; y++) {
				cout << i << y << endl;
				if (array_titik[i+1][y].x > 0 && array_titik[i+1][y].x < 2000) {

					//selisih nilai x sekarang dan dibawahnya
					if (abs(array_titik[i][y].x - array_titik[i + 1][y].x) >= 31) {//80
						cout << "atas_geser" << i << " - x : " << array_titik[i][y].x << " - x : " << array_titik[i + 1][y].x << endl;
						//proses pindah (geser) posisi 
						for (int z = 6; z >= y; z--) {
							array_titik[i][z + 1] = array_titik[i][z];
							array_titik[i][z] = Point(0, 0);
						}
						if (y + 1 > Max_pointcount && array_titik[i][y + 1] != Point(0, 0)) {
							Max_pointcount = y + 1;
						}
					}
				}

				/*
					Untuk dapat melakukan regresi, jumlah minimum titik yang dibutuhkan ada 2
					jika tidak ada titik yang terdeteksi (0,0) maka, masuk ke else untuk mencari 2 titik
					di atasnya (cari atas1 dan cari atas2) agar bisa melakukan regresi
					Regresi (Regresi hanya dihitung jika nilai A dan B !=0)
					jika nilai hasil regresi adalah nol, maka yang digunakan nilai regresi yang terakhir sebelum nol
				*/

				else
				{
					int atas_1 = 0, atas_2 = 0, bawah_1 = 0, bawah_2 = 0;
					int index_atas = 3, atas_n = 0; //bawah_6

					//cari atas_1 (misal titik setelahnya [i+1] bernilai nol, maka akan dicari nilai di atasnya yang tidak nol)
					for (int a = i + 1; a <= baris_garis; a++) {
						if (array_titik[a][y].y != 0) {
							atas_1 = a;
							break;
						}
					}

					//cari atas2 (misal titik setelahnya [atas_1+1] bernilai nol, maka akan dicari nilai di atasnya yang tidak nol)
					for (int a = atas_1 + 1; a <= baris_garis; a++) {
						if (array_titik[a][y].y != 0) {
							atas_2 = a;
							break;
						}
					}
					int n = 0;
					//cari atas_n (titik kedua bawah yang akan diregresi)
					for (int a = atas_1 + 1; a <= baris_garis; a++) {

						if (array_titik[a][y].y > 0) {
							n++;
							atas_n = a;
							if (n == index_atas) {// n+1 titik terbawah garis
								break;
							}
						}
					}
					if (atas_n > 0) {
						hitungRegresi(array_titik, y, atas_1, atas_n, line1_regA, line1_regB); // regresi 4 titik
						//Hitung_AB(array_titik[atas_1][y], array_titik[atas_2][y], line1_regA, line1_regB);

						if (line1_regB != 0) {
							regresi_cek.y = array_titik[i][y].y;
							regresi_cek.x = (regresi_cek.y - line1_regA) / line1_regB;
						}
						else {
							regresi_cek.x = lastx[y];
						}

						/*
						kalau hasil regresinya jauh (nilai x regresi jauh dengan nilai x titik di sebelahnya),
						maka titik yang ada di sebelahnya tidak masuk kelompok itu, melainkan dipindah kolom
						*/

						if (abs(array_titik[i][y].x - regresi_cek.x) >= 21) {
							for (int z = 6; z >= y; z--) {
								array_titik[i][z + 1] = array_titik[i][z];
								array_titik[i][z] = Point(0, 0);
							}
							if (y + 1 > Max_pointcount && array_titik[i][y + 1] != Point(0, 0)) {
								Max_pointcount = y + 1;
							}
						}
					}




				}

				if (array_titik[i][y].x > 0) {
					lastx[y] = array_titik[i][y].x;
				}
			}
		}

		//+++++++++++++ Verifikasi ke bawah +++++++++++++
		for (int i = FirstMax_pointcount + 1; i <= baris_garis; i++) {
			//for (int i = baris_garis; i >= FirstMax_pointcount + 1; i--) {
			for (int y = 1; y < Max_pointcount + 1; y++) {

				if (array_titik[i ][y].x > 0 && array_titik[i ][y].x < 2000) { //untuk membatasi supaya ga sampe nilai +/-89xxxxx 

					//selisih nilai x sekarang dan diatasnya
					if (abs(array_titik[i][y].x - array_titik[i - 1][y].x) > 31 &&
						abs(array_titik[i][y].x - array_titik[i + 1][y].x) <= 31 &&
						array_titik[i + 1][y] != Point(0, 0)) { //80
						//proses pindah (geser) posisi 

						cout << "bawah-1_geser" << i << " - x : " << array_titik[i][y].x << endl;
						for (int z = 6; z >= y; z--) {
							array_titik[i - 1][z + 1] = array_titik[i - 1][z];
							array_titik[i - 1][z] = Point(0, 0);
						}

						if (y + 1 > Max_pointcount && array_titik[i-1][y + 1] != Point(0, 0)) {
							Max_pointcount = y + 1;
						}
					}
					else if (abs(array_titik[i][y].x - array_titik[i - 1][y].x) > 31 &&
						array_titik[i][y] != Point(0, 0)) {
						cout << "bawahi_geser" << i << " - x : " << array_titik[i][y].x << endl;
						for (int z = 6; z >= y; z--) {
							array_titik[i][z + 1] = array_titik[i][z];
							array_titik[i][z] = Point(0, 0);
						}
					}
				}

				/*
					Untuk dapat melakukan regresi, jumlah minimum titik yang dibutuhkan ada 2
					jika tidak ada titik yang terdeteksi (0,0) maka, masuk ke else untuk mencari 2 titik
					di atasnya (cari atas1 dan cari atas2) agar bisa melakukan regresi
					Regresi (Regresi hanya dihitung jika nilai A dan B !=0)
					jika nilai hasil regresi adalah nol, maka yang digunakan nilai regresi yang terakhir sebelum nol
				*/

				else // jika nilai x array_titik diatasnya 0,0
				{
					int bawah_1 = 0, bawah_2 = 0;
					int index_bawah = 3, bawah_n = 0; //bawah_6

					//cari atas_1 (misal titik sebelumnya [i-1] bernilai nol, maka akan dicari nilai di atasnya yang tidak nol)
					for (int a = i - 1; a >= 0; a--) {
						if (array_titik[a][y].y != 0) {
							bawah_1 = a;
							break;
						}
					}

					//cari atas2 (misal titik sebelumnya [atas_1 - 1] bernilai nol, maka akan dicari nilai diatasnya yang tidak nol) 
					for (int a = bawah_1 - 1; a >= 0; a--) {
						if (array_titik[a][y].y != 0) {
							bawah_2 = a;
							break;
						}
					}

					int n = 0;
					//cari atas_n (titik kedua bawah yang akan diregresi)
					for (int a = bawah_1 - 1; a >= 0; a--) {

						if (array_titik[a][y].y > 0) {
							n++;
							bawah_n = a;
							if (n == index_bawah) {// max index = index_bawah diatas titik terbawah pada garis
								break;
							}
						}
					}

					if (bawah_n > 0) {
						//Hitung_AB(array_titik[bawah_1][y], array_titik[bawah_2][y], line1_regA, line1_regB);
						hitungRegresi(array_titik, y, bawah_n, bawah_1, line1_regA, line1_regB); // regresi 4 titik

						if (line1_regB != 0) {
							regresi_cek.y = array_titik[i][y].y;
							regresi_cek.x = (regresi_cek.y - line1_regA) / line1_regB;
						}

						else {
							regresi_cek.x = lastx[y];
						}

						/*
						kalau hasil regresinya jauh (nilai x regresi jauh dengan nilai x titik di sebelahnya),
						maka titik yang ada di sebelahnya tidak masuk kelompok itu, melainkan dipindah kolom
						*/
						if (abs(array_titik[i][y].x - regresi_cek.x) > 31) { // 120

							for (int z = 6; z >= y; z--) {
								array_titik[i][z + 1] = array_titik[i][z];
								array_titik[i][z] = Point(0, 0);

							}
							if (y + 1 > Max_pointcount && array_titik[i][y + 1] != Point(0, 0)) {
								Max_pointcount = y + 1;
							}
						}
					}


				}

				if (array_titik[i][y].x > 0) {
					lastx[y] = array_titik[i][y].x;
				}
			}
		}

		for (int y = 1; y < Max_pointcount + 1; y++) {
			lastx[y] = array_titik[FirstMax_pointcount][y].x;
		}

		// verifikasi minimal garis memiliki 4 titik
		for (int y = 1; y < 8; y++) {
			int totalTitik = 0;

			for (int i = 1; i < baris_garis; i++) {
				if (array_titik[i][y] != Point(0, 0)) {
					totalTitik++;
				}
			}

			if (totalTitik <= 4) {
				for (int z = y; z < 5; z++) {
					for (int i = 1; i < baris_garis; i++) {

						array_titik[i][z] = array_titik[i][z + 1];
						array_titik[i][z + 1] = Point(0, 0);
					}
				}
			}

			totalTitik = 0;
		}

		//+++++++++++++ Verifikasi ke bawah TAMBAHAN +++++++++++++
		for (int i = 1; i <= baris_garis; i++) {
			//for (int i = baris_garis; i >= FirstMax_pointcount + 1; i--) {
			for (int y = 1; y < Max_pointcount + 1; y++) {

				if (array_titik[i][y].x > 0 && array_titik[i][y].x < 2000) { //untuk membatasi supaya ga sampe nilai +/-89xxxxx 

					//selisih nilai x sekarang dan diatasnya
					if (abs(array_titik[i][y].x - array_titik[i - 1][y].x) > 31 &&
						abs(array_titik[i][y].x - array_titik[i + 1][y].x) <= 31 &&
						array_titik[i + 1][y] != Point(0, 0)) { //80
						//proses pindah (geser) posisi 

						cout << "bawah-1_geser" << i << " - x : " << array_titik[i][y].x << endl;
						for (int z = 6; z >= y; z--) {
							array_titik[i - 1][z + 1] = array_titik[i - 1][z];
							array_titik[i - 1][z] = Point(0, 0);

						}
						if (y + 1 > Max_pointcount && array_titik[i-1][y + 1] != Point(0, 0)) {
							Max_pointcount = y + 1;
						}
					}
					else if (abs(array_titik[i][y].x - array_titik[i - 1][y].x) > 31 &&
						array_titik[i][y] != Point(0, 0)) {
						cout << "bawahi_geser" << i << " - x : " << array_titik[i][y].x << endl;
						for (int z = 6; z >= y; z--) {
							array_titik[i][z + 1] = array_titik[i][z];
							array_titik[i][z] = Point(0, 0);
							
						}
						if (y + 1 > Max_pointcount && array_titik[i][y + 1] != Point(0, 0)) {
							Max_pointcount = y + 1;
						}
					}
				}

				/*
					Untuk dapat melakukan regresi, jumlah minimum titik yang dibutuhkan ada 2
					jika tidak ada titik yang terdeteksi (0,0) maka, masuk ke else untuk mencari 2 titik
					di atasnya (cari atas1 dan cari atas2) agar bisa melakukan regresi
					Regresi (Regresi hanya dihitung jika nilai A dan B !=0)
					jika nilai hasil regresi adalah nol, maka yang digunakan nilai regresi yang terakhir sebelum nol
				*/

				else // jika nilai x array_titik diatasnya 0,0
				{
					int bawah_1 = 0, bawah_2 = 0;
					int index_bawah = 3, bawah_n = 0; //bawah_6

					//cari atas_1 (misal titik sebelumnya [i-1] bernilai nol, maka akan dicari nilai di atasnya yang tidak nol)
					for (int a = i - 1; a >= 0; a--) {
						if (array_titik[a][y].y != 0) {
							bawah_1 = a;
							break;
						}
					}

					//cari atas2 (misal titik sebelumnya [atas_1 - 1] bernilai nol, maka akan dicari nilai diatasnya yang tidak nol) 
					for (int a = bawah_1 - 1; a >= 0; a--) {
						if (array_titik[a][y].y != 0) {
							bawah_2 = a;
							break;
						}
					}

					int n = 0;
					//cari atas_n (titik kedua bawah yang akan diregresi)
					for (int a = bawah_1 - 1; a >= 0; a--) {

						if (array_titik[a][y].y > 0) {
							n++;
							bawah_n = a;
							if (n == index_bawah) {// max index = index_bawah diatas titik terbawah pada garis
								break;
							}
						}
					}

					if (bawah_n > 0) {
						//Hitung_AB(array_titik[bawah_1][y], array_titik[bawah_2][y], line1_regA, line1_regB);
						hitungRegresi(array_titik, y, bawah_n, bawah_1, line1_regA, line1_regB); // regresi 4 titik

						if (line1_regB != 0) {
							regresi_cek.y = array_titik[i][y].y;
							regresi_cek.x = (regresi_cek.y - line1_regA) / line1_regB;
						}

						else {
							regresi_cek.x = lastx[y];
						}

						/*
						kalau hasil regresinya jauh (nilai x regresi jauh dengan nilai x titik di sebelahnya),
						maka titik yang ada di sebelahnya tidak masuk kelompok itu, melainkan dipindah kolom
						*/
						if (abs(array_titik[i][y].x - regresi_cek.x) > 21) { // 120

							for (int z = 6; z >= y; z--) {
								array_titik[i][z + 1] = array_titik[i][z];
								array_titik[i][z] = Point(0, 0);
							}
							if (y + 1 > Max_pointcount && array_titik[i][y + 1] != Point(0, 0)) {
								Max_pointcount = y + 1;
							}
						}
					}


				}

				if (array_titik[i][y].x > 0) {
					lastx[y] = array_titik[i][y].x;
				}
			}
		}

		for (int y = 1; y < Max_pointcount + 1; y++) {
			lastx[y] = array_titik[FirstMax_pointcount][y].x;
		}

		//===================================== REGRESI ===========================================//

		for (int x = 1; x <= Max_pointcount; x++) {

			//+++++++++++++ Regresi Atas +++++++++++++
			int atas_1 = 0, atas_2 = 0;

			//cari atas1 (titik pertama atas yang akan diregresi)
			for (int i = 1; i <= baris_garis; i++) {
				if (array_titik[i][x].y != 0) {
					atas_1 = i;
					break;
				}
			}

			//cari atas2 (titik kedua bawah yang akan diregresi)
			for (int i = atas_1 + 1; i <= baris_garis; i++) {
				if (array_titik[i][x].y != 0) {
					atas_2 = i;
					break;
				}
			}

			//Hitung Regresi Atas
			Hitung_AB(array_titik[atas_1][x], array_titik[atas_2][x], line1_regA, line1_regB);

			array_titik[0][x].y = imgOriginal.rows - tinggi_atas;
			array_titik[0][x].x = (array_titik[0][x].y - line1_regA) / line1_regB;

			//Agar Hasil Regresi Tidak Melebihi ROI
			if (array_titik[0][x].x < rook_points[3].x)
			{
				array_titik[0][x] = Point(0.0);
			}
			else if (array_titik[0][x].x > rook_points[2].x)
			{
				array_titik[0][x] = Point(0.0);
			}

			//+++++++++++++ Regresi Bawah +++++++++++++

			int bawah_1 = 0, bawah_2 = 0;

			//cari bawah1 (titik pertama bawah yang akan diregresi)
			for (int i = baris_garis - 1; i > 0; i--) {
				if (array_titik[i][x].y != 0) {
					bawah_1 = i;
					break;
				}
			}

			//cari bawah2 (titik kedua bawah yang akan diregresi)
			for (int i = bawah_1 - 1; i > 0; i--) {
				if (array_titik[i][x].y != 0) {
					bawah_2 = i;
					break;
				}
			}

			//cout << "bawah1 : " << bawah_1 << "bawah2: " << bawah_2 << endl;

			//Hitung Regresi Bawah
			Hitung_AB(array_titik[bawah_1][x], array_titik[bawah_2][x], line1_regA, line1_regB);
			array_titik[baris_garis][x].y = imgOriginal.rows;
			array_titik[baris_garis][x].x = (array_titik[baris_garis][x].y - line1_regA) / line1_regB;

			//Agar Hasil Regresi Tidak Melebihi ROI
			if (array_titik[baris_garis][x].x < rook_points[0].x)
			{
				array_titik[baris_garis][x] = Point(0.0);
			}
			else if (array_titik[baris_garis][x].x > rook_points[1].x)
			{
				array_titik[baris_garis][x] = Point(0.0);
			}
		}

		//print data
		//Max_pointcount = 0;
		if (debuggingMode == 1) { // print nilai array
			//+++++++++++++ Print array stored sesudah Verifikasi +++++++++++++ //HASIL AKHIR ARRAY GARIS
			cout << "===== SESUDAH FERIVIKASI =====" << endl;

			for (int k = 0; k < baris_garis + 1; k++) {
				for (int i = 1; i < 7; i++) {
					cout << array_titik[k][i] << "    ";
					if (array_titik[k][i] != Point(0, 0) && Max_pointcount < i) {
						Max_pointcount = i; // update max garis
					}
				}
				cout << endl;
			}

			cout << "Max_pointcount =" << Max_pointcount << endl;
		}


		//Hitung lebar titik putih - garis jalan
		for (int i = array_titik[baris_garis - 1][2].x; i < imgcols; i++) {

			nilai = (int)imgThresholded.at<uchar>(array_titik[baris_garis - 1][2].y, i);

			if (!nilai) {
				int lebar_garis = i - (array_titik[baris_garis - 1][2].x);
				cout << " lebar garis = " << lebar_garis << endl;
				break;
			}
		}

		//===================================== Gambar Garis ===========================================

		for (int i = 1; i < Max_pointcount + 1; i++) {
			int lasty = 0;
			for (int k = 1; k <= baris_garis; k++) {

				//Gambar garis yang terdeteksi
				if (array_titik[k][i].x > 0 && array_titik[k - 1][i].x > 0) {
					line(imgOriginal, array_titik[k][i], array_titik[k - 1][i], Scalar(255, 0, 0), 4); //biru
				}

				//Gambar garis hasil regresi
				else if (array_titik[k][i].x > 0 && array_titik[lasty][i].x > 0) {
					line(imgOriginal, array_titik[k][i], array_titik[lasty][i], Scalar(0, 0, 255), 4); //merah
				}

				if (array_titik[k][i].x > 0) {
					lasty = k;
				}
			}
		}


		//===================================== Hitung Error ===========================================

		//Mencari nilai y di titik 1 tingkat di atas h (titik ketinggian pengukuran lebar jalan dan error)
		for (int i = 1; i <= Max_pointcount; i++) { //iterasi jumlah garis

			for (int j = 0; j <= baris_garis; j++) {	// iterasi di dalam suatu garis
				if (array_titik[j][i].y < h) {
					temp = j;	//nilai y di titik 1 tingkat di atas h
				}
				else break;
			}

			temp1 = temp; // nilai index x di titik 1 tingkat diatas h

			//kondisi jika nilai x di titik j tadi bernilai 0 (temp1-1)
			while (array_titik[temp1][i] == Point(0, 0)) {
				temp1 = temp1 - 1;
				kosong[i] = 1;
			}


			index1[i] = temp1;
			//cout << "index " << i << " = " << index1[i] << endl; // index titik terakhir tiap garis

			int n = 0;
			int index_bawah = 4, bawah_n = 0;
			//cari bawah2 (titik kedua bawah yang akan diregresi)
			for (int j = temp1 - 1; j > 0; j--) {
				if (array_titik[j][i] != Point(0, 0)) {
					if (array_titik[j][i].y != 0) {
						n++;
						bawah_n = j;
						if (n == index_bawah) {// 5 titik terbawah garis
							//bawah_n = j;
							break;
						}
					}
				}
				else
					break;
			}


			//putText(imgOriginal, std::to_string(array_titik[temp][i].y), Point(250, h - 160), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
			////
			//Gambar titik untuk regresi ketinggian (h) - untuk menentukan titik perhitungan error posisi

			if (array_titik[temp][i] == Point(0, 0)) {
				line(imgOriginal, array_titik[temp1][i], array_titik[temp1][i], Scalar(0, 255, 255), 5);
			}
			else {
				line(imgOriginal, array_titik[temp][i], array_titik[temp][i], Scalar(0, 0, 255), 5);
			}

			line(imgOriginal, array_titik[temp + 1][i], array_titik[temp + 1][i], Scalar(255, 255, 255), 5);
			////

			//Regresi ketinggian (h)
			//Hitung_AB(array_titik[temp1][i], array_titik[temp1 + 1][i], line1_regA, line1_regB);

			hitungRegresi(array_titik, i, bawah_n, temp1, line1_regA, line1_regB);
			//hitungRegresi(array_titik, i, baris_garis-7, baris_garis-3, line1_regA, line1_regB);
			regresi_cek.y = h;
			regresi_cek.x = (regresi_cek.y - line1_regA) / line1_regB;
			regB[i] = line1_regB;

			colsH[i] = regresi_cek.x;	//nilai x di garis putih di titik h

			for (int j = 1; j <= baris_garis; j++) {	// isi nilai y di semua titik
				if (array_titik[j][i].y == 0 && array_titik[j - 1][i].y != 0) {
					array_titik[j][i].y = array_titik[j - 1][i].y + 9;
				}
			}
			for (int j = 0; j <= baris_garis; j++) {	// iterasi di dalam suatu garis
				if (array_titik[j][i].y < h2) {
					temp_belok = j;	//nilai y di titik 1 tingkat di atas h2
				}
				else break;
			}
			temp1_belok = temp_belok;
			if (colsH[i] >= tengah_frame) { // jika garis i di sebelah kanan maka colsH2 = 640
				/*line(imgOriginal,
					Point(tengah_frame, array_titik[temp_belok][i].y),
					array_titik[temp_belok][i],
					Scalar(255, 0, 50), 2);*/
				cout << "kanan" << endl;
				if (array_titik[temp_belok][i].x == 0) {
					colsH2[i] = imgcols; //640
				}
				else {
					colsH2[i] = array_titik[temp_belok][i].x;
				}
			} else if (colsH[i] < tengah_frame) {// jika garis i di sebelah kanan maka colsH2 = 0
				/*line(imgOriginal,
					array_titik[temp_belok][i],
					Point(tengah_frame, array_titik[temp_belok][i].y),
					Scalar(5, 255, 55), 4);*/
				cout << "kiri" << endl;
				colsH2[i] = array_titik[temp_belok][i].x; //0
			}

		}

		
		// untuk menentukan index garis kanan jalan (temp_garis);
		for (int i = 1; i <= Max_pointcount; i++) {
			if (colsH[i] >= tengah_frame)
			{
				temp_garis = i;
				break;
			}
		}	
		if (temp_garis <= 1) {
			for (int i = 1; i <= Max_pointcount; i++) {
				if (colsH[i] < tengah_frame)
				{
					temp_garis = i + 1;
					break;
				}
				else if (colsH[i] > tengah_frame) {
					temp_garis = i;
					break;
				}
			}
		}
		

		// untuk menentukan ada persimpangan jalan atau tidak
		if (colsH2[temp_garis] == 640) {
			belokH2 += 2; //belok kanan 
		}
		if (colsH2[temp_garis - 1] == 0) {
			belokH2 += 1; //belok kiri
		}
		
		/*if (temp_garis <= 1) {
			temp_garis = 2;
		}*/
		//temp_garis = 3;
		//if (array_titik[temp_garis][])
		if (abs(lebarJalan) < 1500) {
			lastLebarJalanH1 = lebarJalan;
		}
		lebarJalan = ((colsH[temp_garis] - colsH[temp_garis - 1]));
		int lebarJalanH2 = ((colsH2[temp_garis] - colsH2[temp_garis - 1]));
		int ref_tengahH2 = ((colsH2[temp_garis] + colsH2[temp_garis - 1]) / 2);
		
		cout << "colsH :" << colsH[temp_garis - 1] << "\t-\t" << colsH[temp_garis] << endl;
		cout << "colsH2 :" << colsH2[temp_garis - 1] << "\t-\t" << colsH2[temp_garis] << endl;
		cout << "lebarJalan = " << lebarJalan << endl; 
		cout << "lebarJalan H2 = " << lebarJalanH2<< endl;
		cout << "errorTest = " << errorH1Test << endl;
		cout << "last_lebarJalan = " << lastLebarJalanH1 << endl;
		//Titik Tengah
		line(imgOriginal, Point(imgOriginal.cols / 2, h), Point(imgOriginal.cols / 2, h), Scalar(255, 255, 255), 8);

		line(imgOriginal, Point(imgOriginal.cols / 2, h2), Point(imgOriginal.cols / 2, h2), Scalar(255, 255, 255), 8);
		line(imgOriginal, Point(colsH[temp_garis - 1], h), Point( (colsH[temp_garis]), h), Scalar(0, 0, 255), 2);

		line(imgOriginal, Point(ref_tengahH2, h2), Point(ref_tengahH2, h2+20), Scalar(15, 10, 255), 2);
		line(imgOriginal, Point(colsH2[temp_garis - 1], h2), Point((colsH2[temp_garis]), h2), Scalar(255, 0, 25), 2);

		prevError = error;
		prevRefSusur = ref_susur;
		//Error Dua Garis
		if (temp_garis > 1) {
			if (!(kosong[temp_garis] == 0 ^ kosong[temp_garis - 1] == 0)) {//colsH[temp_garis] > 0 && colsH[temp_garis - 1] > 0 && colsH[temp_garis] <1000 && colsH[temp_garis - 1] <1000) {
				//////

				ref_titik_tengah = colsH[temp_garis - 1] + ((colsH[temp_garis] - colsH[temp_garis - 1]) / 2);

				ref_susur = colsH[temp_garis] - ref_titik_tengah;
				if (susurTest == 0)
					susurTest = ref_susur;
				else if (abs(ref_susur - susurTest) <= 140) {
					susurTest = (susurTest + ref_susur) / 2;
				}
				error = ref_titik_tengah - (imgOriginal.cols / 2);

				errorH1Test = ((imgOriginal.cols / 2) - colsH[temp_garis - 1]);
				errorH1Test = 100*( errorH1Test/ lastlebarJalanH1 );

				Last_ref_titik_tengah = ref_titik_tengah;
				line(imgOriginal, Point(ref_titik_tengah, h), Point(ref_titik_tengah, 480), Scalar(0, 0, 255), 2);


				cout << "error 2 garis : " << (colsH[temp_garis] - colsH[temp_garis - 1]) << endl;

			}

			//Error Garis Kiri
			else if (kosong[temp_garis] == 1) {//colsH[temp_garis] <= 0) {
				ref_susur = susurTest;
				ref_titik_tengah = ref_susur + colsH[temp_garis - 1]; //colsH[temp_garis - 1] - ref_susur; **REVISI**
				line(imgOriginal, Point(ref_titik_tengah, h), Point(ref_titik_tengah, 450), Scalar(255, 255, 0), 3);

				error = ref_titik_tengah - imgOriginal.cols / 2;
				errorH1Test = 100*((imgOriginal.cols / 2) - colsH[temp_garis - 1]) / lastLebarJalanH1;
				cout << "error garis kiri" << ref_susur << endl;
				//cout << "error garis kiri" << ref_susur << endl;
			}

			//Error garis kanan
			else if (kosong[temp_garis - 1] == 1) {//colsH[temp_garis - 1] < 0) {
				ref_susur = susurTest;
				ref_titik_tengah = colsH[temp_garis] - ref_susur;
				line(imgOriginal, Point(ref_titik_tengah, h), Point(ref_titik_tengah, 450), Scalar(0, 255, 255), 3);

				error = ref_titik_tengah - imgOriginal.cols / 2;
				errorH1Test =  100*(1 - ( (colsH[temp_garis] - (imgOriginal.cols / 2)) / lastLebarJalanH1 ));

				cout << "error garis kanan : " << ref_susur << endl;
			}

			else if (error < -50 || error >  50) {
				ref_titik_tengah = Last_ref_titik_tengah;
				line(imgOriginal, Point(ref_titik_tengah, h), Point(ref_titik_tengah, h), Scalar(255, 255, 0), 3);

				error = ref_titik_tengah - imgOriginal.cols / 2;
				errorH1Test = lastErrorH1Test;

				cout << "error kelebihan : " << error << endl;
			}
		}
		else {
			//tes algoritm 1 jalur
			if (colsH[temp_garis] < imgOriginal.cols / 2) {
				error = (colsH[temp_garis] + susurTest) - imgOriginal.cols / 2;
			}
			else {
				error = (colsH[temp_garis] - susurTest) - imgOriginal.cols / 2;
			}
		}
		error = error * 250 / lastLebarJalanH1;

		Last_ref_titik_tengah = ref_titik_tengah;
		error_f = ((regB[temp_garis] + regB[temp_garis - 1] + 0.0901) / 0.0299); ///0.0099);// error * 250;// / lebarJalan;
		if ((error) < -200 || error > 200) // dipakai saat sample video
			error = prevError;

		
		if (belokH2 == 0) { // tanda hanya ada jalur lurus
			int koreksiH2 = 0;
			errorh2 = ((colsH2[temp_garis] + colsH2[temp_garis - 1]) / 2) - 320;
			errorh2 = (errorh2 + koreksiH2) * 250 / lebarJalan; // conversi 
		}
		else if (belokH2 == 1) { // tanda ada belokan ke kiri
			errorh2 = -500;
		}
		else if (belokH2 == 2) {
			errorh2 = 500;
		}
		else {
			errorh2 = 999;
		}

		// Print Data
		cout << "error = " << error << endl;
		cout << "errorh2 = " << errorh2 << endl;
		cout << regB[temp_garis - 1] << "\t" << regB[temp_garis] << endl;
		cout << error_f << endl;
		//Print nilai error 
		putText(imgOriginal, "error :" + std::to_string(error), Point(240, h), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, "errorH2 :" + std::to_string(errorh2), Point(240, h - 40), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, "jmlGaris :" + std::to_string(Max_pointcount), Point(240, h - 80), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, "LebarJalanH1 :" + std::to_string(lebarJalan), Point(240, h - 120), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, "LebarJalanH2 :" + std::to_string(lebarJalanH2), Point(240, h - 160), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, "susurTestH1 :" + std::to_string(320 - (colsH[temp_garis-1])), Point(240, h - 200), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, "errorH1Test :" + std::to_string(errorH1Test), Point(240, h - 240), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);

		//putText(imgOriginal, "ref_tengah :" + std::to_string(ref_titik_tengah), Point(240, h - 40), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, "jml_garis :" + std::to_string(temp_garis), Point(240, h - 80), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH[1]), Point(80, h - 120), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH[2]), Point(160, h - 120), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH[3]), Point(240, h - 120), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH[4]), Point(320, h - 120), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH[5]), Point(400, h - 120), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH[6]), Point(480, h - 120), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH2[1]), Point(80, h - 80), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH2[2]), Point(160, h - 80), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH2[3]), Point(240, h - 80), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH2[4]), Point(320, h - 80), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH2[5]), Point(400, h - 80), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//putText(imgOriginal, std::to_string(colsH2[6]), Point(480, h - 80), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		
		
		//putText(imgOriginal, "test :" + std::to_string((error_f)), Point(240, h - 240), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//================================================================================================================================//

		//resize(imgOriginal, imgResized, Size(640, 480));
		//imshow("imgResized", imgResized);
		//imshow("Thresholded", imgThresholded);
		//imshow("hsv1", hsv1);
		//imshow("HSV Filter", imgHSV);
		//imshow("imgContour area", imgHasil);
		//imshow("imgROI", imgThresholded);

		imshow(" imgOriginal", imgOriginal);
		//cout<<(imgHSV).mean()<<endl;

		// ----- membuat video output ------
		if (recording)
			video.write(imgOriginal); //extra


		char key = waitKey(33);
		if (key == 27) { break; }
	}
	destroyAllWindows();
	return 0;
}
