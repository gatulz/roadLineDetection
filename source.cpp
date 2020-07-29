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
int HLow = 223;// 230;
int HHigh = 255;
int SLow = 227;
int SHigh = 255;
int VLow = 203;
int VHigh = 255;

//  ========== VARIABEL ROI ==========

Point rook_points[4];

int lebar_atas = 640;// 560;
int lebar_bawah = 640;
int tinggi_bawah = 0;
int tinggi_atas = 280;// 400;

//========== VARIABEL SEARCH LINE ==========
int pixel = 0;
int nilai = 0;
int lastpixel = 0;
int searchblack = 0;
int pixelskip = 100;

Mat imgOriginal, imgResized;
Mat imgHSV;

int FirstMax_y;

int lebar_garis;
int frame_width = 640;//640; //extra
int frame_height = 480;// 480; //extra

vector<Vec4i> hierarchy;

//========== ERROR ==========
int ref_susur = 0, prevRefSusur=0;

int printThreshold = 1;

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

void hitungRegresi(Point line[][6], int index, int min, int max, long double& regA, long double& regB) {

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
	kuadrattotalX = pow(totalX, 2); //3.519.376
	
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

	int error = 0, prevError = 0;
	float error_f = 0;
	int susurTest = 0;

	//------ akses video ---------

	VideoCapture cap("23juli/jalan_2.mp4"); 
	//VideoCapture cap("23juli/miringkanan_2.mp4");
	//VideoCapture cap("29juli2/jalan2.mp4");//miringkanan_2.mp4"); //Video bagus dari yutup
	//VideoCapture cap("/home/autodrive/IMG_1139.MOV"); //Video kamera iPhone8plus
	//VideoCapture cap("C:/Users/user/Desktop/multilinegambar/zebra2.mp4"); punya ka rumaisha

	// ----- akses video streaming -----//


	//VideoCapture cap(0);

	if (!cap.isOpened()) {
		cout << "Kamera tidak dapat di akses" << endl;
		return -1;
	}



	cap.set(CAP_PROP_FRAME_WIDTH, frame_width);// 640); //TADINYA PAKE CV_ DI DEPANNYA
	cap.set(CAP_PROP_FRAME_HEIGHT, frame_height);// 480);


	// ----- membuat video output ------ //NAMAFILE
	VideoWriter video("23juli_jalann2.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, Size(frame_width, frame_height));	//extra

	while (1) {

		//----- Akses Video ----
		cap >> imgOriginal;
		//------- Akses Gambar -----
		
		//imgOriginal = imread("screenshot/1/-50.jpg");
		//imgOriginal = imread("29juli2/ERROR_-50_3.jpg");
		resize(imgOriginal, imgOriginal, Size(640, 480));

		//Atur parameter HSV dengan taskbar
		Trackbar_HSV();
		inRange(imgOriginal, Scalar(HLow, SLow, VLow), Scalar(HHigh, SHigh, VHigh), imgHSV);
		imshow("HSV Filter", imgHSV);


		//========== VARIABEL IMAGE ==========
		int imgrows = imgOriginal.rows;
		int imgcols = imgOriginal.cols;
		int imgrowshalf = 0.56 * imgOriginal.rows; //extra //0.7


		//========== VARIABEL SORTED ARRAY ==========
		int linescan = imgrows / 50;
		int baris_garis = 1;
		int pointcount;
		int Max_pointcount = 0;
		int lastpointcount = 0;
		int FirstMax_pointcount = 0;
		int lastx[5];
		Point point_titik[200];
		int nilai_titik = 0;
		Point array_titik[100][6];
		Point regresi_cek;
		long double line1_regA, line1_regB, line2_regA, line2_regB;


		//========== ERROR ==========
		int temp;
		int temp1;
		int colsH[5];
		int kosong[5] = { 0,0,0,0,0 };
		int h = imgOriginal.rows*0.75;// 0.94;//0.986, 0.88;  //titik ketinggian garis merah varH
		int h2 = imgOriginal.rows * 0.7;
		int h3 = imgOriginal.rows * 0.6;
		int Last_ref_titik_tengah=0;
		int ref_titik_tengah = 0;
		int error_sudut = 0;
		colsH[1] = 0;
		colsH[2] = 0;
		temp = 0;
		int temp_garis = 0;
		int tengah_frame = imgOriginal.cols / 2;


		//========== ZEBRACROSS ==========
		int lebar = 0;
		int zebracross = 0;



		int index1[5] = { 0,0,0,0,0 };
		int indexY[5] = { 0,0,0,0,0 };


		//================================== PRE PROCESSING =========================================

		//HSV Filter
		Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
		inRange(imgOriginal, Scalar(HLow, SLow, VLow), Scalar(HHigh, SHigh, VHigh), imgHSV);

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
			if ((contourArea(contours[i]) > 500) && (contourArea(contours[i]) < 8000)) //500 - 8000 100 - 10000
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
				Max_pointcount = pointcount; //jumlah maximum titik tiap garis
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
				for (int i = 1; i < (Max_pointcount + 1); i++) { //x
					cout << array_titik[k][i] << "    ";
				}
				cout << endl;
			}

			cout << "Max_pointcount Sebelum =" << Max_pointcount << endl;
			cout << "First_Max_pointcount Sebelum =" << FirstMax_pointcount << endl;
		}
		//===================================== Verifikasi Array ===========================================

		/*
		Jika kolom berikutnya (max_pointcount+1) ada nilai yang tidak (0,0)
		maka max_pointcount = (max_pointcount + 1)
		*/
		
		//+++++++++++++ Verifikasi ke atas +++++++++++++
		for (int i = FirstMax_pointcount-1 ; i >= 0; i--) {
			for (int y = 1; y < Max_pointcount + 1; y++) {
				cout << i << y << endl;
				if (array_titik[i+1][y].x > 0 && array_titik[i+1][y].x < 2000) {

					//selisih nilai x sekarang dan dibawahnya
					if (abs(array_titik[i][y].x - array_titik[i + 1][y].x) > 31) {//80
						cout << "atas_geser" << i << " - x : " << array_titik[i][y].x <<" - x : " << array_titik[i+1][y].x << endl;
						//proses pindah (geser) posisi 
						for (int z = Max_pointcount; z >= y; z--) {
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

				else
				{
					int atas_1 = 0, atas_2 = 0, bawah_1 = 0, bawah_2 = 0;
					int index_atas = 3, atas_n=0; //bawah_6

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

						if (array_titik[a][y].y >0) {
							n++;
							atas_n = a;
							if (n == index_atas) {// n+1 titik terbawah garis
								break;
							}
						}
					}
					if (atas_n > 0 ) {
						hitungRegresi(array_titik, y, atas_1, atas_n, line1_regA, line1_regB); // regresi 4 titik
						//Hitung_AB(array_titik[atas_1][y], array_titik[atas_2][y], line1_regA, line1_regB);

						if (line1_regB != 0) { //line1_regA != 0 && line1_regB != 0) {
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

						if (abs(array_titik[i][y].x - regresi_cek.x) >= 51) {
							for (int z = Max_pointcount; z >= y; z--) {
								array_titik[i][z + 1] = array_titik[i][z];
								array_titik[i][z] = Point(0, 0);
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
		for (int i = FirstMax_pointcount+1 ; i <= baris_garis; i++) {
		//for (int i = baris_garis; i >= FirstMax_pointcount + 1; i--) {
			for (int y = 1; y < Max_pointcount + 1; y++) {

				if (array_titik[i-1][y].x > 0 && array_titik[i-1][y].x < 2000) { //untuk membatasi supaya ga sampe nilai +/-89xxxxx 

					//selisih nilai x sekarang dan diatasnya
					if (abs(array_titik[i][y].x - array_titik[i - 1][y].x) > 51 ){//&& 
						//abs(array_titik[i][y].x - array_titik[i + 1][y].x) <= 51) { //80
						//proses pindah (geser) posisi 

						cout << "bawah-1_geser" << i << " - x : " << array_titik[i][y].x << endl;
						for (int z = Max_pointcount; z >= y; z--) {
							array_titik[i][z + 1] = array_titik[i][z];
							array_titik[i][z] = Point(0, 0);
						}
					}
					/*else if (abs(array_titik[i][y].x - array_titik[i - 1][y].x) > 51) {
						cout << "bawahi_geser" << i << " - x : " << array_titik[i][y].x << endl;
						for (int z = Max_pointcount; z >= y; z--) {
							array_titik[i][z + 1] = array_titik[i][z];
							array_titik[i][z] = Point(0, 0);
						}
					}*/
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
						if (abs(array_titik[i][y].x - regresi_cek.x) > 51) { // 120

							for (int z = Max_pointcount; z >= y; z--) {
								array_titik[i][z + 1] = array_titik[i][z];
								array_titik[i][z] = Point(0, 0);
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



		for (int y = 1; y < Max_pointcount + 1; y++) {
			int totalTitik = 0;

			for (int i = 1; i < baris_garis; i++) {
				if (array_titik[i][y] != Point(0, 0)) {
					totalTitik++;
				}
			}

			if (totalTitik < 2) {
				for (int z = y; z <= Max_pointcount; z++) {
					for (int i = 1; i < baris_garis; i++) {

						array_titik[i][z] = array_titik[i][z + 1];
						array_titik[i][z + 1] = Point(0, 0);
					}
				}
			}

			totalTitik = 0;
		}


		//===================================== REGRESI ===========================================

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

		if (debuggingMode == 1) {
			//+++++++++++++ Print array stored sesudah Verifikasi +++++++++++++ //HASIL AKHIR ARRAY GARIS
			cout << "===== SESUDAH FERIVIKASI =====" << endl;

			for (int k = 0; k < baris_garis + 1; k++) {
				for (int i = 1; i < 5; i++) {
					cout << array_titik[k][i] << "    ";
					if (array_titik[k][i] != Point(0, 0) && Max_pointcount<i) {
						Max_pointcount = i;
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


		/*sebelum masuk sini
		buat algoritma isi array yg kosong 
		PENTING
		
		
		*/
		
		if (isiArray == 1) {
			for (int i = 1; i <= Max_pointcount; i++) { //iterasi jumlah garis
				int bawah_1 = 0, bawah_2 = 0, bawah_n = 0;
				int index_bawah = 4; //bawah_6

				//cari bawah1 (titik pertama bawah yang akan diregresi)
				for (int j = baris_garis - 1; j > 0; j--) {
					if (array_titik[j][i].y != 0) {
						bawah_1 = j;
						break;
					}
				}

				for (int j = bawah_1 - 1; j > 0; j--) {
					if (array_titik[j][i].y != 0) {
						bawah_2 = j;
						break;
					}
				}
				int n = 0;
				//cari bawah2 (titik kedua bawah yang akan diregresi)
				for (int j = bawah_1 - 1; j > 0; j--) {

					if (array_titik[j][i].y != 0) {
						n++;
						if (n == index_bawah) {// 6 titik terbawah garis
							bawah_n = j;
							break;
						}
					}
				}
				//cout << "bawah1 : " << bawah_1 << "\tbawah2: " << bawah_2 << "\tbawah_" << index_bawah << ": " << bawah_n << endl;

				for (int j = 0; j <= baris_garis; j++) {	// iterasi di dalam suatu garis
					if (array_titik[j][i].y < h) {
						temp = j;	//nilai y di titik 1 tingkat di atas h
					}
					else break;
				}
				index1[i] = temp;
				//cout << "index " << i << " = " << index1[i] << endl; // index titik terakhir tiap garis


				for (int j = 0; j <= baris_garis; j++) {	// iterasi di dalam suatu garis
					if (j != 0 && array_titik[j][i].y == 0 && array_titik[j - 1][i].y != 0) {
						array_titik[j][i].y = array_titik[j - 1][i].y + 9;
					}
				}

				/*hitungRegresi(array_titik, i, bawah_2, bawah_1, line1_regA, line1_regB);
				cout << "21 - REGA : " << line1_regA << "REGB : " << line1_regB << endl;
				array_titik[24][i].x = (array_titik[24][i].y - line1_regA) / line1_regB;
				cout << "x : " << array_titik[24][i].x << "\t y : " << array_titik[24][i].y << endl;
				hitungRegresi(array_titik, i, bawah_n, bawah_1, line1_regA, line1_regB);
				cout << "31 - REGA : " << line1_regA << "REGB : " << line1_regB << endl;
				array_titik[24][i].x = (array_titik[24][i].y - line1_regA) / line1_regB;
				cout << "x : " << array_titik[24][i].x << "\t y : " << array_titik[24][i].y << endl;
				array_titik[24][i].x = 0;
				hitungRegresi(array_titik, i, bawah_1-5, bawah_1, line1_regA, line1_regB);
				cout << "61 - REGA : " << line1_regA << "REGB : " << line1_regB << endl;
				array_titik[24][i].x = (array_titik[24][i].y - line1_regA) / line1_regB;
				cout << "x : " << array_titik[24][i].x << "\t y : " << array_titik[24][i].y << endl;
				array_titik[24][i].x = 0;
				Hitung_AB(array_titik[bawah_1][i], array_titik[bawah_2][i], line1_regA, line1_regB);
				*/

				hitungRegresi(array_titik, i, bawah_n, bawah_1, line1_regA, line1_regB);
				//cout << "ASLI - REGA : " << line1_regA << "\tREGB : " << line1_regB << endl;

				for (int j = bawah_1 + 1; j <= baris_garis; j++) {
					if (array_titik[j][i].x == 0) {
						// LAKUKAN REGRESSI dari 2 titik terbawah sehingga didapat nilai x di titik ketinggian h

					//Hitung Regresi Bawah
						array_titik[j][i].x = (array_titik[j][i].y - line1_regA) / line1_regB;

					}
				}

			}
		}
		
		

		/*if (debuggingMode == 1) {
			cout << "===== SESUDAH REVISI =====" << endl;

			for (int k = 0; k < baris_garis + 1; k++) {
				for (int i = 1; i < (Max_pointcount + 4); i++) {
					cout << array_titik[k][i] << "    ";
				}
				cout << endl;
			}

			cout << "Max_pointcount =" << Max_pointcount << endl;
			cout << "===========================" << endl;
		}*/

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
			int index_bawah = 4, bawah_n=0;
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
			regresi_cek.y = h;
			regresi_cek.x = (regresi_cek.y - line1_regA) / line1_regB;

			colsH[i] = regresi_cek.x;	//nilai x di garis putih di titik h
		}

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
					temp_garis = i+1;
					break;
				}
				else if (colsH[i] > tengah_frame) {
					temp_garis = i;
					break;
				}
			}
		} 
		/*if (temp_garis <= 1) {
			temp_garis = 2;
		}*/
		//temp_garis = 3;
		//if (array_titik[temp_garis][])
		int lebarJalan = ((colsH[temp_garis] - colsH[temp_garis - 1]));
		cout <<"colsH :"<< colsH[temp_garis-1] << "\t-\t" << colsH[temp_garis ] << endl;
		cout << "lebar jalan = " << ((colsH[temp_garis] - colsH[temp_garis - 1]) ) << endl; ;// colsH[temp_garis - 1] + (colsH[temp_garis] - colsH[temp_garis - 1]) << endl;
		line(imgOriginal, Point(colsH[temp_garis - 1], h), Point(colsH[temp_garis - 1] + (colsH[temp_garis] - colsH[temp_garis - 1]), h), Scalar(0, 0, 255), 2);

		prevError = error;
		prevRefSusur = ref_susur;
		//Error Dua Garis
		if (temp_garis > 1) {
			if (!(kosong[temp_garis]==0 ^ kosong[temp_garis-1] == 0) ){//colsH[temp_garis] > 0 && colsH[temp_garis - 1] > 0) {//

				ref_titik_tengah = colsH[temp_garis - 1] + ((colsH[temp_garis] - colsH[temp_garis - 1]) / 2);

				ref_susur = colsH[temp_garis] - ref_titik_tengah;
				if (susurTest == 0)
					susurTest = ref_susur;
				else if (abs(ref_susur - susurTest) < 100) {
					susurTest = (susurTest + ref_susur) / 2;
				}
				error = ref_titik_tengah - (imgOriginal.cols / 2);

				Last_ref_titik_tengah = ref_titik_tengah;
				line(imgOriginal, Point(ref_titik_tengah, h), Point(ref_titik_tengah, 480), Scalar(0, 0, 255), 2);


				cout << "error 2 garis : " << (colsH[temp_garis] - colsH[temp_garis - 1]) << endl;

			}

			//Error Garis Kiri
			else if (kosong[temp_garis] == 1) {//colsH[temp_garis] <= 0) {

				ref_titik_tengah = ref_susur + colsH[temp_garis - 1]; //colsH[temp_garis - 1] - ref_susur; **REVISI**
				line(imgOriginal, Point(ref_titik_tengah, h), Point(ref_titik_tengah, 450), Scalar(255, 255, 0), 3);

				error = ref_titik_tengah - imgOriginal.cols / 2;

				cout << "error garis kiri" << ref_susur << endl;
				//cout << "error garis kiri" << ref_susur << endl;
			}

			//Error garis kanan
			else if (kosong[temp_garis - 1] == 1) {//colsH[temp_garis - 1] < 0) {
				ref_titik_tengah = colsH[temp_garis] - ref_susur;
				line(imgOriginal, Point(ref_titik_tengah, h), Point(ref_titik_tengah, 450), Scalar(0, 255, 255), 3);

				error = ref_titik_tengah - imgOriginal.cols / 2;

				cout << "error garis kanan : " << ref_susur << endl;
			}

			else if (error < -50 || error >  50) {
				ref_titik_tengah = Last_ref_titik_tengah;
				line(imgOriginal, Point(ref_titik_tengah, h), Point(ref_titik_tengah, h), Scalar(255, 255, 0), 3);

				error = ref_titik_tengah - imgOriginal.cols / 2;

				cout << "error kelebihan : " << error << endl;
			}
		}
		else {
			//tes algoritm 1 jalur
			if (colsH[temp_garis] < imgOriginal.cols / 2) {
				error = (colsH[temp_garis] + susurTest)-imgOriginal.cols / 2;
			}
			else {
				error = (colsH[temp_garis] - susurTest) - imgOriginal.cols / 2;
			}
		}
		
		Last_ref_titik_tengah = ref_titik_tengah;
		error_f = 0;// error * 250;// / lebarJalan;
		//if (abs(error - prevError) > 150) // dipakai saat sample video
		//	error = prevError;

		//Titik Tengah
		line(imgOriginal, Point(imgOriginal.cols / 2, h), Point(imgOriginal.cols / 2, h), Scalar(255, 255, 255), 8);

		cout << "error = " << error << endl;
		//Print nilai error 
		putText(imgOriginal, "error :" + std::to_string(error), Point(240, h),FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, "ref_tengah :" + std::to_string(ref_titik_tengah), Point(240, h - 40), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, "jml_garis :" + std::to_string(temp_garis), Point(240, h - 80), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, std::to_string(colsH[1]), Point(180, h - 120), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, std::to_string(colsH[2]), Point(260, h - 120), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		putText(imgOriginal, "LebarJalan :" + std::to_string(lebarJalan), Point(240, h - 160), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);

		putText(imgOriginal, "susurTest :" + std::to_string(susurTest), Point(240, h - 200), FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 0), 2, LINE_AA);
		//================================================================================================================================//


		resize(imgOriginal, imgResized, Size(640, 480));
		//imshow("imgResized", imgResized);
		//imshow("Thresholded", imgThresholded);
		imshow("imgContour area", imgHasil);
		imshow("imgOriginal", imgOriginal);

		// ----- membuat video output ------
		video.write(imgOriginal); //extra


		char key = waitKey(33);
		if (key == 27) { break; }
	}
	destroyAllWindows();
	return 0;
}
