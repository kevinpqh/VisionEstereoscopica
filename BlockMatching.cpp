#include "header.h"

// Suaviza las distorciones del mapa de disparidadaridad
void suavizarDisparidad(Mat_<float> & disparidad, int numeroDisparidad) 
{
	Mat_<float> disparidad1;
	float ultimoPixel = 10;
	float mindisparidadarity = 30; // Número mínimo de disparidadaridades
	
	//Recorrido del mapa de disparidadaridad rellenando los vacios
	for (int i = 0; i < disparidad.rows; i++)
	{
		for (int j = numeroDisparidad; j < disparidad.cols; j++)
		{
			//Se llena con un valor por defecto
			if (disparidad(i, j) <= mindisparidadarity) 
				disparidad(i, j) = ultimoPixel;
			//Actualizacion del valor
			else 
				ultimoPixel = disparidad(i, j);
		}
	}
	int border = 4;
	//Ampliación del mapa de disparidadaridad
	copyMakeBorder(disparidad, disparidad1, border, border, border, border, BORDER_REPLICATE);
	Mat element = getStructuringElement(MORPH_ELLIPSE, Size(border * 2 + 1, border * 2 + 1));
	
	//Transformaciones de morfología
	morphologyEx(disparidad1, disparidad1, CV_MOP_OPEN, element); //Eliminar pequeñas regiones claras
	morphologyEx(disparidad1, disparidad1, CV_MOP_CLOSE, element); //Eliminar pequeñas regiones oscuras
	
	//Recorte del mapa de disparidadaridad
	disparidad = disparidad1(Range(border, disparidad.rows - border), Range(border, disparidad.cols - border)).clone();
}

// Calcula el mapa de disparidadaridades mediante el algoritmo semi-global block matching
void calculardisparidadaridad(Mat &imgIzquierda, Mat &imgDerecha, Mat_<float> &disparidad, int ndisparidad)  
{
	StereoSGBM sgbm;
	int nChannels = imgDerecha.channels();

	//Parámetros iniciales
	sgbm.SADWindowSize = 3;
	sgbm.numberOfdisparidadarities = ndisparidadaridad;
	sgbm.preFilterCap = 63;
	sgbm.P1 = 8 * nChannels * sgbm.SADWindowSize * sgbm.SADWindowSize;
	sgbm.P2 = 32 * nChannels * sgbm.SADWindowSize * sgbm.SADWindowSize;
	sgbm.mindisparidadarity = 0;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 32;
	sgbm.disparidad12MaxDiff = 1;
	sgbm.fullDP = false;

	Mat disparidadTemp, disparidad8;
	sgbm(imgIzquierda, imgDerecha, disparidadTemp);
	
	//Conversión a una matriz flotante de 32-bit de 1 canal
	disparidadTemp.convertTo(disparidadaridad, CV_32FC1, 1.0 / 16);
	
	//Conversión a una matriz unsigned de 8-bits (escala de grises)
	disparidad.convertTo(disparidad8, CV_8U, 255.0 / ndisparidad);
	imshow("disparidadaridad original", disparidad8);

	//Suavizar las distorciones
	Fixdisparidadarity(disparidad, ndisparidad);
	disparidad.convertTo(disparidad8, CV_8U, 255.0/ndisparidad);
	imshow("disparidadaridad suavizada", disparidad8);
}

// Elección de los puntos característicos con Block Matching
void ChooseKeyPointsBM(Mat_<float> &disparidadaridad, int ndisparidad, int nEdgePt, int nFlatPt,
					   vector<Point2f> &ptsL, vector<Point2f> &ptsR) 
{
	Mat_<float>  dCopy, dx, dy, dEdge;
	dCopy = disparidad.colRange(Range(ndisparidad, disparidad.cols)).clone();
	normalize(dCopy, dCopy, 0, 1, NORM_MINMAX);
	//imshow("disparidadaridad", dCopy);
	
	Mat dShow(dCopy.size(), CV_32FC3);
	if (dCopy.channels() == 1)
		cvtColor(dCopy, dShow, CV_GRAY2RGB);

	int sobelWinSz = 7;
	Sobel(dCopy, dx, -1, 1, 0, sobelWinSz);
	Sobel(dCopy, dy, -1, 0, 1, sobelWinSz);
	magnitude(dx, dy, dEdge);
	normalize(dEdge, dEdge, 0, 10, NORM_MINMAX);
	//imshow("Borde de la disparidadaridad", dEdge);

	int filterSz[] = {50, 30};
	float slope[] = {4, 8};
	int keepBorder = 5;
	int cnt = 0;
	double value;
	float minValue = .003;
	Point2f selPt1, selPt2;
	Mat_<float> dEdgeCopy1 = dEdge.clone();

	// Búsqueda de los bordes significativos y asigna 1 o 2 puntos característicos cerca a ellos
	while (cnt < nEdgePt)
	{
		Point loc;
		minMaxLoc(dEdgeCopy1, NULL, &value, NULL, &loc);
		if (value < minValue) break;

		float dx1 = dx(loc), dy1 = dy(loc);
		if (abs(dx1) >= abs(dy1))
		{
			selPt1.y = selPt2.y = loc.y;
			selPt1.x = loc.x - (dx1 > 0 ? slope[1] : slope[0]) + ndisparidad;
			selPt2.x = loc.x + (dx1 > 0 ? slope[0] : slope[1]) + ndisparidad;
			if (selPt1.x > keepBorder+ndisparidad)
			{
				ptsL.push_back(selPt1);
				ptsR.push_back(selPt1 - Point2f(disparidad(selPt1), 0));
				circle(dShow, selPt1-Point2f(ndisparidad, 0), 2, CV_RGB(255, 0, 0), 2);
				cnt++;
			}
			if (selPt2.x < disparidad.cols - keepBorder)
			{
				ptsL.push_back(selPt2);
				ptsR.push_back(selPt2 - Point2f(disparidad(selPt2), 0));
				circle(dShow, selPt2-Point2f(ndisparidad, 0), 2, CV_RGB(0, 255, 0), 2);
				cnt++;
			}
			
			int left = min(filterSz[1], loc.x),
				top = min(filterSz[0], loc.y),
				right = min(filterSz[1], dCopy.cols - loc.x - 1),
				bot = min(filterSz[0], dCopy.rows - loc.y - 1);
			Mat sub = dEdgeCopy1(Range(loc.y - top, loc.y + bot + 1), Range(loc.x - left, loc.x + right + 1));
			sub.setTo(Scalar(0));
		}
		else
		{
			selPt1.x = selPt2.x = loc.x+ndisparidad;
			selPt1.y = loc.y - (dy1 > 0 ? slope[1] : slope[0]);
			selPt2.y = loc.y + (dy1 > 0 ? slope[0] : slope[1]);
			if (selPt1.y > keepBorder)
			{
				ptsL.push_back(selPt1);
				ptsR.push_back(selPt1 - Point2f(disparidad(selPt1), 0));
				circle(dShow, selPt1-Point2f(ndisparidad, 0), 2, CV_RGB(255, 255, 0), 2);
				cnt++;
			}
			if (selPt2.y < disparidad.rows-keepBorder)
			{
				ptsL.push_back(selPt2);
				ptsR.push_back(selPt2 - Point2f(disparidad(selPt2), 0));
				circle(dShow, selPt2-Point2f(ndisparidad, 0), 2, CV_RGB(0, 255, 255), 2);
				cnt++;
			}

			int left = min(filterSz[0], loc.x),
				top = min(filterSz[1], loc.y),
				right = min(filterSz[0], dCopy.cols-loc.x - 1),
				bot = min(filterSz[1], dCopy.rows-loc.y - 1);
			Mat sub = dEdgeCopy1(Range(loc.y - top, loc.y + bot + 1), Range(loc.x - left, loc.x + right + 1));
			sub.setTo(Scalar(0));
		}
	}
	imshow("Puntos característicos en bordes",dShow);
	int filterSz0 = 6;
	keepBorder = 3;
	cnt = 0;
	Mat_<float> dEdgeCopy2;
	GaussianBlur(dEdge, dEdgeCopy2, Size(0, 0), 5);
	char str[10];

	// Búsqueda de las áreas planas, asignación de 1 punto característico cerca a ellas
	while (cnt < nFlatPt)
	{
		Point loc;
		minMaxLoc(dEdgeCopy2, &value, NULL, &loc, NULL);
		if (value == 10) break;

		loc.x += ndisparidad;
		if (loc.x > keepBorder + ndisparidad && loc.y > keepBorder &&
			loc.x < disparidad.cols && loc.y < disparidad.rows)
		{
			ptsL.push_back(loc);
			ptsR.push_back(Point2f(loc.x, loc.y) - Point2f(disparidad(loc), 0));
			circle(dShow, Point2f(loc.x, loc.y) - Point2f(ndisparidad, 0), 2, CV_RGB(255, 0, 255), 2);
			cnt++;
			snprintf(str, 10, "%.1f", disparidad(loc));
			putText(dShow, str, Point(loc.x - ndisparidad + 3, loc.y), FONT_HERSHEY_SIMPLEX, .3, CV_RGB(255, 0, 255));
		}

		loc.x -= ndisparidad;
		int filterSz1 = (10 - value * 3) * filterSz0;
		int left = min(filterSz1, loc.x),
			top = min(filterSz1, loc.y),
			right = min(filterSz1, dCopy.cols - loc.x - 1),
			bot = min(filterSz1, dCopy.rows-loc.y - 1);
		Mat sub = dEdgeCopy2(Range(loc.y - top, loc.y + bot + 1), Range(loc.x - left, loc.x + right + 1));
		sub.setTo(Scalar(10));
	}
	imshow("Puntos Característicos en planos",dShow);
}

void guardarDisparidad(Mat_<float> &disparidad){
	Mat disparidadGuardada;
	
	//Normalización entre 0 y 1
	normalize(disparidad, disparidadGuardada, 0, 1, NORM_MINMAX);
	
	//Conversión a escala de grises
	disparidadGuardada.convertTo(disparidadGuardada, CV_8U, 255);
	imwrite("disparidad.jpg", disparidadGuardada);
}

void GetPairBM(Mat &imgIzquierda, Mat &imgDerecha, vector<Point2f> &ptsL, vector<Point2f> &ptsR) 
{
	Mat_<float> disparidad;

	int numOfdisparidad = 80; // Número de disparidadaridades, divisible entre 16
	Calcdisparidadarity(imgIzquierda, imgDerecha, disparidad, numOfdisparidad);
	guardarDisparidad(disparidad);

	int numOfEgdePt = 80, numOfFlatPt = 50;
	ChooseKeyPointsBM(disparidad, numOfdisparidad, numOfEgdePt, numOfFlatPt, ptsL, ptsR);
	waitKey();
}
