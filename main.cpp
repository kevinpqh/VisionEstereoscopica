#include<iostream>
#include "header.h"

using namespace std;

namespace reconstruction
{
	enum Algorithm {FEATURE_POINT, BLOCK_MATCHING};
};

using namespace reconstruction;

int main (int argc, char *argv[]) {
	std::cout<<"Ejecutar"<<std::endl;
	
	reconstruction::Algorithm g_algo;
	
	int value = 0;
	std::cout<<"Seleccione Algoritmo:"<<std::endl;
	std::cout<<"1: FEATURE POINT:"<<std::endl;
	std::cout<<"2: BLOCK MATCHING"<<std::endl;
	std::cout<<"> ";
	std::cin>>value;
	
	if (value == 1){
		g_algo = FEATURE_POINT;
	} else 	if (value == 2){
		g_algo = BLOCK_MATCHING;
	}
	else{
		std::cout<<"Opcion Incorrecta FIN"<<std::endl;
		exit(1);
	}
	
	///Carga de las imágenes
	String filename = "plastic.png";
	Mat imgL = imread("Images/left/"+ filename); 
	Mat	imgR = imread("Images/right/"+ filename);
	
	imshow("Imagen Izquierda", imgL);
	imshow("Imagen Derecha", imgR);
	
	waitKey(0);
	
	if (!(imgL.data) || !(imgR.data))
	{
		cerr<<"No se pudo cargar imagen!"<<endl;
		exit(1);
	}
	
	/************************************************************************/
	///Redimensión de las imágenes                                          */
	/************************************************************************/
	float stdWidth = 800, resizeScale = 1;
	if (imgL.cols > stdWidth * 1.2)
	{
		resizeScale = stdWidth / imgL.cols;
		Mat imgL1, imgR1;
		resize(imgL, imgL1, Size(), resizeScale, resizeScale);
		resize(imgR, imgR1, Size(), resizeScale, resizeScale);
		imgL = imgL1.clone();
		imgR = imgR1.clone();
	}
	
	/************************************************************************/
	/// Elección de los puntos característicos en la imagen izquierda        */
	/// Cálculo de los puntos correspondientes en la imagen derecha           */
	/************************************************************************/
	cout << "Calculando puntos caracteristicos ..." << endl;
	vector<Point2f> ptsL, ptsR;
	vector<int> ptNum;
	if (g_algo == FEATURE_POINT)
		GetPair(imgL, imgR, ptsL, ptsR);
	else if (g_algo == BLOCK_MATCHING)
		GetPairBM(imgL, imgR, ptsL, ptsR);
	
	/************************************************************************/
	/* Cálculo de las coordenadas 3D                                        */
	/************************************************************************/
	vector<Point3f> pts3D;
	float focalLenInPixel = 3740 * resizeScale, baselineInMM = 160;
	Point3f center3D;
	Vec3f size3D;
	float scale = 0.2; // Escala de la coordenada z para concentrar el espacio
	focalLenInPixel *= scale;
	
	cout << "Calculando coordenadas 3D ..." << endl;
	StereoTo3D(ptsL, ptsR, pts3D, focalLenInPixel, baselineInMM, imgL, center3D, size3D);
	
	/************************************************************************/
	/* Triangulación de Delaunay                                            */
	/************************************************************************/
	cout << "Ejecutando triangulacion ..." << endl;
	vector<Vec3i> tri;
	TriSubDiv(ptsL, imgL, tri);
	
	/************************************************************************/
	/* Dibujo de la escena 3D mediante OpenGL                               */
	/************************************************************************/
	char *argv1 [1];
	int argc1 = 1;
	argv[0] = strdup("Reconstruccion");
	glutInit(&argc1, argv1); // Inicializa la librería GLUT
	InitGl(); //Inicializa las funciones de openGL
	
	cout << "Creando textura 3D ..." << endl;
	GLuint tex = Crear3DTextura(imgL, tri, ptsL, pts3D, center3D, size3D);
	ShowW(tex, center3D, size3D);
	
	std::cout<<"end"<<std::endl;
	
	return 0;
}

