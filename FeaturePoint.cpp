#include "header.h"
/*
* ALGORTIMO PARA BUSCAR PUNTOS CARACTERISTICOS DE LAS IMAGENES
*/
// SIFT (Scale-Invariant feature transform) es 50 veces más lento pero obtiene 7 veces más puntos igualados
// FAST (Features from accelerated segment test) detecta más puntos que SURF
// STAR/MSER generan pocos puntos característicos

#define DETECTOR_TYPE	"FAST" // FAST, SIFT, SURF, STAR, MSER, GFTT, HARRIS
#define DESCRIPTOR_TYPE	"SIFT" // SURF, SIFT, BRIEF 
#define MATCHER_TYPE	"FlannBased" // BruteForce, FlannBased, BruteForce-L1

#define MAXM_FILTER_TH	.8	// Umbral utilizado en GetPair
#define HOMO_FILTER_TH	60	// Umbral utilizado en GetPair
#define NEAR_FILTER_TH	40	// Los puntos diff deben tener una distancia superior a NEAR_FILTER_TH

// Elegir los puntos correspondientes en las imágenes estéreo para la reconstrucción 3D
void GetPair(Mat &imgL, Mat &imgR, vector<Point2f> &ptsL, vector<Point2f> &ptsR) 
{
	cv::initModule_nonfree();
	Mat descriptorsL, descriptorsR;
	double tt = (double) getTickCount();
	//realizamos la deteccion de puntos caracteristicos
	Ptr<FeatureDetector> detector = FeatureDetector::create(DETECTOR_TYPE); // Modo de fábrica
	vector<KeyPoint> keypointsL, keypointsR; 
	detector->detect(imgL, keypointsL);
	detector->detect(imgR, keypointsR);
	
	//obtenemos descriptores de los puntos caracteristicos
	Ptr<DescriptorExtractor> de = DescriptorExtractor::create(DESCRIPTOR_TYPE);
	//SurfDescriptorExtractor de(4,2,true);
	de->compute(imgL, keypointsL, descriptorsL);
	de->compute(imgR, keypointsR, descriptorsR);
	tt = ((double) getTickCount() - tt) / getTickFrequency(); // 620 * 555 pic, aproximadamente 2s para SURF, 120s para SIFT
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(MATCHER_TYPE);
	vector< vector<DMatch> > matches;
	matcher->knnMatch(descriptorsL, descriptorsR, matches, 2); // L: consulta, R: train
	vector<DMatch> passedMatches; // Guardar para dibujar
	DMatch m1, m2;
	vector<Point2f> ptsRtemp, ptsLtemp;
	// se optiene la distancia de los de ambas imagenes
	for(size_t i = 0; i < matches.size(); i++)
	{
		m1 = matches[i][0];
		m2 = matches[i][1];
		if (m1.distance < MAXM_FILTER_TH * m2.distance)
		{
			ptsRtemp.push_back(keypointsR[m1.trainIdx].pt);
			ptsLtemp.push_back(keypointsL[i].pt);
			passedMatches.push_back(m1);
		}
	}
	
	//hallamos los puntos similares en imagen derecha e izquierda
	Mat HLR;
	HLR = findHomography(Mat(ptsLtemp), Mat(ptsRtemp), CV_RANSAC, 3);
	cout << "Homografia:" << endl << HLR << endl;
	Mat ptsLt; 
	perspectiveTransform(Mat(ptsLtemp), ptsLt, HLR);
	
	vector<char> matchesMask(passedMatches.size(), 0);
	int cnt = 0;
	///mostramos las lineas de distancia de la imagen derecha e izquierda
	for(size_t i1 = 0; i1 < ptsLtemp.size(); i1++)
	{
		Point2f prjPtR = ptsLt.at<Point2f>((int)i1, 0); // prjx = ptsLt.at<float>((int)i1,0), prjy = ptsLt.at<float>((int)i1,1);
		
		if(abs(ptsRtemp[i1].x - prjPtR.x) < HOMO_FILTER_TH &&
		   abs(ptsRtemp[i1].y - prjPtR.y) < 2) // Restricción más estricta
		{
			vector<Point2f>::iterator iter = ptsL.begin();
			for (; iter != ptsL.end(); iter++)
			{
				Point2f diff = *iter - ptsLtemp[i1];
				float dist = abs(diff.x) + abs(diff.y);
				if (dist < NEAR_FILTER_TH) break;
			}
			if (iter != ptsL.end()) continue;
			
			ptsL.push_back(ptsLtemp[i1]);
			ptsR.push_back(ptsRtemp[i1]);
			cnt++;
			if (cnt%1 == 0) matchesMask[i1] = 1; // No quiero dibujar a muchas estancias
		}
	}
	
	///dibujamos ambas imagenes en una sola ventana
	Mat outImg;
	drawMatches(imgL, keypointsL, imgR, keypointsR, passedMatches, outImg, 
				Scalar::all(-1), Scalar::all(-1), matchesMask, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	char title[50];
	snprintf(title, 50, "%.3f s, %d matches, %d passed", tt, matches.size(), cnt);
	imshow(title, outImg);
	waitKey();
}
