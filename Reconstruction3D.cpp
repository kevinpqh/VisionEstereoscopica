#include <time.h>
#include "header.h"
//ALGORITMO DE BOWYER WATSON N log N
/*
	1-establecemos malla triangular inicial ,
		encontramos n triangulo R que contenga el conjutno de putnos V, donde R es ventana auxiliar y conectamos cualquier diagonal de R como triangulo incial de DELAUNAY
	2-suponiendo que encontramos una malla triangular T insertamos un punto P 
		-buscamos el triangulo adyacente
		-hacemos deteccion por circunferencia circunscrita
		-encotrnamos todos los circulos que contienena P y eliminamos los triangulos ARISTA mutua, para formar CAVIDAD POLIGONAL DE DELAUNAY
		-luego conectamos cada vertice a P y formamos una nueva malla triangular de DELAUNAY
*/
/*
	Triangulación de Delaunay para el MAPEO DE TEXTURAS
*/
void TriSubDiv(vector<Point2f> &pts, Mat &img, vector<Vec3i> &tri) //recibe ek vector de puntos caracteristicos, la matriz con la imagen, y como resultado nos otorga los triangulos
{
	CvSubdiv2D* subdiv; // Subdivision PARTICION DUAL es el GRAFICO DE VORONOI, del conjunto de puntos 2D de entrada, para tealizar seementacion 
	CvMemStorage* storage = cvCreateMemStorage(0);// Almacenamiento para los puntos obtenidos por la subdivision de Delaunay
	Rect rc = Rect(0, 0, img.cols, img.rows);//Cuadro delimitador externo, rectangulo es igual al tamaño de la imagen
	subdiv = cvCreateSubdiv2D(CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv),sizeof(CvSubdiv2DPoint),sizeof(CvQuadEdge2D),storage);//creamos la subdivision en si
	
	cvInitSubdivDelaunay2D(subdiv, rc);//se establece los limites del rectangulo virtual
	/*	
		Recorremos la matriz de puntos
	*/
	for (size_t i = 0; i < pts.size(); i++)
	{
		CvSubdiv2DPoint *pt = cvSubdivDelaunay2DInsert(subdiv, pts[i]);//insertamos los puntos en la sudbision, usando la funcion de Delaunay de openCV
		pt->id = i;
	}
	/*
		ESTABLECEMOS UN LECTOR DE SECUENCIA PARA ANALIZAR LAS ARISTAS DE LOS TRIANGULOS
	*/
	CvSeqReader reader;//estructura para recorrido del todos los segementos de delaunay
	int total = subdiv->edges->total;//numero total de bordes
	int elem_size = subdiv->edges->elem_size;//tamaño del borde
	cvStartReadSeq((CvSeq*)(subdiv->edges), &reader, 0);//iniciamos la lectura
	Point buf[3];
	const Point *pBuf = buf;
	Vec3i verticesIdx;
	Mat imgShow = img.clone();//clonamos la matriz de la imagen
	srand((unsigned)time(NULL));
	/*
		RECORREMOS TODOS LOS TRIANGULOS DE LA TRIANGULIZACION INICIAL
	*/
	for(int i = 0; i < total; i++)
	{   
		CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);//CAST del READERPOINTER en una arista de cuadrado
		if(CV_IS_SET_ELEM(edge)) //retorna TRUE si el nodo especificado esta presente
		{
			CvSubdiv2DEdge t = (CvSubdiv2DEdge)edge; 
			int iPointNum = 3;//puntos de lectura del triangulo
			Scalar color = CV_RGB(rand()&255, rand()&255, rand()&255);//color alaetorio para las aristas del triangulo
			
			int j;
			/*
				INICIAMOS RECONOCIMIENTO DEL BORDE
			*/
			for(j = 0; j < iPointNum; j++ )
			{
				CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg(t);//obtenemos la fuente de la arista t
				if(!pt) break;//SI NO SE ENCUENTRA VERIFICA LA OPUESTA
				buf[j] = pt->pt;//almacenamos el punto
				verticesIdx[j] = pt->id;// Obtener el número de identificación del vértice, almacenamos el id de los tres puntos en los verticesIdx  
				t = cvSubdiv2DGetEdge(t, CV_NEXT_AROUND_LEFT);//area izquierda siguiente proxima ventana
			}
			if (j != iPointNum)//si se encuentra la arista continuamos
				continue;
			if (isGoodTri(verticesIdx, tri))//verificamos que el trignulo no se vuelva a dibujar
			{
				polylines(imgShow, &pBuf, &iPointNum, 1, true, color,1, CV_AA, 0);//dibuja los tres bordes
			}
			/*
				INICIAMOS RECONOCIMIENTO DEL BORDE OPUESTO
			*/
			t = (CvSubdiv2DEdge)edge + 2;//realiza la misma operacion con el borde opuesto
			
			for(j = 0; j < iPointNum; j++)
			{
				CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg(t);
				if(!pt) break;
				buf[j] = pt->pt;
				verticesIdx[j] = pt->id;
				t = cvSubdiv2DGetEdge(t, CV_NEXT_AROUND_LEFT);
			}   
			if (j != iPointNum)
				continue;
			if (isGoodTri(verticesIdx, tri))
			{
				polylines( imgShow, &pBuf, &iPointNum, 1, true, color,1, CV_AA, 0);//dibuja los tres bordes
			}
		}
		CV_NEXT_SEQ_ELEM(elem_size, reader);
	}
	
	waitKey();
}

/*
	CALCULO DE COORDENADAS 3D
*/
void StereoTo3D( vector<Point2f> ptsL, vector<Point2f> ptsR, vector<Point3f> &pts3D, float focalLenInPixel, float baselineInMM, Mat img, Point3f &center3D, Vec3f &size3D)
{
	/*
		USAMOS GEOMETRIA EQUIPOLAR PARA EL CALCULO DE COORDENADAS 3D
	*/
	vector<Point2f>::iterator iterL = ptsL.begin(), iterR = ptsR.begin();//iterador inicialmente vacio
	
	float xl, xr, ylr;//x imagen izquierda, z imagen derecha, y promedio
	float imgH = float(img.rows), imgW = float(img.cols);
	Point3f pt3D;
	/*
		Establecimiento de maximos y minimos de la imagen
	*/
	float minX = 1e9, maxX = -1e9;
	float minY = 1e9, maxY = -1e9;
	float minZ = 1e9, maxZ = -1e9;
	
	Mat imgShow = img.clone();//duplicmos la matriz de la imagen para trabajar sobre ella
	char str[100];
	int ptCnt = ptsL.size(), showPtNum = 30, cnt = 0;//contador de puntos
	int showIntv = max(ptCnt / showPtNum, 1);//muestra de intervalos
	
	for (; iterL != ptsL.end(); iterL++, iterR++)
	{
		xl = iterL->x;
		xr = iterR->x;
		ylr = (iterL->y + iterR->y) / 2;
		
		/*
			Formula para obtener X, Y, Z, despeje de la formula de DISPARIDAD
		*/
		pt3D.z = -focalLenInPixel * baselineInMM / (xl - xr);
		pt3D.y = -(-ylr + imgH / 2) * pt3D.z / focalLenInPixel;
		pt3D.x = (imgW / 2 - xl) * pt3D.z / focalLenInPixel;
		
		/*
			SETEO DE LIMITES DE LA IMAGEN TRIDIMENSIONAL
		*/
		minX = min(minX, pt3D.x);
		maxX = max(maxX, pt3D.x);
		minY = min(minY, pt3D.y);
		maxY = max(maxY, pt3D.y);
		minZ = min(minZ, pt3D.z);
		maxZ = max(maxZ, pt3D.z);
		
		/*
			Almacena el punto tridimensional obtenido		
		*/
		pts3D.push_back(pt3D);
	}
	
	waitKey();
	
	/*
		CALCULO DEL CENTRO DE LA IMAGEN TRIDIMENSIONAL
	*/
	center3D.x = (minX + maxX) / 2;
	center3D.y = (minY + maxY) / 2;
	center3D.z = (minZ + maxZ) / 2;
	
	/*
		CALCULO DEL TAMAÑO DE LA IMAGEN TRIDIMENSIONAL
	*/
	size3D[0] = maxX - minX;
	size3D[1] = maxY - minY;
	size3D[2] = maxZ - minZ;
}

/*
	Funcion que evita el redibujado y eliminan los vertices del triangulo virtual
*/
bool isGoodTri(Vec3i &v, vector<Vec3i> & tri)
{
	int a = v[0], b = v[1], c = v[2];
	v[0] = min(a, min(b,c));//para encontrar el punto final de insercion del triangulo
	v[2] = max(a, max(b,c));//almacena el valor maximo, para formar el angulo minimo mas pequeño
	v[1] = a + b + c - v[0] - v[2];//un valor itnermedio
	if (v[0] == -1) return false;
	
	vector<Vec3i>::iterator iter = tri.begin();//incialmente vacio
	for(; iter != tri.end(); iter++)
	{
		Vec3i &check = *iter;//si la liena se va alamcenar deja de hacer BREAK y al isnerta
		if (check[0]==v[0] &&check[1]==v[1] &&check[2]==v[2])break;//verificamos que no se encuentre pintada la arista del triangulo
	}
	if (iter == tri.end())
	{
		tri.push_back(v);//almacenamos vertice del triangulo
		return true;
	}
	return false;
}
