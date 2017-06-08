#include "stdafx.h"
#include <vector>
#include <iostream>
#include "CImg.h"
#include <fstream>//ifstream、ofstream头文件 
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "windows.h" // for time statistics

using namespace cimg_library;
using namespace std;

/*************************分割线***********************/
//********************以下是三角剖分程序***************/
/******************************************************/

// DEFINES ////////////////////////////////////////////////
#define MAX_VERTEX_NUM 4092

#ifdef SINGLE
#define REAL float
#else
#define REAL double
#endif


// TYPES //////////////////////////////////////////////////
typedef struct VERTEX2D_TYP
{
	REAL x;
	REAL y;

} VERTEX2D, *VERTEX2D_PTR;

typedef struct EDGE_TYP
{
	VERTEX2D v1;
	VERTEX2D v2;

} EDGE, *EDGE_PTR;

typedef struct TRIANGLE_TYP
{
	int i1; // vertex index
	int i2;
	int i3;

	TRIANGLE_TYP* pNext;
	TRIANGLE_TYP* pPrev;

} TRIANGLE, *TRIANGLE_PTR;

typedef struct MESH_TYP
{
	int vertex_num;
	int triangle_num;

	VERTEX2D_PTR pVerArr; // point to outer vertices arrary
	TRIANGLE_PTR pTriArr; // point to outer triangles arrary

} MESH, *MESH_PTR;



// PROTOTYPES ///////////////////////////////////////////
// Delaunay triangulation functions
void InitMesh(MESH_PTR pMesh, int ver_num);
void UnInitMesh(MESH_PTR pMesh);

void AddBoundingBox(MESH_PTR pMesh);
void RemoveBoundingBox(MESH_PTR pMesh);;
void IncrementalDelaunay(MESH_PTR pMesh);

void Insert(MESH_PTR pMesh, int ver_index);
bool FlipTest(MESH_PTR pMesh, TRIANGLE_PTR pTestTri);

REAL InCircle(VERTEX2D_PTR pa, VERTEX2D_PTR pb, VERTEX2D_PTR pp, VERTEX2D_PTR  pd);
REAL InTriangle(MESH_PTR pMesh, VERTEX2D_PTR pVer, TRIANGLE_PTR pTri);

void InsertInTriangle(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index);
void InsertOnEdge(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index);

// Helper functions
void RemoveTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pTri);
TRIANGLE_PTR AddTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pPrevTri, int i1, int i2, int i3);

// I/O functions
void Input(char* pFile, MESH_PTR pMesh);
void Output(char* pFile, MESH_PTR pMesh);

// GLOBALS ////////////////////////////////////////////////



// The format of input file should be as follows:
// The First Line: amount of vertices (the amount of vertices/points needed to be triangulated)
// Other Lines: x y z (the vertices/points coordinates, z should be 0 for 2D)
// E.g. 
// 4
// -1 -1 0
// -1 1 0
// 1 1 0
// 1 -1 0
void Input(char* pFile, MESH_PTR pMesh)
{
	FILE *fp;
	//FILE* fp = fopen_s(&fp, pFile, "r");

	if (fopen_s(&fp, pFile, "r"))
	{
		fprintf(stderr, "Error:%s open failed\n", pFile);
		exit(1);
	}

	//int face;
	int amount;

	//fscanf( fp, "%d", &face);
	fscanf_s(fp, "%d", &amount);
	if (amount < 3)
	{
		fprintf(stderr, "Error:vertex amount should be greater than 2, but it is %d \n", amount);
		exit(1);
	}

	InitMesh(pMesh, amount);

	REAL x, y;
	for (int j = 3; j<amount + 3; ++j)
	{
		fscanf_s(fp, "%lg %lg", &x, &y);
		((VERTEX2D_PTR)(pMesh->pVerArr + j))->x = x;
		((VERTEX2D_PTR)(pMesh->pVerArr + j))->y = y;
	}


	fclose(fp);
}

// Algorithm IncrementalDelaunay(V)
// Input: 由n个点组成的二维点集V
// Output: Delaunay三角剖分DT
//	1.add a appropriate triangle boudingbox to contain V ( such as: we can use triangle abc, a=(0, 3M), b=(-3M,-3M), c=(3M, 0), M=Max({|x1|,|x2|,|x3|,...} U {|y1|,|y2|,|y3|,...}))
//	2.initialize DT(a,b,c) as triangle abc
//	3.for i <- 1 to n 
//		do (Insert(DT(a,b,c,v1,v2,...,vi-1), vi))   
//	4.remove the boundingbox and relative triangle which cotains any vertex of triangle abc from DT(a,b,c,v1,v2,...,vn) and return DT(v1,v2,...,vn).
void IncrementalDelaunay(MESH_PTR pMesh)
{
	// Add a appropriate triangle boudingbox to contain V
	AddBoundingBox(pMesh);

	// Get a vertex/point vi from V and Insert(vi)
	for (int i = 3; i<pMesh->vertex_num + 3; i++)
	{
		Insert(pMesh, i);
	}

	// Remove the bounding box
	RemoveBoundingBox(pMesh);
}

// The format of output file should be as follows:
// triangle index
// x1 y1 (the coordinate of first vertex of triangle)
// x2 y2 (the coordinate of second vertex of triangle)
// x3 y3 (the coordinate of third vertex of triangle)
void Output(char* pFile, MESH_PTR pMesh)
{
	FILE* fp;
	//FILE* fp = fopen(pFile, "w");
	if (fopen_s(&fp, pFile, "w"))
	{
		fprintf(stderr, "Error:%s open failed\n", pFile);

		UnInitMesh(pMesh);
		exit(1);
	}

	TRIANGLE_PTR pTri = pMesh->pTriArr;
	int* pi;
	int vertex_index;
	int tri_index = 0;
	while (pTri != NULL)
	{
		fprintf(fp, "Triangle: %d\n", ++tri_index);

		pi = &(pTri->i1);
		for (int j = 0; j<3; j++)
		{
			vertex_index = *pi++;
			fprintf(fp, "%lg %lg\n", ((VERTEX2D_PTR)(pMesh->pVerArr + vertex_index))->x, ((VERTEX2D_PTR)(pMesh->pVerArr + vertex_index))->y);
		}

		pTri = pTri->pNext;
	}

	fclose(fp);

	UnInitMesh(pMesh);
}


// Allocate memory to store vertices and triangles
void InitMesh(MESH_PTR pMesh, int ver_num)
{
	// Allocate memory for vertex array
	pMesh->pVerArr = (VERTEX2D_PTR)malloc((ver_num + 3)*sizeof(VERTEX2D));
	if (pMesh->pVerArr == NULL)
	{
		fprintf(stderr, "Error:Allocate memory for mesh failed\n");
		exit(1);
	}

	pMesh->vertex_num = ver_num;

}

// Deallocate memory
void UnInitMesh(MESH_PTR pMesh)
{
	// free vertices
	if (pMesh->pVerArr != NULL)
		free(pMesh->pVerArr);

	// free triangles
	TRIANGLE_PTR pTri = pMesh->pTriArr;
	TRIANGLE_PTR pTemp = NULL;
	while (pTri != NULL)
	{
		pTemp = pTri->pNext;
		free(pTri);
		pTri = pTemp;
	}
}

void AddBoundingBox(MESH_PTR pMesh)
{
	REAL max = 0;
	REAL max_x = 0;
	REAL max_y = 0;
	REAL t;

	for (int i = 3; i<pMesh->vertex_num + 3; i++)
	{
		t = fabs(((VERTEX2D_PTR)(pMesh->pVerArr + i))->x);
		if (max_x < t)
		{
			max_x = t;
		}

		t = fabs(((VERTEX2D_PTR)(pMesh->pVerArr + i))->y);
		if (max_y < t)
		{
			max_y = t;
		}
	}

	max = max_x > max_y ? max_x : max_y;

	//TRIANGLE box;
	//box.v1 = VERTEX2D(0, 3*max);
	//box.v2 = VERTEX2D(-3*max, 3*max);
	//box.v3 = VERTEX2D(3*max, 0);

	VERTEX2D v1 = { 0, 4 * max };
	VERTEX2D v2 = { -4 * max, -4 * max };
	VERTEX2D v3 = { 4 * max, 0 };

	// Assign to Vertex array
	*(pMesh->pVerArr) = v1;
	*(pMesh->pVerArr + 1) = v2;
	*(pMesh->pVerArr + 2) = v3;

	// add the Triangle boundingbox
	AddTriangleNode(pMesh, NULL, 0, 1, 2);
}

void RemoveBoundingBox(MESH_PTR pMesh)
{
	int statify[3] = { 0,0,0 };
	int vertex_index;
	int* pi;
	int k = 1;

	// Remove the first triangle-boundingbox
	//pMesh->pTriArr = pMesh->pTriArr->pNext;
	//pMesh->pTriArr->pPrev = NULL; // as head

	TRIANGLE_PTR pTri = pMesh->pTriArr;
	TRIANGLE_PTR pNext = NULL;
	while (pTri != NULL)
	{
		pNext = pTri->pNext;

		statify[0] = 0;
		statify[1] = 0;
		statify[2] = 0;

		pi = &(pTri->i1);
		for (int j = 0, k = 1; j<3; j++, k *= 2)
		{
			vertex_index = *pi++;

			if (vertex_index == 0 || vertex_index == 1 || vertex_index == 2) // bounding box vertex
			{
				statify[j] = k;
			}
		}

		switch (statify[0] | statify[1] | statify[2])
		{
		case 0: // no statify
			break;
		case 1:
		case 2:
		case 4: // 1 statify, remove 1 triangle, 1 vertex
			RemoveTriangleNode(pMesh, pTri);
			break;
		case 3:
		case 5:
		case 6: // 2 statify, remove 1 triangle, 2 vertices
			RemoveTriangleNode(pMesh, pTri);
			break;
		case 7: // 3 statify, remove 1 triangle, 3 vertices
			RemoveTriangleNode(pMesh, pTri);
			break;
		default:
			break;
		}

		// go to next item
		pTri = pNext;
	}
}


// Return a positive value if the points pa, pb, and
// pc occur in counterclockwise order; a negative
// value if they occur in clockwise order; and zero
// if they are collinear. The result is also a rough
// approximation of twice the signed area of the
// triangle defined by the three points.
REAL CounterClockWise(VERTEX2D_PTR pa, VERTEX2D_PTR pb, VERTEX2D_PTR pc)
{
	return ((pb->x - pa->x)*(pc->y - pb->y) - (pc->x - pb->x)*(pb->y - pa->y));
}

// Adjust if the point lies in the triangle abc
REAL InTriangle(MESH_PTR pMesh, VERTEX2D_PTR pVer, TRIANGLE_PTR pTri)
{
	int vertex_index;
	VERTEX2D_PTR pV1, pV2, pV3;

	vertex_index = pTri->i1;
	pV1 = (VERTEX2D_PTR)(pMesh->pVerArr + vertex_index);
	vertex_index = pTri->i2;
	pV2 = (VERTEX2D_PTR)(pMesh->pVerArr + vertex_index);
	vertex_index = pTri->i3;
	pV3 = (VERTEX2D_PTR)(pMesh->pVerArr + vertex_index);

	REAL ccw1 = CounterClockWise(pV1, pV2, pVer);
	REAL ccw2 = CounterClockWise(pV2, pV3, pVer);
	REAL ccw3 = CounterClockWise(pV3, pV1, pVer);

	REAL r = -1;
	if (ccw1>0 && ccw2>0 && ccw3>0)
	{
		r = 1;
	}
	else if (ccw1*ccw2*ccw3 == 0 && (ccw1*ccw2 > 0 || ccw1*ccw3 > 0 || ccw2*ccw3 > 0))
	{
		r = 0;
	}

	return r;
}

// Algorithm Insert(DT(a,b,c,v1,v2,...,vi-1), vi)
// 1.find the triangle vavbvc which contains vi // FindTriangle()
// 2.if (vi located at the interior of vavbvc)  
// 3.    then add triangle vavbvi, vbvcvi and vcvavi into DT // UpdateDT()
// FlipTest(DT, va, vb, vi)
// FlipTest(DT, vb, vc, vi)
// FlipTest(DT, vc, va, vi)
// 4.else if (vi located at one edge (E.g. edge vavb) of vavbvc) 
// 5.    then add triangle vavivc, vivbvc, vavdvi and vivdvb into DT (here, d is the third vertex of triangle which contains edge vavb) // UpdateDT()
// FlipTest(DT, va, vd, vi)
// FlipTest(DT, vc, va, vi)
// FlipTest(DT, vd, vb, vi)
// FlipTest(DT, vb, vc, vi)
// 6.return DT(a,b,c,v1,v2,...,vi)
void Insert(MESH_PTR pMesh, int ver_index)
{
	VERTEX2D_PTR pVer = (VERTEX2D_PTR)(pMesh->pVerArr + ver_index);
	TRIANGLE_PTR pTargetTri = NULL;
	TRIANGLE_PTR pEqualTri1 = NULL;
	TRIANGLE_PTR pEqualTri2 = NULL;

	int j = 0;
	TRIANGLE_PTR pTri = pMesh->pTriArr;
	while (pTri != NULL)
	{
		REAL r = InTriangle(pMesh, pVer, pTri);
		if (r > 0) // should be in triangle
		{
			pTargetTri = pTri;
		}
		else if (r == 0) // should be on edge
		{
			if (j == 0)
			{
				pEqualTri1 = pTri;
				j++;
			}
			else
			{
				pEqualTri2 = pTri;
			}

		}

		pTri = pTri->pNext;
	}

	if (pEqualTri1 != NULL && pEqualTri2 != NULL)
	{
		InsertOnEdge(pMesh, pEqualTri1, ver_index);
		InsertOnEdge(pMesh, pEqualTri2, ver_index);
	}
	else
	{
		InsertInTriangle(pMesh, pTargetTri, ver_index);
	}
}

void InsertInTriangle(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index)
{
	int index_a, index_b, index_c;
	TRIANGLE_PTR pTri = NULL;
	TRIANGLE_PTR pNewTri = NULL;

	pTri = pTargetTri;
	if (pTri == NULL)
	{
		return;
	}

	// Inset p into target triangle
	index_a = pTri->i1;
	index_b = pTri->i2;
	index_c = pTri->i3;

	// Insert edge pa, pb, pc
	for (int i = 0; i<3; i++)
	{
		// allocate memory
		if (i == 0)
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_a, index_b, ver_index);
		}
		else if (i == 1)
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_b, index_c, ver_index);
		}
		else
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_c, index_a, ver_index);
		}

		// go to next item
		if (pNewTri != NULL)
		{
			pTri = pNewTri;
		}
		else
		{
			pTri = pTri;
		}
	}

	// Get the three sub-triangles
	pTri = pTargetTri;
	TRIANGLE_PTR pTestTri[3];
	for (int i = 0; i< 3; i++)
	{
		pTestTri[i] = pTri->pNext;

		pTri = pTri->pNext;
	}

	// remove the Target Triangle
	RemoveTriangleNode(pMesh, pTargetTri);

	for (int i = 0; i< 3; i++)
	{
		// Flip test
		FlipTest(pMesh, pTestTri[i]);
	}
}

void InsertOnEdge(MESH_PTR pMesh, TRIANGLE_PTR pTargetTri, int ver_index)
{
	int index_a, index_b, index_c;
	TRIANGLE_PTR pTri = NULL;
	TRIANGLE_PTR pNewTri = NULL;

	pTri = pTargetTri;
	if (pTri == NULL)
	{
		return;
	}

	// Inset p into target triangle
	index_a = pTri->i1;
	index_b = pTri->i2;
	index_c = pTri->i3;

	// Insert edge pa, pb, pc
	for (int i = 0; i<3; i++)
	{
		// allocate memory
		if (i == 0)
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_a, index_b, ver_index);
		}
		else if (i == 1)
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_b, index_c, ver_index);
		}
		else
		{
			pNewTri = AddTriangleNode(pMesh, pTri, index_c, index_a, ver_index);
		}

		// go to next item
		if (pNewTri != NULL)
		{
			pTri = pNewTri;
		}
		else
		{
			pTri = pTri;
		}
	}

	// Get the two sub-triangles
	pTri = pTargetTri;
	TRIANGLE_PTR pTestTri[2];
	for (int i = 0; i< 2; i++)
	{
		pTestTri[i] = pTri->pNext;
		pTri = pTri->pNext;
	}

	// remove the Target Triangle
	RemoveTriangleNode(pMesh, pTargetTri);

	for (int i = 0; i< 2; i++)
	{
		// Flip test
		FlipTest(pMesh, pTestTri[i]);
	}
}

// Precondition: the triangle satisfies CCW order
// Algorithm FlipTest(DT(a,b,c,v1,v2,...,vi), va, vb, vi)
// 1.find the third vertex (vd) of triangle which contains edge vavb // FindThirdVertex()
// 2.if(vi is in circumcircle of abd)  // InCircle()
// 3.    then remove edge vavb, add new edge vivd into DT // UpdateDT()
//		  FlipTest(DT, va, vd, vi)
//		  FlipTest(DT, vd, vb, vi)

bool FlipTest(MESH_PTR pMesh, TRIANGLE_PTR pTestTri)
{
	bool flipped = false;

	int index_a = pTestTri->i1;
	int index_b = pTestTri->i2;
	int index_p = pTestTri->i3;

	int statify[3] = { 0,0,0 };
	int vertex_index;
	int* pi;
	int k = 1;

	// find the triangle which has edge consists of start and end
	TRIANGLE_PTR pTri = pMesh->pTriArr;

	int index_d = -1;
	while (pTri != NULL)
	{
		statify[0] = 0;
		statify[1] = 0;
		statify[2] = 0;

		pi = &(pTri->i1);
		for (int j = 0, k = 1; j<3; j++, k *= 2)
		{
			vertex_index = *pi++;
			if (vertex_index == index_a || vertex_index == index_b)
			{
				statify[j] = k;
			}
		}

		switch (statify[0] | statify[1] | statify[2])
		{
		case 3:
			if (CounterClockWise((VERTEX2D_PTR)(pMesh->pVerArr + index_a), (VERTEX2D_PTR)(pMesh->pVerArr + index_b), (VERTEX2D_PTR)(pMesh->pVerArr + pTri->i3)) < 0)
			{
				index_d = pTri->i3;
			}

			break;
		case 5:
			if (CounterClockWise((VERTEX2D_PTR)(pMesh->pVerArr + index_a), (VERTEX2D_PTR)(pMesh->pVerArr + index_b), (VERTEX2D_PTR)(pMesh->pVerArr + pTri->i2)) < 0)
			{
				index_d = pTri->i2;
			}

			break;
		case 6:
			if (CounterClockWise((VERTEX2D_PTR)(pMesh->pVerArr + index_a), (VERTEX2D_PTR)(pMesh->pVerArr + index_b), (VERTEX2D_PTR)(pMesh->pVerArr + pTri->i1)) < 0)
			{
				index_d = pTri->i1;
			}

			break;

		default:
			break;
		}

		if (index_d != -1)
		{
			VERTEX2D_PTR pa = (VERTEX2D_PTR)(pMesh->pVerArr + index_a);
			VERTEX2D_PTR pb = (VERTEX2D_PTR)(pMesh->pVerArr + index_b);
			VERTEX2D_PTR pd = (VERTEX2D_PTR)(pMesh->pVerArr + index_d);
			VERTEX2D_PTR pp = (VERTEX2D_PTR)(pMesh->pVerArr + index_p);

			if (InCircle(pa, pb, pp, pd) < 0) // not local Delaunay
			{
				flipped = true;

				// add new triangle adp,  dbp, remove abp, abd.
				// allocate memory for adp
				TRIANGLE_PTR pT1 = AddTriangleNode(pMesh, pTestTri, pTestTri->i1, index_d, pTestTri->i3);
				// allocate memory for dbp
				TRIANGLE_PTR pT2 = AddTriangleNode(pMesh, pT1, index_d, pTestTri->i2, index_p);
				// remove abp
				RemoveTriangleNode(pMesh, pTestTri);
				// remove abd
				RemoveTriangleNode(pMesh, pTri);

				FlipTest(pMesh, pT1); // pNewTestTri satisfies CCW order
				FlipTest(pMesh, pT2); // pNewTestTri2  satisfies CCW order

				break;
			}
		}

		// go to next item	
		pTri = pTri->pNext;
	}

	return flipped;
}

// In circle test, use vector cross product
REAL InCircle(VERTEX2D_PTR pa, VERTEX2D_PTR pb, VERTEX2D_PTR pp, VERTEX2D_PTR  pd)
{
	REAL det;
	REAL alift, blift, plift, bdxpdy, pdxbdy, pdxady, adxpdy, adxbdy, bdxady;

	REAL adx = pa->x - pd->x;
	REAL ady = pa->y - pd->y;

	REAL bdx = pb->x - pd->x;
	REAL bdy = pb->y - pd->y;

	REAL pdx = pp->x - pd->x;
	REAL pdy = pp->y - pd->y;

	bdxpdy = bdx * pdy;
	pdxbdy = pdx * bdy;
	alift = adx * adx + ady * ady;

	pdxady = pdx * ady;
	adxpdy = adx * pdy;
	blift = bdx * bdx + bdy * bdy;

	adxbdy = adx * bdy;
	bdxady = bdx * ady;
	plift = pdx * pdx + pdy * pdy;

	det = alift * (bdxpdy - pdxbdy)
		+ blift * (pdxady - adxpdy)
		+ plift * (adxbdy - bdxady);

	return -det;
}

// Remove a node from the triangle list and deallocate the memory
void RemoveTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pTri)
{
	if (pTri == NULL)
	{
		return;
	}

	// remove from the triangle list
	if (pTri->pPrev != NULL)
	{
		pTri->pPrev->pNext = pTri->pNext;
	}
	else // remove the head, need to reset the root node
	{
		pMesh->pTriArr = pTri->pNext;
	}

	if (pTri->pNext != NULL)
	{
		pTri->pNext->pPrev = pTri->pPrev;
	}

	// deallocate memory
	free(pTri);
}

// Create a new node and add it into triangle list
TRIANGLE_PTR AddTriangleNode(MESH_PTR pMesh, TRIANGLE_PTR pPrevTri, int i1, int i2, int i3)
{
	// test if 3 vertices are co-linear
	if (CounterClockWise((VERTEX2D_PTR)(pMesh->pVerArr + i1), (VERTEX2D_PTR)(pMesh->pVerArr + i2), (VERTEX2D_PTR)(pMesh->pVerArr + i3)) == 0)
	{
		return NULL;
	}

	// allocate memory
	TRIANGLE_PTR pNewTestTri = (TRIANGLE_PTR)malloc(sizeof(TRIANGLE));

	pNewTestTri->i1 = i1;
	pNewTestTri->i2 = i2;
	pNewTestTri->i3 = i3;

	// insert after prev triangle
	if (pPrevTri == NULL) // add root
	{
		pMesh->pTriArr = pNewTestTri;
		pNewTestTri->pNext = NULL;
		pNewTestTri->pPrev = NULL;
	}
	else
	{
		pNewTestTri->pNext = pPrevTri->pNext;
		pNewTestTri->pPrev = pPrevTri;

		if (pPrevTri->pNext != NULL)
		{
			pPrevTri->pNext->pPrev = pNewTestTri;
		}

		pPrevTri->pNext = pNewTestTri;
	}

	return pNewTestTri;
}

/************************分割线**********************************/
//*********************三角剖分程序完毕**************************/
/****************************************************************/

int _tmain(int argc, _TCHAR* argv[]) {
	//读入图片
	CImg<unsigned char> src;
	CImg<unsigned char> tar;
	src.load_bmp("data/3.bmp");
	tar.load_bmp("data/4.bmp");
	int w = src._width;
	int h = src._height;

	//读取特征点
	int srcT[50][2], tarT[50][2];//1-45 valid
	int num, i = 0, j = 0;
	fstream srcfile;
	srcfile.open("data/3.txt");
	if (srcfile.is_open()) { //打开成功 
		while (!srcfile.eof()) { //文件状态是否到达末尾 
			if (i == 0) {
				srcfile >> num;
				i++;
			}
			else {
				srcfile >> srcT[i][0] >> srcT[i][1];
				i++;
			}	
		}
	}
	srcfile.close();

	fstream tarfile;
	tarfile.open("data/4.txt");
	if (tarfile.is_open()) { //打开成功 
		while (!tarfile.eof()) { //文件状态是否到达末尾 
			if (j == 0) {
				tarfile >> num;
				j++;
			}
			else {
				tarfile >> tarT[j][0] >> tarT[j][1];
				j++;
			}
		}
	}
	tarfile.close();


	int medT[50][2];//中间图特征点
	for (int rate = 1; rate <= 11; rate++) {
		fstream in;
		in.open("data/medT2.txt");//存储中间特征点图
		in << num << endl;
		//按比例求中间图特征点的位置
		for (i = 1; i <= num; i++) {
			medT[i][0] = (double)rate / 12 * tarT[i][0] + (double)(12 - rate) / 12 * srcT[i][0];
			medT[i][1] = (double)rate / 12 * tarT[i][1] + (double)(12 - rate) / 12 * srcT[i][1];
			in << round(medT[i][0]) << " " << round(medT[i][1]) << endl;
		}
		in.close();

		//对中间特征点进行三角分割
		MESH medmesh;
		Input("data/medT2.txt", &medmesh);
		IncrementalDelaunay(&medmesh);
		//Output("data/med_out.txt", &medmesh);

		CImg<unsigned char> medimg = CImg<unsigned char>(w, h, 1, 3);//中间图
		cimg_forXY(medimg, x, y) {
			//扫描剖分的三角形
			TRIANGLE_PTR medp = medmesh.pTriArr;//中间图三角形数组

			while (medp != NULL)
			{
				VERTEX2D v;
				v.x = x;
				v.y = y;
				//在这个三角形内部
				if (InTriangle(&medmesh, &v, medp) != -1) {
					//获取三角形的三个顶点
					
					VERTEX2D_PTR mv1, mv2, mv3;//中间三角形的三个顶点
					mv1 = (VERTEX2D_PTR)(medmesh.pVerArr + medp->i1);
					mv2 = (VERTEX2D_PTR)(medmesh.pVerArr + medp->i2);
					mv3 = (VERTEX2D_PTR)(medmesh.pVerArr + medp->i3);

					//找出对应于原三角形和目标三角形的下标
					int u1, u2, u3;
					for (int k = 1; k <= num; k++) {
						if (medT[k][0] == mv1->x && medT[k][1] == mv1->y) u1 = k;
						if (medT[k][0] == mv2->x && medT[k][1] == mv2->y) u2 = k;
						if (medT[k][0] == mv3->x && medT[k][1] == mv3->y) u3 = k;
					}


					//矩阵运算
					//m0表示中间图
					CImg<float> m0(3, 3);
					m0(0, 0) = mv1->x;
					m0(1, 0) = mv2->x;
					m0(2, 0) = mv3->x;
					m0(0, 1) = mv1->y;
					m0(1, 1) = mv2->y;
					m0(2, 1) = mv3->y;
					m0(0, 2) = m0(1, 2) = m0(2, 2) = 1;

					CImg<float> m0_inv = m0.pseudoinvert();
					
					//m1表示原图
					CImg<float> m1(3, 3);
					m1(0, 0) = srcT[u1][0];
					m1(1, 0) = srcT[u2][0];
					m1(2, 0) = srcT[u3][0];
					m1(0, 1) = srcT[u1][1];
					m1(1, 1) = srcT[u2][1];
					m1(2, 1) = srcT[u3][1];
					m1(0, 2) = m1(1, 2) = m1(2, 2) = 1;

					//cur是当前坐标
					CImg<float> cur(1, 3);
					cur(0, 0) = x;
					cur(1, 0) = y;
					cur(2, 0) = 1;
					CImg<float> pos1(1, 3);
					pos1 = m1*m0_inv*cur;

					//m2表示目标图
					CImg<float> m2(3, 3);

					m2(0, 0) = tarT[u1][0];
					m2(1, 0) = tarT[u2][0];
					m2(2, 0) = tarT[u3][0];
					m2(0, 1) = tarT[u1][1];
					m2(1, 1) = tarT[u2][1];
					m2(2, 1) = tarT[u3][1];
					m2(0, 2) = m2(1, 2) = m2(2, 2) = 1;
					CImg<float> pos2(1, 3);
					pos2 = m2*m0_inv*cur;

					//加权求和作为中间图
					medimg(x, y, 0) = round((double)(12 - rate) / 12 * src(round(pos1(0, 0)), round(pos1(0, 1)), 0) + (double)rate / 12 * tar(round(pos2(0, 0)), round(pos2(0, 1)), 0));
					medimg(x, y, 1) = round((double)(12 - rate) / 12 * src(round(pos1(0, 0)), round(pos1(0, 1)), 1) + (double)rate/ 12 * tar(round(pos2(0, 0)), round(pos2(0, 1)), 1));
					medimg(x, y, 2) = round((double)(12 - rate) / 12 * src(round(pos1(0, 0)), round(pos1(0, 1)), 2) + (double)rate/ 12 * tar(round(pos2(0, 0)), round(pos2(0, 1)), 2));
					break;
				}
				medp = medp->pNext;
			}
		}
		//medimg.display();
		if (rate == 1) medimg.save("result2/1.bmp");
		if (rate == 2) medimg.save("result2/2.bmp");
		if (rate == 3) medimg.save("result2/3.bmp");
		if (rate == 4) medimg.save("result2/4.bmp");
		if (rate == 5) medimg.save("result2/5.bmp");
		if (rate == 6) medimg.save("result2/6.bmp");
		if (rate == 7) medimg.save("result2/7.bmp");
		if (rate == 8) medimg.save("result2/8.bmp");
		if (rate == 9) medimg.save("result2/9.bmp");
		if (rate == 10) medimg.save("result2/10.bmp");
		if (rate == 11) medimg.save("result2/11.bmp");
	}
	

	return 0;
}