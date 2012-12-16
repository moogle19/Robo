#include "mini.h"
#include <vector>
#include "newmat.h"
#include "newmatap.h"
#include "newmatio.h"
#include "math.h"

using std::vector;

double dist(const double* p1, const double* p2){
	double result = p1[0]*p2[0] + p1[1]*p2[1] + p1[2]*p2[2];
	return sqrt(result);
};

double Point_Point_Align(const vector<PtPair>& pairs, double *alignfx){
	double error = 0;
		
	// Compute centroids
	double cm[3] = {0,0,0};
	double cd[3] = {0,0,0};
	// compute c_d and c_m
	for(int i=0; i < pairs.size(); i++)
	{
		cm[0] += pairs[i].p1[0];
		cm[1] += pairs[i].p1[1];
		cm[2] += pairs[i].p1[2];
		cd[0] += pairs[i].p2[0];
		cd[1] += pairs[i].p2[1];
		cd[2] += pairs[i].p2[2];
	}
	cm[0] /= pairs.size();
	cm[1] /= pairs.size();
	cm[2] /= pairs.size();
	cd[0] /= pairs.size();
	cd[1] /= pairs.size();
	cd[2] /= pairs.size();


	// Get centered PtPairs
	double m[pairs.size()][3];
	double d[pairs.size()][3];
	for(int i=0; i < pairs.size(); i++){
		m[i][0] = pairs[i].p1[0] - cm[0];
		m[i][1] = pairs[i].p1[1] - cm[1];
		m[i][2] = pairs[i].p1[2] - cm[2];
		d[i][0] = pairs[i].p2[0] - cd[0];
		d[i][1] = pairs[i].p2[1] - cd[1];
		d[i][2] = pairs[i].p2[2] - cd[2];
	}

	// Fill H matrix
	Matrix H(3,3), R(3,3);
	// Fill the H matrix (formula 2)
	for(int k=1; k <= 3; k++)
	{
    		for(int l=1; l <= 3; l++)
		{
      			H(l,k)=0;
    			for(int i=0; i < pairs.size(); i++)
				{
      				H(l,k)+= m[i][k-1]*d[i][l-1];
    			}
    		}
	}

	Matrix U(3,3);
	DiagonalMatrix Lambda(3);
	Matrix V(3,3);

	// Make SVD
	// use the SVD function provided in the netmat lib (with its strange syntax!)
	// calculate Lambda, U, V (formula: H = U Lambda V^T)
	// void SVD(const Matrix& A, DiagonalMatrix& Q, Matrix& U, Matrix& V, bool withU, bool withV)
	SVD(H, Lambda, U, V, true, true);
	// Get rotation
	// calculate R (formula: R = VU^T)
	R = V * U.t();

	//output of matrix via printf
	// printf("R: \n");
	// printf("%lf %lf %lf \n", R(1,1), R(1,2), R(1,3));
	// printf("%lf %lf %lf \n", R(2,1), R(2,2), R(2,3));
	// printf("%lf %lf %lf \n", R(3,1), R(3,2), R(3,3));
	// or via streams
	// std::cout << R << std::endl

	// Calculate translation
	double translation[3];
	// calculate translation (formula 3)
	translation[0]= cm[0] - (R(1,1)*cd[0] + R(1,2)*cd[1] + R(1,3)*cd[2]);
	translation[1]= cm[1] - (R(2,1)*cd[0] + R(2,2)*cd[1] + R(2,3)*cd[2]);
	translation[2]= cm[2] - (R(3,1)*cd[0] + R(3,2)*cd[1] + R(3,3)*cd[2]);
	
	// Fill result in OpenGL style matrix;
	// its a 4x4 matrix conatining rotation and translation
	alignfx[0] = R(1,1);
	alignfx[1] = R(2,1);
	alignfx[2] = 0;
	alignfx[2] = R(3,1);
	alignfx[3] = 0;
	alignfx[4] = R(1,2);
	alignfx[5] = R(2,2);
	alignfx[6] = R(3,2);
	alignfx[7] = 0;
	alignfx[8] = R(1,3);
	alignfx[9] = R(2,3);
	alignfx[10] = R(3,3);
	alignfx[11] = 0;
	alignfx[12] = translation[0];
	alignfx[13] = translation[1];
	alignfx[14] = translation[2];
	alignfx[15] = 1;

	// compute error (formula 1)
	double temp[3];
	error=0;
	for(int i=0; i <  pairs.size(); i++)
	{
		temp[0]= pairs[i].p1[0] - (R(1,1)*pairs[i].p2[0] + R(1,2)*pairs[i].p2[1] + R(1,3)*pairs[i].p2[2] + translation[0]);
		temp[1]= pairs[i].p1[1]	- (R(2,1)*pairs[i].p2[0] + R(2,2)*pairs[i].p2[1] + R(2,3)*pairs[i].p2[2] + translation[1]);
		temp[2]= pairs[i].p1[2] - (R(3,1)*pairs[i].p2[0] + R(3,2)*pairs[i].p2[1] + R(3,3)*pairs[i].p2[2] + translation[2]);
		error+= temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2];
	}
	return error;
}

