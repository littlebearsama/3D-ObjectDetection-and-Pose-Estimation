/**
 * Fit a circle through points using Numerical Recipes.
 */

#include <stdio.h>
#include <math.h>
#include "nrutil.h"

static const double TINY  = 1.0e-20;

static void ludcmp(double **a, int n, int *indx, double *d)
{
	int i,imax=-1,j,k;
	double big,dum,sum,temp;
	double *vv;

	vv=dvector(1,n);
	*d=1.0;
	for (i=1;i<=n;i++) {
		big=0.0;
		for (j=1;j<=n;j++)
			if ((temp=fabs(a[i][j])) > big) big=temp;
		if (big == 0.0) nrerror("Singular matrix in routine ludcmp");
		vv[i]=1.0/big;
	}
	for (j=1;j<=n;j++) {
		for (i=1;i<j;i++) {
			sum=a[i][j];
			for (k=1;k<i;k++) sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
		}
		big=0.0;
		for (i=j;i<=n;i++) {
			sum=a[i][j];
			for (k=1;k<j;k++)
				sum -= a[i][k]*a[k][j];
			a[i][j]=sum;
			if ( (dum=vv[i]*fabs(sum)) >= big) {
				big=dum;
				imax=i;
			}
		}
		if (j != imax) {
			for (k=1;k<=n;k++) {
				dum=a[imax][k];
				a[imax][k]=a[j][k];
				a[j][k]=dum;
			}
			*d = -(*d);
			vv[imax]=vv[j];
		}
		indx[j]=imax;
		if (a[j][j] == 0.0) a[j][j]=TINY;
		if (j != n) {
			dum=1.0/(a[j][j]);
			for (i=j+1;i<=n;i++) a[i][j] *= dum;
		}
	}
	free_dvector(vv,1,n);
}

static void lubksb(double **a, int n, int *indx, double b[])
{
	int i,ii=0,ip,j;
	double sum;

	for (i=1;i<=n;i++) {
		ip=indx[i];
		sum=b[ip];
		b[ip]=b[i];
		if (ii)
			for (j=ii;j<=i-1;j++) sum -= a[i][j]*b[j];
		else if (sum) ii=i;
		b[i]=sum;
	}
	for (i=n;i>=1;i--) {
		sum=b[i];
		for (j=i+1;j<=n;j++) sum -= a[i][j]*b[j];
		b[i]=sum/a[i][i];
	}
}

static void lin_solve(double **A, int n, double *b)
{
  double d;
  int *indx = ivector(1, n);

  ludcmp(A, n, indx, &d);
  lubksb(A, n, indx, b);
  free_ivector(indx, 1, n);
}

static void AperB(double **A, double **B, double **res,
           int rowA, int colA, int rowB, int colB)
{
  int p,q,l;
  for (p=1;p<=rowA;p++)
    for (q=1;q<=colB;q++)
    {
      res[p][q]=0.0;
      for (l=1;l<=colA;l++)
        res[p][q]=res[p][q]+A[p][l]*B[l][q];
    }
}

static void A_TperB(double **A, double **B, double **res,
       int rowA, int colA, int rowB, int colB)
{
  int p,q,l;
  for (p=1;p<=colA;p++)
    for (q=1;q<=colB;q++)
    {
      res[p][q]=0.0;
      for (l=1;l<=rowA;l++)
        res[p][q]=res[p][q]+A[l][p]*B[l][q];
    }
}

static void AperB_T(double **A, double **B, double **res,
       int rowA, int colA, int rowB, int colB)
{
  int p,q,l;
  for (p=1;p<=colA;p++)
    for (q=1;q<=colB;q++)
    {
      res[p][q]=0.0;
      for (l=1;l<=rowA;l++)
        res[p][q]=res[p][q]+A[p][l]*B[q][l];
    }
}

static void A_Tperb(double **A, double *b, double *res, int rowA, int colA)
{
  int p,l;
  for (p=1;p<=colA;p++)
  {
    res[p]=0.0;
    for (l=1;l<=rowA;l++)
      res[p]=res[p]+A[l][p]*b[l];
  }
}

void fit_circle(double x[], double y[], int n, double *cx, double *cy,
    double *r)
{
  int i;
  double **A = dmatrix(1, n, 1, 3);
  double **AT_A = dmatrix(1, 3, 1, 3);
  double *b = dvector(1, n);
  double *AT_b = dvector(1, 3);

  for(i = 0; i < n; i++)
  {
    A[i+1][1] = x[i];
    A[i+1][2] = y[i];
    A[i+1][3] = 1.;
    b[i+1] = -x[i]*x[i] - y[i]*y[i];
  }
  A_TperB(A, A, AT_A, n, 3, n, 3);
  A_Tperb(A, b, AT_b, n, 3);
  lin_solve(AT_A, 3, AT_b);
  *cx = -AT_b[1]/2.;
  *cy = -AT_b[2]/2.;
  *r = sqrt(AT_b[1]*AT_b[1]/4. + AT_b[2]*AT_b[2]/4. - AT_b[3]);

  free_dmatrix(A, 1, n, 1, 3);
  free_dmatrix(AT_A, 1, 3, 1, 3);
  free_dvector(b, 1, n);
  free_dvector(AT_b, 1, 3);
}

