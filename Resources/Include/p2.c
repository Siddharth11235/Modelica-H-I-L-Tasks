//This example involves taking two arrays as input and returning their pearson coefficient as output

#include <stdio.h>
#include <gsl/gsl_statistics.h>

double co_var(double* x, double* y, int n)
{

	const size_t stride = 1;
	const size_t n1 = n;

	double pearson = gsl_stats_correlation(x, stride, y, stride, n1);
	return pearson;
}

//The main function has been provided solely for the sake of testing the function. It will be removed when the problem set is formally passed on.
/*
int main(void)
{
	double x[5] = {1.0,  7.7 , 2.5, 3.1, 2.444};
	double y[5] = {1.4,  3.2 , 8.1, 6.7, 2.12};
	int n = 5;
	double p =co_var(x,y,n);
	printf ("%f\n",p);
	return 0;

}*/
