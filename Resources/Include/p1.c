//This is a simple example meant to test your capabilities with basic library linking functionality. All you have to do is return the output to modelica.



#include <stdio.h>
#include <gsl/gsl_sf_bessel.h>

double sf_bessel(double x)
{
	double y = gsl_sf_bessel_J0 (x);
	return y;	
}


//The main function has been provided solely for the sake of testing the function. It will be removed when the problem set is formally passed on.
/*
int main(void)
{
	double x = 5;
	double a = sf_bessel(x);

	printf("a = %f \n",a);
	return 0;
}
*/
