//This is a basic practice case, to build intuition. 

#include  <stdio.h>

double diff1(double x)
{
  double op;
  op =3*x*x + 2*x +1;
  return op;
}

double diff2(double x)
{
  double op;
  op = x*x - 2*x*x*x + 5;
  return op;
}


//The main function has been provided solely for the sake of testing the function. It will be removed when the problem set is formally passed on.
/*
int main(void)
{
	double x = 5;
	double a = diff1(x);
	double b = diff2(x);

	printf("a = %f and b = %f\n",a,b);
	return 0;
}

*/


