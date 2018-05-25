//This program takes a double array and returns the roots of the polynomial defined by them. The roots in question can be complex, and the complex data type will need usage in modelica.



#include <stdio.h>
#include <gsl/gsl_poly.h>


void poly_roots(double* a, const size_t size,double *z,size_t size_z)
{
  /* coefficients of P(x) =  -1 + x^5  */

  gsl_poly_complex_workspace * w = gsl_poly_complex_workspace_alloc ((size+1));

  gsl_poly_complex_solve (a, size+1, w, z);

  gsl_poly_complex_workspace_free (w);
	for (int i = 0; i < size; i++)
    {
      printf ("z%d = %+.18f %+.18f\n",
              i, z[2*i], z[2*i+1]);
    }
  

}

//The main function has been provided solely for the sake of testing the function. It will be removed when the problem set is formally passed on.
/*
int main (void)
{
 	double a[6] = { -1, 0, 0, 0, 0, 1 };
	int size_a = 5;
	double *z;	
	poly_roots(a, size_a,z, 10);
	
	return 0;
}

*/
