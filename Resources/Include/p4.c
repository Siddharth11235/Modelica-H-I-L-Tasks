//This involves multiplication of two vectors. 


#include <stdio.h>
#include <gsl/gsl_vector.h>

double *vector_mul(double* a, double *b, size_t size)
{
	int i;
  	gsl_vector * a_vec = gsl_vector_alloc (size);
	gsl_vector * b_vec = gsl_vector_alloc (size);
	gsl_vector * c_vec = gsl_vector_alloc (size);
  	for (i = 0; i < size; i++)
    {
      	gsl_vector_set (a_vec, i, a[i]);
		gsl_vector_set (b_vec, i, b[i]);
    }
	gsl_vector_mul(a_vec,b_vec);
	gsl_vector_memcpy(c_vec, a_vec);
	double c[size];
  	for (i = 0; i < size; i++) /* OUT OF RANGE ERROR */
    {
		c[i] = gsl_vector_get (c_vec, i);
      	printf ("v_%d = %g\n", i, gsl_vector_get (c_vec, i));
    }

  	gsl_vector_free (a_vec);
	gsl_vector_free (b_vec);
	gsl_vector_free (c_vec);
	return c;
}

/*
//The main function has been provided solely for the sake of testing the function. It will be removed when the problem set is formally passed on.
int main (void)
{
	double a[5] = {1.0, 2.0, 3.0, 4.0, 5.0};
	double b[5] = {0.1, 0.2, 0.3, 0.4, 0.5};	
	double *c;
	c = vector_mul(a, b, 5);
	return 0;
}*/
