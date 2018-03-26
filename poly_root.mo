function poly_root

input Real[:] a;
output Real [size(a,1)*2] y;

 external poly_roots(a,size(a,1),y,size(y,1) ) annotation( Include = "#include \"p3.c\"", Library={"gsl","gslcblas","m"});

end poly_root;
