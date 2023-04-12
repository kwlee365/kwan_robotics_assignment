function [Matrix_3dim] = CrossProductOpertator(Vector_3dim)


x = Vector_3dim(1);
y = Vector_3dim(2);
z = Vector_3dim(3);

Matrix_3dim = [ 0 -z  y;
                z  0 -x;
               -y  x  0];
end

