function [HMat] = HomogeneousTransform(alpha, a, theta, d)

HMat1 = [1 0 0 a;
        0 cos(alpha) -sin(alpha) 0;
        0 sin(alpha) cos(alpha) 0;
        0 0 0 1];

HMat2 = [cos(theta) -sin(theta) 0 0;
         sin(theta) cos(theta) 0 0;
         0 0 1 d;
         0 0 0 1];

HMat = HMat1*HMat2;

end

