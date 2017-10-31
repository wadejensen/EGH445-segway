function [A,B] = linearize(p)
    
        mp = p.m + p.M  + p.J/p.r^2;
        

    % inertia matrix
    M = [p.m*p.L^2 p.m*p.L; p.m*p.L mp];
    
    % applied force/torque
    alpha = inv(M) * [p.m*p.L*p.g; 0]
    beta = inv(M) * [-1; 1/p.r];
    
    A = [
        0        1 0 0
        alpha(1) 0 0 0
        0        0 0 1
        alpha(2) 0 0 0
        ];
    
    B = [
        0
        beta(1)
        0
        beta(2)
        ];
