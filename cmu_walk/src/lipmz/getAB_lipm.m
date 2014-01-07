function [A, B] = getAB_lipm(x, u, dt, m, g, z)
	A = [ 1 0 dt 0; 
          0 1 0 dt; 
          dt*g/z 0 1 0;
          0 dt*g/z 0 1];
    B = [ 0 0;
          0 0;
          -dt*g/z 0;
          0 -dt*g/z];
    
%     A = [ 1 dt ; dt*g/z 1 ];
%     B = [ 0; -dt*g/z ];
end