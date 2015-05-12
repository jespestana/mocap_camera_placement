%% rotations_script
syms Y P R real

Rx = [     1       0       0; 
           0  cos(R) -sin(R);
           0  sin(R)  cos(R) ];
Ry = [cos(P)       0  sin(P);
           0       1       0;
     -sin(P)       0  cos(P) ];
Rz = [cos(Y) -sin(Y)       0;
      sin(Y)  cos(Y)       0; 
           0       0       1 ];

       
% ejes moviles: UVW
% ejes fijos: XYZ
% yaw Y - Z/W; pitch P - Y/V; roll R - X/U

% WVU ó XYZ
Rt  = Rz*Ry*Rx; 