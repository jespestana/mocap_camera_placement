% mocap_camera_placement:
%   Program to calculate the approximate tracking volume obtained by a
%   tracking system camera setup. Current values are valid for Flex13
%   Optitrack tracking system.

clear all
close all
clc

rotations_script
DEG2RAD = pi    / 180.0;
RAD2DEG = 180.0 / pi;

%% Calculation of the Tracking Volume

% Configuration parameters
L1 = 4.25;
L2 = 5.40;
% fov_h = 56.00; % deg
fov_h = 56.00; % deg
fov_v = 44.80; % deg
camera_range = 10.0; % m
DE =.15; % m
Vox_Size = -2; % 0.125 m (Voxel size)
% Vox_Size = log(.2)/log(2) + 1; // 0.20 m
% Vox_Size = -3; % 0.0625 m
% min_cameras_point = 3; << it is set later

% Volume where the tracking volume is calculated:
%                  orig_x   orig_y  orig_z  size_x  size_y  size_z
% tracking_volume = [4.0      4.0     0.0     L2+8   L1+8   3.125];
%                  orig_x   orig_y  orig_z  size_x  size_y  size_z
tracking_volume = [0.15     0.15    0.0     L2+.30  L1+.60  3.125];
origin = tracking_volume(1:3)';
% Camera parameters M = matrix( num_cameras, 8)
%    attitude: {Y,yaw} , {P,pitch} , {R,roll} in deg
%    fov: field of view in deg (horizontal:h, vertical:v)
%    center_x center_y center_z att_Y   att_P             att_R   fov_h   fov_v
% M = [0.0      0.0      3.05     45.0    4.16+fov_v/2.0  0.0     56.0    44.80
%      0.0      0.0      0.0       0.0     0.0            0.0     56.0    44.80;
%      0.0      L1      3.0     -45.0    50.0             0.0     56.0    44.80];

%    center_x center_y center_z att_Y    att_P           att_R   fov_h   fov_v
% M = [0.0+DE   L1-DE    3.05     -45.0    4.16+fov_v/2.0  0.0     fov_h    fov_v;
%      0.0+DE   L1/2.0   3.05       0.0    4.16+fov_v/2.0  0.0     fov_h    fov_v;
%      0.0+DE   0.0+DE   3.05      45.0    4.16+fov_v/2.0  0.0     fov_h    fov_v;
%      L2/2.0   0.0+DE   3.05      90.0    4.16+fov_v/2.0  0.0     fov_h    fov_v;
%      L2-DE    0.0+DE   3.05     135.0    4.16+fov_v/2.0  0.0     fov_h    fov_v;
%      L2-DE    L1/2.0   3.05     180.0    4.16+fov_v/2.0  0.0     fov_h    fov_v;
%      L2-DE    L1-DE    3.05     225.0    4.16+fov_v/2.0  0.0     fov_h    fov_v;
%      (L2/2.0) L1-DE    3.05     270.0    4.16+fov_v/2.0  0.0     fov_h    fov_v];

% deltaY = atan(L1/L2) * RAD2DEG;
deltaY = atan((2.5)/5.0) * RAD2DEG; % 26.5 deg
% deltaY = atan((2.12)/5.0) * RAD2DEG; % 23.0 deg
% deltaY = atan((2.0)/5.0) * RAD2DEG; % 21.8 deg
% deltaY = atan((1.5)/5.0) * RAD2DEG; % 16.7 deg
%    center_x center_y center_z att_Y        att_P            att_R   fov_h   fov_v
M = [0.0+DE   L1-DE    3.05     0.0-deltaY   4.16+fov_v/2.0   0.0     fov_h   fov_v;
     0.0+DE   L1/2.0   3.05     0.0          4.16+fov_v/2.0   0.0     fov_h   fov_v;
     0.0+DE   0.0+DE   3.05     0.0+deltaY   4.16+fov_v/2.0   0.0     fov_h   fov_v;
     L2/2.0   0.0+DE   3.05     90.0         4.16+fov_v/2.0   0.0     fov_h   fov_v;
     L2-DE    0.0+DE   3.05     180.0-deltaY 4.16+fov_v/2.0   0.0     fov_h   fov_v;
     L2-DE    L1/2.0   3.05     180.0        4.16+fov_v/2.0   0.0     fov_h   fov_v;
     L2-DE    L1-DE    3.05     180.0+deltaY 4.16+fov_v/2.0   0.0     fov_h   fov_v;
     L2/2.0   L1-DE    3.05     270.0        4.16+fov_v/2.0   0.0     fov_h   fov_v];

%    center_x center_y center_z att_Y   att_P             att_R   fov_h   fov_v
% M = [0.0      2.0      3.0       0.0    4.16+fov_v/2.0  0.0     fov_h    fov_v]

% Creating "camera_object"s
%   Note that the camera axis in the local frame are:
%        "x direction" optical axis
%        "y direction" left
%        "z direction" right
Ncam = size(M,1);
cameras = cell(Ncam,1);
for i=1:Ncam
    cameras{i} = camera_object;
    cameras{i}.center = M(i,1:3)' + origin;
    Y = M(i,4) * DEG2RAD;
    P = M(i,5) * DEG2RAD;
    R = M(i,6) * DEG2RAD;
    cameras{i}.rot_matrix_w_c = eval(Rt); % from camera to world
    cameras{i}.rot_matrix_c_w = cameras{i}.rot_matrix_w_c'; % from world to camera
    cameras{i}.fov_h_deg = M(i,7);
    cameras{i}.fov_v_deg = M(i,8);
    cameras{i}.hc_h_max = tan( cameras{i}.fov_h_deg * DEG2RAD / 2.0);
    cameras{i}.hc_v_max = tan( cameras{i}.fov_v_deg * DEG2RAD / 2.0);
    cameras{i}.camera_range = camera_range;
end

% Calculation of the Tracked Volume
voxel_side_lenght = 2^(Vox_Size-1) % m
Nvolx = ceil( tracking_volume(4) / voxel_side_lenght );
Nvoly = ceil( tracking_volume(5) / voxel_side_lenght );
Nvolz = ceil( tracking_volume(6) / voxel_side_lenght );

VoxelMat=zeros(Nvolx,Nvoly,Nvolz);
for i=1:Nvolx
    for j=1:Nvoly
        for k=1:Nvolz
            p_w = ( [i j k]' - 0.5 *[1 1 1]' ) * voxel_side_lenght;
            visible_from = 0;
            for n=1:Ncam
                if ( camera_point_visible_bool( cameras{n}, p_w ) )
                    visible_from = visible_from + 1;
                end
            end
            VoxelMat(i,j,k) = visible_from;
        end
    end
end
VoxelMat_backup = VoxelMat;

%% Plotting Tracking Volume
min_cameras_point = 3; % sets minimum number of cameras that have to see a point
                       %   to be considered as "tracked"
VoxelMat = VoxelMat_backup >= min_cameras_point;
figure
% Voxel representation of Tracking Volume
[vol_handle]=VoxelPlotter(VoxelMat, Vox_Size);
view(3);
daspect([1,1,1]);
set(gca,'xlim',[0 Nvolx] * voxel_side_lenght, ...
        'ylim',[0 Nvoly] * voxel_side_lenght, ...
        'zlim',[0 Nvolz] * voxel_side_lenght);
hold all

scatter3(M(:,1),M(:,2),M(:,3),'b','fill')
legend('tracking volume','cameras')
A = zeros(Ncam,3);
for n=1:Ncam
    text(M(n,1),M(n,2),M(n,3)+.1,num2str(n))
    A(n,1:3) = cameras{n}.rot_matrix_w_c(:,1)';
end
larrow = 0.3;
quiver3(M(:,1),M(:,2),M(:,3),A(:,1)*larrow,A(:,2)*larrow,A(:,3)*larrow,'b','LineWidth',3,'AutoScale','off')
axis equal
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

%% camera_point_visible_bool Test script
% 
% p_w    = [1.0    1.0    0.0]' * 3.2 / sqrt(2) + [0 0 1]';
% p_w    = [1.0   -0.53   -0.41]';
% p_w    = [0.0    4.0    0.0]';
% 
% % cameras{2}
% % camera_point_visible_bool( cameras{2}, p_w )
% 
% visible_from = 0;
% visible_from_cam = zeros(1,Ncam);
% for n=1:Ncam
%     if ( camera_point_visible_bool( cameras{n}, p_w ) )
%         visible_from = visible_from + 1;
%         visible_from_cam(1,n) = 1;
%     end
% end
% 
% visible_from 
% visible_from_cam

%% voxelplotter_example
% 
% voxelplotter_example
