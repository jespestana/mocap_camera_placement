classdef camera_object
    %CAMERA generates a Camera Object
    %   A Geometry Camera object creates a camera object, defining its
    %   optical and geometric properites.
    
    properties
        center
        rot_matrix_w_c
        rot_matrix_c_w
        fov_h_deg
        fov_v_deg
        hc_h_max
        hc_v_max 
        camera_range
    end
 
end

