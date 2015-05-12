function [ is_visible ] = camera_point_visible_bool( camera, p_w )
%CAMERA_POINT_VISIBLE_BOOL Summary of this function goes here
%   The function calculates whether the point p_w is visible from the
%   camera.

is_visible = false;
p_c = camera.rot_matrix_c_w * (p_w - camera.center);

depth = p_c(1);
if (depth > 0)
    hc_h = p_c(2) / depth;
    hc_v = p_c(3) / depth;
%     disp(' ')
%     disp(' ')
%     disp(hc_h)
%     disp(camera.hc_h_max)
%     disp(hc_v)
%     disp(camera.hc_v_max)
    
    if ( (abs(hc_h) <= camera.hc_h_max) && (abs(hc_v) <= camera.hc_v_max) && ( depth <= camera.camera_range ) )
        is_visible = true;
    end
else
    is_visible = false;
end

end

