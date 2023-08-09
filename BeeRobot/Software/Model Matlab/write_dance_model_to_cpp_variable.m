function [] = write_dance_model_to_cpp_variable(vel)

% number of points in trajectory 
N = size(vel,1);

% fixed-point scaler
scaler_xy = 100000;
scaler_phi = 1000000;

vel_fixed_point_xy  = round( vel(:, 1:2) * scaler_xy );
vel_fixed_point_phi = round( vel(:, 3) * scaler_phi );

fid = fopen('dance_model_static_trajectory.h', 'w');

if (fid)
    fprintf(fid, '#define FIXED_POINT_SCALER_YX %d\n\n', scaler_xy);
    fprintf(fid, '#define FIXED_POINT_SCALER_PHI %d\n\n', scaler_phi);
    fprintf(fid, '#define NUM_POINTS_TRAJECTORY %d\n\n', N);
    write_array_variable_to_file(fid, 'dance_speed_x', vel_fixed_point_xy(:,1));
    write_array_variable_to_file(fid, 'dance_speed_y', vel_fixed_point_xy(:,2));
    write_array_variable_to_file(fid, 'dance_speed_a', vel_fixed_point_phi);
end

fclose(fid);


function write_array_variable_to_file(file_id, variable_name, data)
fprintf(file_id, 'const int %s[] = {%d', variable_name, data(1));
fprintf(file_id, ', %d', data(2:end));
fprintf(file_id, '}; \n\n');