%neues Tanzmodell
function [P, V] = dance_model()

global waggle_angle waggle_divergence waggle_speed waggle_duration waggle_ramp_length waggle_freq waggle_amp_xy waggle_amp_a return_speed return_duration return_orientation_function return_velocityX_function return_velocityY_function dt T drift
global return_speed return_duration return_orientation_function dt T drift Qglobal pos exc
global return_orientation_weights return_speed_function_offset return_orientation_function_offset turn_scale return_orientation_function

% parameters
waggle_angle        = 0;                    % in degrees?, ccw, 0° points upwards (TODO: check in code)
waggle_divergence   = 25 * pi / 180;        % in rad, default im paper: 32°
waggle_speed        = 15;                   % in mm/s forward speed in waggle run
waggle_duration     = 450;                  % in ms
waggle_freq         = 13;                   % in Hz
waggle_amp_a        = 5.9 * pi / 180;       % in rad, peak to baseline 
return_speed        = 20.1;                 % in mm/s
return_duration     = 2130;                 % in ms
exc                 = 22;                   % in mm, length of the excenter (motor axis to center of bee)
waggle_ramp_length  = 150;                  %[ms]

resamling_steps     = 700;
resampling_coeff    = resamling_steps / return_duration;

%return orientation velocity
%        return_orientation_velocity(x) = p1*x^3 + p2*x^2 + p3*x + p4
%      Coefficients (with 95% confidence bounds):
%        p1 =      -4.581  (-4.767, -4.395)
%        p2 =       7.517  (7.234, 7.801)
%        p3 =      -3.572  (-3.694, -3.45)
%        p4 =      0.8562  (0.8421, 0.8703)
p1 =      -4.581;
p2 =       7.517;
p3 =      -3.572;
p4 =       0.8562;
          
func_string = sprintf('%f*(%f*x.^3 + %f*x.^2 + %f*x + %f)',resampling_coeff, p1, p2, p3, p4)
return_orientation_function = inline(func_string);

t = linspace(0,1,return_duration);

alpha = exp(10)/100;

return_orientation_weights = ((((t-0.5).^2)+alpha));
return_orientation_weights = return_orientation_weights / ( (sum(return_orientation_weights) / return_duration) );

figure

plot(return_orientation_function(t) .* return_orientation_weights)

figure

%t = linspace(0, 1, return_duration);
%plot(t, return_orientation_function(t))

%we have to scale the return_orientation_velocity by turn scale to be in 
%the right pose when entering the next waggle                      
t = linspace(0,1,return_duration);
f = return_orientation_function(t);
turn_scale = (360-(waggle_divergence/2)*180/pi)/sum(f)

%return velocity
% curve_speedx = 
% 
%      General model:
%        curve_speedx(x) = p0*x^4 + p1*x^3 + p2*x^2 + p3*x + p4
%      Coefficients (with 95% confidence bounds):
%        p0 =       0.728  (0.6792, 0.7768)
%        p1 =      -1.466  (-1.564, -1.367)
%        p2 =      0.9225  (0.857, 0.988)
%        p3 =     -0.1805  (-0.1966, -0.1644)
%        p4 =     0.03409  (0.03293, 0.03525)
p0 =        0.728;
p1 =       -1.466;
p2 =        0.9225;
p3 =       -0.1805;
p4 =        0.03409;  
func_string = sprintf('%f*(%f*x.^4 + %f*x.^3 + %f*x.^2 + %f*x + %f)',resampling_coeff, p0, p1, p2, p3, p4);
return_velocityX_function = inline(func_string);               


% curve_speedy = 
% 
%      General model:
%        curve_speedy(x) = p0*x^4 + p1*x^3 + p2*x^2 + p3*x + p4
%      Coefficients (with 95% confidence bounds):
%        p0 =      0.2683  (0.2002, 0.3364)
%        p1 =     -0.2951  (-0.4325, -0.1578)
%        p2 =    -0.04566  (-0.137, 0.04571)
%        p3 =      0.1027  (0.08023, 0.1251)
%        p4 =     -0.0337  (-0.03532, -0.03209)
p0 =        0.2683;
p1 =       -0.2951;
p2 =        -0.04566;
p3 =       0.1027;
p4 =        -0.0337;  
func_string = sprintf('%f*(%f*x.^4 + %f*x.^3 + %f*x.^2 + %f*x + %f)',resampling_coeff, p0, p1, p2, p3, p4);
return_velocityY_function = inline(func_string);               


%close all

figure
plot(return_velocityX_function(t))
hold
plot(return_velocityY_function(t))

sum(return_velocityX_function(t))
sum(return_velocityY_function(t))

% stores timeseries of positions
P = [];

% stores timeseries of speeds
V = [];


%initial position
pos = [0 0 waggle_divergence/2];
%?
Q = 0;

for i = 1:1
    %1st waggle
    for t = linspace(0, waggle_duration, waggle_duration)
        q = getWaggleVelocity(t,1);
        V = [V; q];
        
        pos = pos + q; %cumulative sum
        P = [P; pos]; %positions
    end


    %1st return
    for t = linspace(0, return_duration, return_duration)
        q = getReturnVelocity(t, pos(3),1);
        V = [V; q];
        
        pos = pos + q;
        P = [P; pos];    
    end

    %pos = [0 0 -waggle_divergence/2];
    %pos(3) = -waggle_divergence/2; 

    %2nd waggle
    for t = linspace(0, waggle_duration, waggle_duration)
        q = getWaggleVelocity(t,0);
        V = [V; q];
        
        pos = pos + q; %cumulative sum
        P = [P; pos]; %positions
    end

    %2nd return
    for t = linspace(0, return_duration, return_duration)
        q = getReturnVelocity(t, pos(3),0);
        V = [V; q];
        
        pos = pos + q;
        P = [P; pos];    
    end
end


size(P)
figure
hold off


% subplot(2,1,1)
% plot(v(:,1), v(:,2), 'g-.');
% hold on
% 
% v(:,1) = v(:,1) - exc*cos(v(:,3));
% v(:,2) = v(:,2) - exc*sin(v(:,3));
% 
% for i = 2:length(v)
%     c = sqrt((v(i,1) - v(i-1,1))^2 + (v(i,2) - v(i-1,2))^2);
%     c = c*1000; % mm per sec
%     c = min(1, (c) / 10); %% 20 mm per second is max velocity
%     c = max(0, c);
%     
%     handle = plot(v(i,1), v(i,2));
%     set(handle,'Color',[c 0 (1-c)]);
%     hold on
% 
% end
% axis image

% subplot(2,1,2)
% plot(v(:,3)*180/pi)
% subsam = 100;
% ind = waggle_duration:subsam:(waggle_duration + return_duration);
% x1 = v(ind,1)'; 
% y1 = v(ind,2)';
% 
% x2 = x1 + .5*(cos(v(ind,3))');
% y2 = y1 + .5*(sin(v(ind,3))');
% 
% x3 = x1 - .5*(cos(v(ind,3))');
% y3 = y1 - .5*(sin(v(ind,3))');
% 
% line([x2; x3], [y2; y3])
% 
% z = 1;
% while (1)
%     plot(v(:,1), v(:,2), 'r');
%     hold on
%     line([x2(z); x3(z)], [y2(z); y3(z)])
%     hold off
%     z = mod(z + 1, 2*(return_duration + waggle_duration));
%     axis equal
%     pause(0.1)
% end





   
function v = getWaggleVelocity(t, left)
global waggle_angle waggle_divergence waggle_speed waggle_duration waggle_ramp_length waggle_freq waggle_amp_xy waggle_amp_a return_speed return_duration return_orientation_function dt T drift
%return value
v = [0 0 0];

%number of waggle periods
num_periods = ((waggle_duration/1000)*waggle_freq);
sample_points_for_one_period = 1000 / waggle_freq;

if (left)
    sign = 1;
else
    sign = -1;
end

ramp = 1;
num_periods = (waggle_duration/1000.0) * waggle_freq;
sample_points_for_one_period = 1000.0 / waggle_freq;

%damp one period at start and end
if (t < sample_points_for_one_period)
    ramp = t / sample_points_for_one_period;
else 
    if (t > (sample_points_for_one_period * (num_periods - 1)))
        ramp = (waggle_duration - t) / sample_points_for_one_period;
    end
end

%+ 0.5 := sample the cosine symmetrically
%make it in [0,1]
%t_prcnt = ((t+.5)/waggle_duration);
t_prcnt = (t/waggle_duration);
num_periods = ((waggle_duration/1000)*waggle_freq);

%v(3) = waggle_amp_a * cos( ((waggle_duration / 1000)*waggle_freq) *
%t_prcnt * 2 * pi) / arc;

v(3) = waggle_amp_a * ramp * cos(num_periods * t_prcnt * 2 * pi) / (1000 / (waggle_freq * 4) / (pi*0.5)) ;
v(1) = waggle_speed * cos(waggle_angle + sign*waggle_divergence/2) / 1000; 
v(2) = waggle_speed * sin(waggle_angle + sign*waggle_divergence/2) / 1000; 

function v = getReturnVelocity(t, Ocur, left)
global return_orientation_weights turn_scale return_duration exc return_orientation_function return_velocityX_function return_velocityY_function
v = [0 0 0];	
if (left)
    sign = 1;
else
    sign = -1;
end

ramp = 1;
%damp one period at start and end
d = 100;
if (t < d)
    ramp = t / d;
end
% unit_t = (t+0.5) / return_duration;
unit_t = t / return_duration;

v(3) = 1.05*turn_scale*sign*ramp*pi*return_orientation_function(unit_t)/180;
v(3) = return_orientation_weights(min(return_duration, floor(t)+1)) * turn_scale*sign*ramp*pi*return_orientation_function(unit_t)/180;
% v(1) = (return_velocity_function(unit_t) * cos(Ocur + v(3)) / 1000) + exc*(cos(Ocur+v(3))-cos(Ocur));
% v(2) = (return_velocity_function(unit_t) * sin(Ocur + v(3)) / 1000) + exc*(sin(Ocur+v(3))-sin(Ocur));
%%ACHTUNG!! x und y veocities zur�ck drehen um 
% Ocur !!!!
v_fwd = turn_scale*ramp*return_velocityX_function(unit_t);
v_swd = turn_scale*ramp*sign*return_velocityY_function(unit_t);
c = cos(Ocur);
s = sin(Ocur);
%rotate
v(1) = c*v_fwd - s*v_swd + exc*(cos(Ocur+v(3))-cos(Ocur));
v(2) = s*v_fwd + c*v_swd + exc*(sin(Ocur+v(3))-sin(Ocur));



function v = waggleVelocity(t, left)
% 
% t_prcnt = (t + .5) / waggle_duration;
% ramp = 1;
% sign = 1;
% 					
% num_periods = (waggle_duration/1000.0) * waggle_freq;
% sample_points_for_one_period = 1000.0 / waggle_freq;
% 
% %damp one period at start and end
% if (t < sample_points_for_one_period)
%     ramp = t / sample_points_for_one_period;
% else 
%     if (t > (sample_points_for_one_period * (num_periods - 1)))
%         ramp = (float)(waggle_duration - t) / sample_points_for_one_period;
%     end
% end
% 
% %     Erl�uterung:
% %     #
% %     # cos(t_prcnt * 2 * CV_PI) eine periode auf einen waggle run
% %     # cos(waggle_freq * t2pi) z.b. 13 pro waggle run
% %     # (params->waggle_duration / 1000) waggle dauer in sec
% 
% v(3) = waggle_amp_a * ramp * cos( num_periods * t_prcnt * 2 * pi) / (1000.0 / (waggle_freq * 4) / (pi*0.5)) ;
% 
% if ( left ~= 0 )
%     sign = -1;
% end
% 
% v(3) = sign * v(3);
% a = angle + sign * divergence/2;
% v(1) = waggle_speed * cos(a) / 1000;
% v(2) = waggle_speed * sin(a) / 1000;
% 					
% 					Pcur.x += R.x;
% 					Pcur.y += R.y;
% 					Pcur.z += R.z;
% 			}
% 			else // RETURN RUN
% 			{
% 				R.z = params->return_orientation_difference(t);
% 				if ( pathID == BR_DANCESIM_SUBPATH_RETURN_LEFT)
% 					R.z = -R.z;
% 
% 				double c1 = (cos(R.z + Pcur.z) - cos(Pcur.z))*params->exc_length;
% 				double c2 = (sin(R.z + Pcur.z) - sin(Pcur.z))*params->exc_length;
% 
% 				R.x = (params->return_speed(t) * cos(R.z + Pcur.z) / 1000) + c1;
% 				R.y = (params->return_speed(t) * sin(R.z + Pcur.z) / 1000) + c2;
% 				Pcur.z += R.z;
% 			}
% 			return R;