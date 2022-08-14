clc, clear all
%%

% notes:
% - CEB = R is the rotation matrix or DCM

addpath ../fcn_bib/

do_plot_rotation_candidates  = false;
do_simulate_rotation_on_nose = true;

DashedLine = '--';
SolidLine  = '-';
LineWidth = 3.0;

%%

% here we create a set of possible board alignment orientations and store
% them to quat_candidates as quaternions, not unique like this but it will
% do it for testing

angels_possible = (0:10:350).' * pi/180;
number_of_angels_possible = length(angels_possible);
quat_candidates = []; % we append data at the end
for i = 1:number_of_angels_possible         % roll
    for j = 1:number_of_angels_possible     % pitch
        for k = 1:number_of_angels_possible % yaw
            % Rz(yaw) * Ry(pitch) * Rx(roll) -> quaternion
            quat_candidates(end+1,:) = rpy2quat([angels_possible(i), ...
                angels_possible(j), ...
                angels_possible(k)]);
        end
    end
end
N = size(quat_candidates, 1)

if do_plot_rotation_candidates
    for i = 1:N
        R = quat2CEB(quat_candidates(i,:));
        figure(1)
        Rs = R;
        quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), SolidLine, 'color', [1 0 0]  , ...
            'LineWidth', LineWidth*0.5, 'AutoScale', 'off'), grid on, hold  on
        quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), SolidLine, 'color', [0 0.5 0], ...
            'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
        quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), SolidLine, 'color', [0 0 1]  , ...
            'LineWidth', LineWidth*0.5, 'AutoScale', 'off'), hold off
        view(65, 23), axis equal
        pause();
    end
end

%%

% assumptions:
% 1. the user starts with board alignement angles (0, 0, 0)
% 2. the user has his fc already mounted to the drone
% 3. since we only allow a subset of board orientations i dont think it
%    matters if the user has calibrated the accelerometer or not
% 4. from the fc we receive roll, pitch and heading in the convention
%    Rz(-heading) * Ry(pitch) * Rx(roll)

% 1. user input: drone is flat, we evaluate roll, pitch and heading in deg as soon he presses a button
%                the output here is ez_mahony
roll_user_1    =  33.3;
pitch_user_1   = -31.6;
heading_user_1 =   9.0;

% DCM  contains in the coloums [EexB  , EeyB  , EezB  ]  ( basis of the body  frame w.r.t. the earth frame )
%      and      in the rows    [BexE^T; BeyE^T; BezE^T]  ( basis of the earth frame w.r.t. the body  frame )
% R = [cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)]
%     [cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)]
%     [        -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)]
rpy_fc_1 = [ roll_user_1, ...
             pitch_user_1, ...
            -heading_user_1]
R_fc_1 = quat2CEB(rpy2quat(rpy_fc_1 * pi/180)) % convert the user input to a rotation matrix
ez_mahony  = R_fc_1(3,:).';                    % we only need BezE, which is the last row of the rotation matrix and independent of yaw
% % you can also create ez_mahony directly using only roll and pitch
% ez_mahony_ = [-sin(pitch_user_1 * pi/180); ...
%               cos(pitch_user_1 * pi/180) * sin(roll_user_1 * pi/180); ...
%               cos(roll_user_1 * pi/180) * cos(pitch_user_1 * pi/180)]
          
% 2. user input: drone is on nose, we evaluate roll, pitch and heading in deg as soon he presses a button
%                the output here is ex_mahony
%                i do this numerically but you can do this exactly like above with a negative sign
if do_simulate_rotation_on_nose
    R_fc_2 = quat2CEB(rpy2quat([0 pi/2 0])) * R_fc_1;
    rpy_fc_2 = CEB2rpy(R_fc_2);
    ex_mahony = -R_fc_2(3,:).';
    roll_user_2    =  rpy_fc_2(1) * 180/pi;
    pitch_user_2   =  rpy_fc_2(2) * 180/pi;
    heading_user_2 = -rpy_fc_2(3) * 180/pi;
else
    roll_user_2    =  -89.2;
    pitch_user_2   =  -45.8;
    heading_user_2 =  267.0;
    ex_mahony = -[-sin(pitch_user_2 * pi/180); ...
                  cos(pitch_user_2 * pi/180) * sin(roll_user_2 * pi/180); ...
                  cos(roll_user_2 * pi/180) * cos(pitch_user_2 * pi/180)];
end


% calculate ey_mahony and calculate the rotation matrix and the quaternion
% R = [ex_mahony.'; ...
%      ey_mahony.'; ...
%      ez_mahony.']
ey_mahony = cross(ez_mahony, ex_mahony);
R_mahony = [ex_mahony.'; ey_mahony.'; ez_mahony.'];
rpy_mahony = CEB2rpy(R_mahony);     % roll, pitch, yaw from rotation matrix
quat_mahony = rpy2quat(rpy_mahony); % quaternion from roll, pitch, yaw

% this is the exact solution
rpy_mahony = rpy_mahony * 180/pi
R_mahony

% search rotation candidates based on error angle
ang_abs_min = 1e3;
for i = 1:N
    cos_ang = quat_mahony * quat_candidates(i,:).';
    ang = 2*acos(cos_ang);
    if abs(ang) < ang_abs_min
        ang_abs_min = abs(ang);
        quat_ = quat_candidates(i,:);
    end
end

% % search rotation candidates based on trace
% e_trace_min = 1e3;
% for i = 1:N
%     R_candidate_i = quat2CEB(quat_candidates(i,:));
%     e_trace = 0.5 * trace(eye - R_mahony.' * R_candidate_i);
%     if e_trace < e_trace_min
%         e_trace_min = e_trace;
%         quat_ = quat_candidates(i,:);
%     end
% end

% this is the exact solution that matches the candidates best
% we have to insert negative values from here
rpy_ = quat2rpy(quat_) * 180/pi
R_ = quat2CEB(quat_)

figure(2)
ax(1) = subplot(211);
Rs = R_fc_1;
quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), SolidLine, 'color', [1 0 0]  , 'LineWidth', LineWidth*0.5, 'AutoScale', 'off'), grid on, hold  on
quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), SolidLine, 'color', [0 0.5 0], 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), SolidLine, 'color', [0 0 1]  , 'LineWidth', LineWidth*0.5, 'AutoScale', 'off'), hold off
view(65, 23), axis equal
ax(2) = subplot(212);
Rs = R_fc_1;
quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), SolidLine, 'color', [1 0 0]  , 'LineWidth', LineWidth*0.5, 'AutoScale', 'off'), grid on, hold  on
quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), SolidLine, 'color', [0 0.5 0], 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), SolidLine, 'color', [0 0 1]  , 'LineWidth', LineWidth*0.5, 'AutoScale', 'off')
Rs = R_;
quiver3(0, 0, 0, Rs(1,1), Rs(2,1), Rs(3,1), DashedLine, 'color', [1 0 0]  , 'LineWidth', LineWidth, 'AutoScale', 'off')
quiver3(0, 0, 0, Rs(1,2), Rs(2,2), Rs(3,2), DashedLine, 'color', [0 0.5 0], 'LineWidth', LineWidth, 'AutoScale', 'off')
quiver3(0, 0, 0, Rs(1,3), Rs(2,3), Rs(3,3), DashedLine, 'color', [0 0 1]  , 'LineWidth', LineWidth, 'AutoScale', 'off'), hold off
view(65, 23), axis equal
link = linkprop(ax, {'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
setappdata(gcf, 'StoreTheLink', link), clear ax

