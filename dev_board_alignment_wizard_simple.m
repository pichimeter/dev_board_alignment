clc, clear all

addpath ../fcn_bib

is_matlab = false;  % false -> octave

if is_matlab
  addpath ../fcn_bib
else
  addpath fcn_bib_copy
end

%%

% here we create a set of possible board alignment orientations and store
% them to quat_candidates as quaternions, not unique like this but it will
% do it for testing

angels_possible = (0:30:350).' * pi/180;
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

% DCM  contains in the coloums [EexB  , EeyB  , EezB  ]  ( basis of the body  frame w.r.t. the earth frame )
%      and      in the rows    [BexE^T; BeyE^T; BezE^T]  ( basis of the earth frame w.r.t. the body  frame )
% R = [cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)]
%     [cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)]
%     [        -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)]
ez_mahony = [-sin(pitch_user_1 * pi/180); ...
              cos(pitch_user_1 * pi/180) * sin(roll_user_1 * pi/180); ...
              cos(roll_user_1 * pi/180) * cos(pitch_user_1 * pi/180)];

% 2. user input: drone is on nose, we evaluate roll, pitch and heading in deg as soon he presses a button
%                the output here is ex_mahony
%                mind the negative sign
roll_user_2    =  -89.2;
pitch_user_2   =  -45.8;
ex_mahony = -[-sin(pitch_user_2 * pi/180); ...
              cos(pitch_user_2 * pi/180) * sin(roll_user_2 * pi/180); ...
              cos(roll_user_2 * pi/180) * cos(pitch_user_2 * pi/180)];

% calculate ey_mahony and calculate the rotation matrix and the quaternion
% R = [ex_mahony.'; ...
%      ey_mahony.'; ...
%      ez_mahony.']
ey_mahony = cross(ez_mahony, ex_mahony);
R_mahony = [ex_mahony.'; ey_mahony.'; ez_mahony.'];
quat_mahony = rpy2quat(CEB2rpy(R_mahony)); % quaternion from rotation matrix

% this is the exact solution
rpy_mahony = CEB2rpy(R_mahony) * 180/pi    % roll, pitch, yaw from rotation matrix

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

