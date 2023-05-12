function [output] = CONTROL(input)

%% Variable pass-throughs


%% Error

err_theta   = theta_d-theta;
%err_thetadot   = ?; 

%% Controller
if t <= T   
    Mc      = Kpp*err_theta + Kdp*err_thetadot;
end

% Desired control forces/torques
tauC.XD = 0;    % Desired surge force (N)
tauC.YD = 0;    % Desired sway force
tauC.ZD = 0;    % Desired heave force
tauC.KD = 0;    % Desired roll moment (N-m)
tauC.MD = Mc;   % Desired pitch moment
tauC.ND = 0;    % Desired yaw moment 

%% Controls

% Call Vehicle Dynamics
[fuse_drag] = DRAG(u);