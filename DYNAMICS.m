function [fuse_drag] = DYNAMICS(u)
    
% • Hydrodynamic model: lift, drag, induced drag
% • From Bennett et al. 2021
%   - Output:   Glider Fuselage Drag (N)
%   - Input:    Surge Velocity, u (m/s)

%% Variable pass-throughs

%% Fuselage Drag
X_drag = b*L^2*((rho_s/2)^(3/4))*u^(3/2); % Glider Fuselage Drag

fuse_drag = X_drag; 

%% Glider Hydrodynamic Coefficients

% Forces

% Moments

% Define Hydrostatic forces

% Define sum of hydrodynamic forces in each DOF

% Total forces
X       = XHS + Xhyd + tauC.XD;
Y       = YHS + Yhyd + tauC.YD;
Z       = ZHS + Zhyd + tauC.ZD;
K       = KHS + Khyd + tauC.KD + tau_cmg.K; 
M       = MHS + Mhyd + tauC.MD + tau_cmg.M;
N       = NHS + Nhyd + tauC.ND + tau_cmg.N;

tau     =   [X;Y;Z;K;M;N];

MRB     =   [m+Xudot, 0, m*zg+Xqdot;
            0, m+Zwdot, -m*xg;
            m*zg + Mudot; -m*xg, Iyy + Mqdot];

D       =   [Xu, 0, (m+Zwdot)*w + xg*q + Xqdot;
            0, Zw, -(m+Xudot)*u - (Zg-Xqdot)*q;
            -(m+zwdot)*w - xg*q + Mu; (m + Xudot)*u - (zg-Xqdot)*q, Mq];
            
state_vec_nu = [u;v;w;p;q;r];

% N2L in non-inertial frame gives rate of change of body fixed vector nu

% Define rotation matrices to get velocities in inertial frame
J1      =   [cos(theta), -sin(theta),0;
            sin(thetat), cos(theta), 1;
            0, 0, 1];

% All together Now
