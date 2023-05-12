function [fuse_drag] = DRAG(u)
    
% • Hydrodynamic model: lift, drag, induced drag
% • From Bennett et al. 2021
%   - Output:   Glider Fuselage Drag (N)
%   - Input:    Surge Velocity, u (m/s)

%% Variable pass-throughs

%% Fuselage Drag
X_drag = b*L^2*((rho_s/2)^(3/4))*u^(3/2); % Glider Fuselage Drag

fuse_drag = X_drag; 