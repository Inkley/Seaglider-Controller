function [output] = DRAG(input)

    % • Wing Lift and Drag (Hydrodynamics) --> dynamics of glider wings are
    % determined by the wing angle of attack. NOT equivalent to vehicle pitch
    % angle
    % • Glider Fuselage Drag --> X_drag = bSq^(3/4) =
    % bL^2(rho/2)^(3/4)u^(3/2); b = 0.0191
    % • Hydrodynamic model: lift, drag, induced drag