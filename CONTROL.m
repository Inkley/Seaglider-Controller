function [output] = CONTROL(input)
%   • Characteristic Eq: M_RB*nu_dot + D(nu)*nu + eta_0 = Tau --> ignore C_RB, C_A (Centripetal, Coriolis Forces), M_A negligible in forward direction