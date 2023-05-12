function [err] = GlidEquib(nu,theta,C)

global theta    %#ok<REDEFGI,GVMIS>
global C        %#ok<REDEFGI,GVMIS>

%%%% Function is used to determine equilibrium body fixed velocities
u       = nu(1);
w       = nu(2);
alpha   = atan2(w,u);

if abs(alpha) <= 45*pi/180
    err1 = C(1)*sin(theta)-C(2)*sqrt(abs(u))*u-C(3)*(u^2+w^2)-C(4)*atan2(w,u)^2*(u^2+w^2);
    err2 = C(5)*cos(theta)-C(6)*abs(w)*w-C(7)*atan2(w,u)*(u^2+w^2);
else
    err1 = C(1)*sin(theta)-C(2)*sqrt(abs(u))*u-C(3)*(u^2)-C(4)*atan2(w,u)^2*(u^2);
    err2 = C(5)*cos(theta)-C(6)*abs(w)*w-C(7)*atan2(w,u)*(u^2+w^2);
end

err = [err1;err2];