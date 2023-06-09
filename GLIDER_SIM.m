%   iRobot 1KA Seaglider Controller Code
%   ORE 657 Autonomous Marine Systems
%   Authors: Clara Encke, Tyler Inkley, Patrick Ng, Guilherme Silva

clc; clear; close all;

%% Working Notes
%{
    • Engine Torque Models - Lecture 15/16 Slide 19... someday
    • Glider Control: generate expected commands
    • Dynamic simulation vs. just equilibrium condition formulation
    • G&C Phases: Pitch adjustment, Variable Bouyancy Device (VBD) adjustment, roll adjustment
        Another periodic action performed during *profile phases* (Dive,
        Apogee, Climb), occuring at intervals defined in science file and only
        done when neccessary 
    • --> Add G&C phase adjustments in the future
%}

%% Equilibrium Conditions: Pitch Control (Lecture 20), From MK
%   Function to calculate glider velocities at different climb and dive pitch angles

%   Environmental Factors
rho_s   = 1025;     % Seawater Density Average, kg/m^3
g       = 9.81;     % Accelerate due to Gravity, m/s^2


%   Define Parameter Space
n           = 100;          % Number of points in theta range
thetamax    = 25*pi/180;    % Max theta value (rad)
Theta       = linspace(-thetamax,thetamax,n);

%   Define Glider Parameters

glider.b       = 0.0191;                % Glider fuselage drag coefficient
glider.L       = 1.8;                   % Glider length (m)
glider.Bmax    = 0.3;                   % Max glider breadth (m)
glider.Bave    = 0.2;                   % Average glider breadth (m)
glider.Lw      = (1-glider.Bmax)/2;     % Wing span 
glider.cw      = 0.07;                  % Average wing chord (m)
glider.AR      = glider.Lw/glider.cw;   % Wing Aspect Ratio
glider.Sw      = glider.Lw*glider.cw ;  % Wing wetted area
glider.m       = 55.783;                % GLider mass (kg), where does this value come from?
                                            % From manual: Nominal - 52 kg, Min - 50 kg,
                                            % Max - 54 kg
                                            % MK Calc. based off volume range of glider

Cd0     = 0.015;            % Coefficient of drag of wing at 0 angle of attack
Cdc     = 1.2;              % Coefficient of drag of a cylinder in cross flow

%   Define operational parameters
Delmax  = 55052*(1/100)^3;
dDel    = Delmax *0.05;

%   Run for loop to calculate equilibrium velocities at different angles
c2      = glider.b*glider.L^2*(rho_s/2)^(3/4);
c3      = Cd0*rho_s*glider.Sw;
c4      = 4*pi*glider.AR/(glider.AR+2)^2*rho_s*glider.Sw;
c6      = rho_s*glider.L*glider.Bave*Cdc/2;
c7      = 2*pi*rho_s*glider.AR/(glider.AR+2)*glider.Sw;

%   Pre-allocate
u       = zeros;
w       = u;
alpha   = u;
xdot    = u;
zdot    = u;

%   Global Variables (try to find a better workaround!)
global theta    %#ok<GVMIS>
global C        %#ok<GVMIS>

for ii = 1:n
    theta = Theta(ii);
    if theta > 0
        Del = Delmax;
    else
        Del = Delmax-dDel;
    end
    c1  = (rho_s*Del-glider.m)*g;
    c5  = (glider.m-rho_s*Del)*g;
    C   = [c1,c2,c3,c4,c5,c6,c7];

    %   Numerically solve for [u,v] pair that solves force balance eqn.
    nuu     = [2;2];    % Upper bound for u,w
    nul     = [0,-2];   % Lower bound for u,w
    nu0     = [0,0];    % Initial guess

    nufit   = lsqnonlin(@GlidEquib,nu0,nul,nuu);

    u(ii)   = nufit(1);
    w(ii)   = nufit(2);
    alpha(ii) = atan2(w(ii),u(ii));

    %   Solve for velocities in the inertial coordinate system
    J1 = [cos(theta),-sin(theta);sin(theta),cos(theta)];
    etadot = J1*nufit';
    xdot(ii) = etadot(1);
    zdot(ii) = etadot(2);
end

v_tot = sqrt(u.^2+w.^2);
v_dot_tot = sqrt(xdot.^2 + zdot.^2);
Theta_deg = Theta.*(180/pi);
alpha_deg = alpha.*(180/pi);

%% Results
figure
plot(Theta_deg,u,'LineWidth',2)   
hold on
plot(Theta_deg,w,'LineWidth',2)   
plot(Theta_deg,v_tot,'LineWidth',2)   
plot([-25 25],nu0, '--k','LineWidth',2)   
    xlabel('Glider Pitch Angle, \theta (deg)')
    ylabel('Body Fixed Velocities (m/s)')
    title('Glider Velocity vs. Pitch Angle')
    grid on; grid minor
    legend('u','w','V_{tot}','\nu_0')
    set(gca,'FontSize',12,'LineWidth',1.0)
    print(gcf,'-depsc','nuplot')

figure
plot(Theta_deg,alpha_deg,'LineWidth',2)   
hold on
plot(Theta_deg,Theta_deg,'--r','LineWidth',2)
    xlabel('Glider Pitch Angle, \theta (deg)')
    ylabel('Wing Angle of Atack (deg)')
    title('Glider Angle of Attack vs. Pitch Angle')
    grid on; grid minor
    legend('Wing Angle of Attack','Glider Angle (slope = 1)')
    set(gca,'FontSize',12,'LineWidth',1.0)
    print(gcf,'-depsc','attack')

figure
plot(Theta_deg,xdot,'LineWidth',2)   
hold on
plot(Theta_deg,zdot,'LineWidth',2)  
plot(Theta_deg,v_dot_tot,'LineWidth',2)   
plot([-25 25],nu0, '--k','LineWidth',2)   
    xlabel('Glider Pitch Angle, \theta (deg)')
    ylabel('Inertial Velocities (m/s)')
    title('Glider Velocity vs. Pitch Angle')
    grid on; grid minor
    legend('$\dot{x}$','$\dot{z}$','$V_{tot}$','$\nu_0$','Interpreter','Latex')
    set(gca,'FontSize',12,'LineWidth',1.0)
    print(gcf,'-depsc','etaplot')

%% Update Controller Simulation Suite
%   ASSUMING NO TURNING DYNAMICS --> LONGITUDINAL VEHICLE DYNAMICS 

% Inertial states (starting from rest at the surface)
state.x     = 0;
state.y     = 0;
%state.z     = 0;
%state.phi   = 0;
state.theta = 0;
%state.psi   = 0;

% Body-fixed states (starting from rest at the surface)
state.u     = 0;
%state.v     = 0;
state.w     = 0;
%state.p     = 0;
state.q     = 0;
%state.r     = 0; 

state_vec = [state.x state.y state.theta state.u state.w state.q];

% Gains
% gains.Kpq = ?;
% gains.Kdq = ?;

% Loop frequencies 
%loop.cycleT = 3;                    % s
%loop.fc     = 1/loop.cycleT;        % fc = 0.33 Hz 

% Seaglider Properties
x_f   = 6000; % Desired forward travel distance
d_f   = 1000; % Glider cycle depth

% glider.d_batt;         % Horizontal position of battery assembly center of mass, relative to origin G_0
% glider.z_batt;         % Vertical position of center of mass
% glider.sigma_v;        % Bladder volume ratio
% glider.delta_PH;       % Pressure housing displacement 
% glider.d_b;            % Horizontal distance to bladder
% glider.sigma_m;        % Battery mass ratio
% glider.m_0;            % Glider Mass (minus battery assembly)
% glider.rho_oil;        % Oil density
% glider.p_aft;          % Battery pack position associated with pitch angle theta_climb
% glider.p_fore;         % Battery pack position associated with pitch angle theta_dive
% glider.theta_dive;     % Dive equilibrium condition
% glider.eta_1WP;        % Set of navigation waypoints (lat,lon)

%% Control Cycle Hierarchy

    %%%%%   SURFACE    %%%%%

% 1) Start at surface, sample from GPS receiver to determine vehicle position

% 2) Command first cage motor to drive battery pack to position p_fore

% 3) Command buoyancy engine to pump all working fluid from external bladder to internal chamber. 

    %%%%%   *DIVE*     %%%%%

% 4) Every 5s sample external hydrostatic pressure, calculate current vehicle depth

% 5) Update the remaining vehicle states utilizing a dead-reckoning scheme

%d_batt = tan(theta_dive)*z_batt;

    %%%%%   *APOGEE*   %%%%%

% 6) Command buoyancy engine to pump all working fluid from internal chamber back into external bladder

% 7) Command first cage motor to drive battery pack to position p_aft

    %%%%%   *CLIMB*    %%%%%

% 8) Sample pressure and update vehicle states in same technique that was used during dive phase

% 9) Once vehicle reaches the surface, sample from  GPS receiver again and update all state values

% 10) Repeat Cycle steps 1-9

%d_batt = tan(theta_climb)*z_batt - ((rho_s - rho_oil)*sigma_v*delta_PH*d_b)/(sigma_m*m_0);

%% Glider Simulation
% FROM TJI AUV_SIM.m CONTROLLER SUITE
%[Etadot] = CONTROL(0,state_vec,gains,gyro,auv,params,d,loop);  % Debug fn. 
%[T_OUT, Y_OUT] = ode45(@CONTROL, [0 loop.cycleT], state_vec, [], gains, gyro, auv, params, d, loop); 

%% Results