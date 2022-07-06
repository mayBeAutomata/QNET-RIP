% State Space Representation
%A = eye(4,4);
%B = [0;0;0;1];
%C = eye(2,4);
%D = zeros(2,1);

% Rotary pendulum linear state-space matrix C
C = eye(4, 4);
% Rotary pendulum linear state-space matrix D
D = zeros(4, 1);

% Add actuator dynamics

%A(3,3) = A(3,3) - Kg^2*kt*km/Rm*B(3);
%A(4,3) = A(4,3) - Kg^2*kt*km/Rm*B(4);
%B = Kg * kt * B / Rm;



% State Space Representation
Jt = Jr*Jp + Mp*(Lp/2)^2*Jr + Jp*Mp*Lr^2;

A = [0 0 1 0;
    0 0 0 1;
    0 Mp^2*(Lp/2)^2*Lr*g/Jt -Dr*(Jp+Mp*(Lp/2)^2)/Jt -Mp*(Lp/2)*Lr*Dp/Jt;
    0 Mp*g*(Lp/2)*(Jr+Mp*Lr^2)/Jt -Mp*(Lp/2)*Lr*Dr/Jt -Dp*(Jr+Mp*Lr^2)/Jt];

B = [0;
    0;
    (Jp+Mp*(Lp/2)^2)/Jt;
    Mp*(Lp/2)*Lr/Jt];



% Add actuator dynamics
A(3,3) = A(3,3) - Kg^2*kt*km/Rm*B(3);
A(4,3) = A(4,3) - Kg^2*kt*km/Rm*B(4);
B = Kg * kt * B / Rm;




Mp = 0.027;        % Mp       Mass of pendulum assembly(l+w)                (kg)
lp = 0.153;        % lp       Center of mass of pendulum assembly(l+w)       (m)
r = 0.0826;        %r         Length from motor shaft to pendulum pivot      (m)
%Jp = 0.000698;     % Jp       Pendulum moment of inertia                    (kg.m^2)
Jp = 0.000177;
%Jeq = 0.000368;    %Jeq       Moment of inertia acting on the DC motor shaft (kg.m^2)
Jeq = 0.000183;
Bp = 0;            % Bp       Viscous damping coefficient at pendulum axis    (N.m.s/rad)
Beq = 0;           % Beq      Viscous damping acting on the DC motor shaft    (N.m.s/rad)
Kt = 0.0333;       % Kt       DC motor current-torque constant                (N.m/A)
Km = 0.0333;       % Km       DC motor back-emf constant                      (V.s/rad)
Rm = 8.7;          %Rm        Electrical resistance of the DC motor armature  (Ohms)
g = 9.81;          % g        Gravitational constant                          (m/s^2)


% Rotary pendulum linear state-space matrix A
A = [0 0 1 0;
    0 0 0 1;
    0 (Mp^2*lp^2*r*g/(Jeq*Jp+Jeq*Mp*lp^2+Mp*r^2*Jp)) (-(Jp*Kt*Km+Mp*lp^2*Kt*Km)/Rm/(Jeq*Jp+Jeq*Mp*lp^2+Mp*r^2*Jp)) 0;
    0 (Mp*lp*g*(Jeq+Mp*r^2)/(Jeq*Jp+Jeq*Mp*lp^2+Mp*r^2*Jp)) (-Mp*lp*r*(Kt*Km)/Rm/(Jeq*Jp+Jeq*Mp*lp^2+Mp*r^2*Jp)) 0];



% Rotary pendulum linear state-space matrix B
B = [0; 0; (Kt*(Jp+Mp*lp^2)/Rm/(Jeq*Jp+Jeq*Mp*lp^2+Mp*r^2*Jp)); (Mp*lp*Kt*r/Rm/(Jeq*Jp+Jeq*Mp*lp^2+Mp*r^2*Jp))];


% Rotary pendulum linear state-space matrix C
C = eye(2, 4);

% Rotary pendulum linear state-space matrix D
D = zeros(2, 1);


system = ss(A,B,C,D)

T = ctrb(A,B);
% rank(T)
% [m,n] = size(A)

K = [-5.1367, 78.6977, -2.5548, 10.4881];
K = [-5.48, 82.98, -2.60, 11.52];
K= [-5.4772, 75.1573, -2.4587, 10.0174];
K= [-7.0711, 91.1006, -3.1143, 12.4597];

K = [-5.1367, 78.6977, -2.5548, 10.4881];

%K = [-9.4868, 107.2246, -3.9462, 14.7041];


K = [-6, 85, -2.2575, 11.1563];


%% Ctrb

% Dimensions of matrix A
[m, n] = size(A);

% Check stability
poles = eig(A);
if (any(poles>0))
    disp("System is Unstable!")

else
    disp("System is Stable!")
end

%Check controllability
CO = ctrb(A,B);  % is it controllable
if ((rank(CO)) == n)
    disp("System is Controllable!")

else
    disp("System is UnControllable!")
end



%% PolePlacement

% p is a vector of desired eigenvalues
% p = [-.01; -.02; -.03; -.04]; % not enough
% p = [-.3; -.4; -.5; -.6]; % not enough
% p = [-1; -1.1; -1.2; -1.3]; % not enough

% try increasing p
% p = [-2; -2.1; -2.2; -2.3]; % not enough
% p = [-3; -3.1; -3.2; -3.3]; % not enough
% p = [-7; -7.1; -7.2; -7.3]; %  % just barely
% p = [-7.5; -7.6; -7.7; -7.8]; %  % just barely
p = [-8; -8.1; -8.2; -8.3]; %  works
% p = [-8.5; -8.6; -8.7; -8.8]; %  % just barely

%rest break the system
% p = [-9; -9.1; -9.2; -9.3]; %
% p = [-9.5; -9.6; -9.7; -9.8]; %  % agressive


% Find K gain vector
K_pp = place(A,B,p);

% clp
ss_clp_pp = eig(A - (B*K_pp));


disp('K vector in Pole-Placement is : ')
disp(K_pp)
disp('Poles of ClosedLoopSystem(PP) : ')
disp(ss_clp_pp)



%% Linear Quadratic Regulator

Q = [90 0 0 0;
    0 10 0 0;
    0 0 0 0;
    0 0 0 6];

R = 1;

% find gain matrix using LQR
K_lqr = lqr(A,B,Q,R);

% clp
ss_clp_lqr = eig(A - (B*K_lqr));

disp('K vector in LQR is : ')
disp(K_lqr)
disp('Poles of ClosedLoopSystem(LQR) : ')
disp(ss_clp_lqr)



%% PD Gains

%[b,a] = ss2tf(A,B,C,D);

sys = ss(A,B,C,D);

G = tf(sys);



%% AK_Test

% control specifications
zeta = 0.7;
wn = 4;

% location of dominant poles along real-axis
sigma = zeta*wn;

% location of dominant poles along img axis (damped natural freqency)
wd = wn*sqrt(1-zeta^2);

% desired poles (-30 and -40 are given)
DP = [-sigma+j*wd, -sigma-j*wd, -30, -40];

% find control gain using Matlab pole-placement command
K_ack = acker(A,B,DP);



%% Set "K"

K = K_pp;
K = K_lqr;