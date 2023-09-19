
%Inputs to the NON_LINEAR_AicraftModel simulink
clear;clc
%------Gravity------
g=9.81;

%Mass Properties
mass=4;
Ix =0.6465;
Iy =0.3048;
Iz =0.9359;
Ixz=0.0006584;

%-----------Trim Conditions-------------%
Phi0=0;            %in rad
Theta0=5.1*pi/180; %in rad
Epsi0=0;           %in rad
V0=18;             %resultant
u0=sqrt(V0^2/(1+(tan(Theta0))^2));
v0=0;
w0=u0*tan(Theta0);
p0=0;
q0=0;
r0=0;
d_elevator0=-0.6319*pi/180;
d_Thrust0=5;       %0.8749; % 0.8749
d_rudder0=0;
d_aileron0=0;
w_d0=0;
X0=mass*g*sin(Theta0);
Y0=-mass*g*cos(Theta0)*sin(Phi0);
Z0=-mass*g*cos(Theta0)*cos(Phi0);

%-----------Initial Conditions-------------
Phi_initial=Phi0;%+(2*pi/180);
Theta_initial=Theta0+(2*pi/180);
Epsi_initial=Epsi0;%+(2*pi/180);
u_initial=u0;
v_initial=v0;
w_initial=w0;
p_initial=0;
q_initial=0;
r_initial=0;
d_elevatori=d_elevator0;

%initial_position
Xe0=0; 
Ye0=0;
Ze0=120;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% inital value, edit this
Hi=120; %%%%%%fixed desired value

%Airplane Properties
S=0.471;      %Wing Area
b=1.7;        %Wing Span
c=0.28;       %Mean Wing Chord
%Stability derivatives

%---------------- X force--------------------
CX0=-0.0094304;
CXu=-(0.158*0)-0.0094304;
CX_alpha=(0.42565-0.21544);

CXe=0;
CXt=1;

%---------------- Y force--------------------
CYB=-0.16533;
CYp=-0.00068802; %sign change
CYr=0.15502;     %sign Charge

CY_aileron=0.023693;
CY_rudder=-0.10317;   %sign Change12 %%%%%%%whyyyyy

%---------------- Z force--------------------
CZ0=-0.42565;
CZu=-((0.052968^2)/(1-(0.052968)^2))*0.42565;
CZ_alpha=-(4.8375+0.0094304);
CZ_alpha_dot=1.3359;    %lt/c=2.5
CZq=-7.3583;     %-(-9.96/2.5)*2*u0/c;

CZe=-0.56118; 
CZt=0;

%---------------- L Moment--------------------
CLb=0.029196;    %signChange2
CLp=-0.44965;
CLr=0.0067709;

CL_rudder=0.003592;
CL_aileron=0.32935;    %SignChange2

%---------------- M Moment--------------------
CMu=0.158*0;
CM_alpha=-0.1651;
CM_alpha_dot=-6.5034;
CMq=-11.7782;

CMe=-1.3977; 
CMt=0;%0.18;%%%%%%aaaa ddddddddd%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%---------------- N Moment--------------------
CNB=0.066996;    %sign Change
CNp=-0.030754;
CNr=-0.064937;

CN_rudder=-0.045652;
CN_aileron=0.036215;   %SignChange2

%--------------State space-------------------

Q0 = 196.1738447509;

Xu = Q0*S/(mass*u0)*(2*CX0+CXu);
Xw = Q0*S/(mass*u0)*CX_alpha;
%Xe = Q0*S/(mass)*CXe;
Xe = Q0*S*CXe; % edited no differernce%
%Xt = Q0*S/(mass)*CXt;
Xt = Q0*S*CXt; % edited %

Zu = Q0*S/(mass*u0)*(2*CZ0+CZu);
Zw = Q0*S/(mass*u0)*CZ_alpha;
Zw_d = Q0*S*c/(mass*2*u0^2)*CZ_alpha_dot;
Zq = Q0*S*c/(mass*2*u0)*CZq;
Ze = Q0*S/(mass)*CZe; % done without -%
%Zt = Q0*S/(mass)*CZt; 
Zt = Q0*S*CZt; % edited no differernce%

Mu = Q0*S*c/(Iy*u0)*CMu;
Mw = Q0*S*c/(Iy*u0)*CM_alpha;
Mw_d = Q0*S*c^2/(2*Iy*u0^2)*CM_alpha_dot;
Mq = Q0*S*c^2/(2*Iy*u0)*CMq;
Me = Q0*S*c/(Iy)*CMe;
%Mt = Q0*S*c/(Iy)*CMt;
Mt = Q0*S*c*CMt; % edited no difference%

A_long=[ Xu                    Xw              0              -g*cos(Theta0)            0;...
         Zu                    Zw              u0             -g*sin(Theta0)            0;...
         Mu+Mw_d*Zu            Mw+Mw_d*Zw      Mq+Mw_d*u0          0                    0;...
         0                     0               1                   0                    0;...                           
         -sin(Theta0)       -cos(Theta0)        0        u0*cos(Theta0)+w0*sin(Theta0)   0   ];

B_long=[ Xe         CXt;...
         Ze         0;...
         Me         0;...
         0          0;...
         0          0]; 
     
longi = [ u_initial; w_initial; q_initial; Theta_initial; Ze0];
longd=[u0; w0; q0; Theta0; Hi; 0];

 C_long=[0 0 0 1 0];
 D_long=zeros(5,2);
 cc=ones(1,5) ;
 
 %%100*[1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0 ; 0 0 0 100 0; 0 0 0 0 100];
 %%100*[1/(u0+1)^2 0 0 0 0; 0 1/(w0+1)^2 0 0 0; 0 0 1/(0.1+q0)^2 0 0 ; 0 0 0 1/(0.1+Theta0)^2 0; 0 0 0 0 1/(0.5+Hi)^2];
 %%1000*[1/(u0+0)^2 0 0 0 0; 0 1/(w0+0)^2 0 0 0; 0 0 1/(0.1+q0)^2 0 0 ; 0 0 0 100/(0+Theta0)^2 0; 0 0 0 0 1/(0+Hi)^2];
 Q_long= 1000*[1/(u0+0)^2 0 0 0 0; 0 1/(w0+0)^2 0 0 0; 0 0 1/(100+q0)^2 0 0 ; 0 0 0 100/(0+Theta0)^2 0; 0 0 0 0 1/(0+Hi)^2]
 

 R_long=[1000 0;...
         0 1];
 
 K_long=lqr(A_long, B_long,Q_long,R_long);
 
 %%%state space with refrence tracking:
 A_long_hat=[A_long  zeros(5,1) ;...
            -C_long    0 ];

 B_long_hat=[B_long;
             zeros(1,2)];
 C_long_hat=eye(6);
 D_long_hat=zeros(6,2);
 e_longi = longd- [longi; 0];   %[ 0; 0; 0; -2*pi/180; 0; 0];
 
%%%LQR:
%%% u; w; q; Theta; h; error theta
%% 100*[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 10 0 0; 0 0 0 0 10 0; 0 0 0 0 0 100];
 Q_long_hat= 1000*[1/(u0+0.1)^2 0 0 0 0 0; 0 1/(w0+0.1)^2 0 0 0 0; 0 0 1/(1+q0)^2 0 0 0; 0 0 0 10/(0.1+Theta0)^2 0 0; 0 0 0 0 10/(0.1+Hi)^2 0; 0 0 0 0 0 100/(0.01)^2]


 %K_long_reglator=lqr(A_long, B_long,Q_long,R_long);
 K_long_hat=lqr(A_long_hat, B_long_hat,Q_long_hat,R_long);
 K=K_long_hat(:,1:5);
 Ki=-K_long_hat(:,6);
 
%K_long =K_hat_long;
%K_long(:,6) = [];
%Ki_long=K_hat_long(:,6);

%%%Closed loop system
% reglator_sys = ss((A_long - B_long*K_long), B_long, C_long, D_long);
 sys = ss((A_long_hat - B_long_hat* K_long_hat), B_long_hat, C_long_hat, D_long_hat);

%%% simulation
%(1) Simulation without theta disturbance =2 degree with input zero signal 
 t=0:0.1:10;
u_input=[zeros(1,numel(t));zeros(1,numel(t))];
ff=d_elevatori+zeros(1,numel(t));
kk =d_Thrust0+zeros(1,numel(t));
%u_input=[ff;kk];


[Y,T,e]=lsim(sys,u_input,t,e_longi);
%[y,t,x] = initial(sys, x0, t);
%[y,t,e] = step(sys, t);
figure(1)
plot(t,Theta0*180/pi-e(:,4)*180/pi,'r',t,Theta0*180/pi+zeros(1,numel(t)),'b')
title('Pitch angel Response with theta disturbance =2 degree and zero inputs')
xlabel('time') 
legend('theta','theta0')

figure(2)
plot(t,Ze0-e(:,5),'r',t,Ze0+zeros(1,numel(t)),'b')
title('H Response theta disturbance =2 degree and zero inputs')
xlabel('time') 
legend('h','h0')

figure(3)
plot(t,u0-e(:,1),'r',t,u0+zeros(1,numel(t)),'b')
title('u Response theta disturbance =2 degree and zero inputs')
xlabel('time') 
legend('u','u0')







%%-------------- Lateral LQR------------------

lati = [ v_initial; p_initial; r_initial; Phi_initial; Epsi_initial];
latd=[v0; p0; r0; Phi0; Epsi0; 0];

YB  = Q0*S*CYB/mass;
Yp  = Q0*S*b*CYp/(2*mass*u0);
Yr  = Q0*S*b*CYr/(2*mass*u0);
Yda = Q0*S*CY_aileron/mass;
Ydr = Q0*S*CY_rudder/mass;

NB  = Q0*S*b*CNB/Iz;
Np  = Q0*S*b^2*CNp/(2*Ix*u0);%%%%
Nr  = Q0*S*b^2*CNr/(2*Ix*u0);
Nda = Q0*S*b*CN_aileron/Iz;
Ndr = Q0*S*b*CN_rudder/Iz;

LB  = Q0*S*b*CLb/Ix;
%Lv= Q0*S*b*CLb/(Iz*u0);
Lp  = Q0*S*b^2*CLp/(2*Ix*u0);%%%%
Lr  = Q0*S*b^2*CLr/(2*Ix*u0);
Lda = Q0*S*b*CL_aileron/Ix;
Ldr = Q0*S*b*CL_rudder/Ix;

dem= 1-Ixz^2/(Ix*Iz);
LB_star= LB/dem;
%Lv_star=Lv/dem;
Lp_star= Lp/dem;
Lr_star= Lr/dem;
Lda_star= Lda/dem;
Ldr_star= Ldr/dem;

NB_star= NB/dem;
Np_star= Np/dem;
Nr_star= Nr/dem;
Nda_star= Nda/dem;
Ndr_star= Ndr/dem;

A21 = LB_star + Ixz/Ix*NB_star;
A22 = Lp_star + Ixz/Ix*Np_star;
A23 = Lr_star + Ixz/Ix*Nr_star;
A31 = NB_star + Ixz/Iz*LB_star;
A32 = Np_star + Ixz/Iz*Lp_star;
A33 = Nr_star + Ixz/Iz*Lr_star;

% states[ beta,p,r,phi,epsi]
A_lat= [YB        Yp      -(u0-Yr)   g*cos(Theta0)   0;...
        A21       A22       A23        0             0;...
        A31       A32       A33        0             0;...
        0         1         0          0             0;...
        0         0         1          0             0];
    
B21=Lda_star+Ixz/Ix*Nda_star;
B22=Ldr_star+Ixz/Ix*Ndr_star;
B31=Nda_star+Ixz/Iz*Lda_star;
B32=Ndr_star+Ixz/Iz*Ldr_star;

% aileron, rudder
B_lat=[0        Ydr;...
       B21      B22;...
       B31      B32;...
       0        0;...
       0        0];
 C_lat=[0 0 0 1 0 ];
 D_lat=zeros(5,2);
 
 Q_lat=2*[1 0 0 0 0;...
        0 1 0 0 0;...
        0 0 1 0 0;...
        0 0 0 1 0;...
        0 0 0 0 1];
  R_lat=[1 0;...
         0 1];
  K_lat=lqr(A_lat, B_lat,Q_lat,R_lat);
  
%%%state space with refrence tracking:
  A_lat_hat=[A_lat  zeros(5,1) ;...
             -C_lat    0 ];

 B_lat_hat=[B_lat;
             zeros(1,2)];
 C_lat_hat=eye(6);
 D_lat_hat=zeros(6,2);
 e_lati = latd-[lati;0]; %[ 0; 0; 0; -2*pi/180; 0; 0];

% %%%LQR:
 Q_lat_hat=[1 0 0 0  0    0;...
         0 1 0 0  0    0;...
         0 0 1 0  0    0;...
         0 0 0 10 0    0;...
         0 0 0 0  10   0;...
         0 0 0 0  0  100];


 %K_lat_reglator=lqr(A_lat, B_lat,Q_lat,R_lat);
 K_lat_hat=lqr(A_lat_hat, B_lat_hat,Q_lat_hat,R_lat);

 %%%Closed loop system
 % reglator_sys = ss((A_lat - B_lat*K_lat), B_lat, C_lat, D_lat);
  sys_lat = ss((A_lat_hat - B_lat_hat* K_lat_hat), B_lat_hat, C_lat_hat, D_lat_hat);


% %%% simulation
% %(1) Simulation without theta disturbance =2 degree with input zero signal 
 t_lat=0:0.1:10;
 u_lat_input=[zeros(1,numel(t_lat));zeros(1,numel(t_lat))];
%u_lat_input=[d_elevatori +zeros(1,numel(t_lat));d_Thrust0+zeros(1,numel(t_lat))];

[Y_lat,T_lat,e_lat]=lsim(sys_lat,u_lat_input,t,e_lati);
%[y,t,x] = initial(sys, x0, t);
%[y,t,e] = step(sys, t);
%figure(4)
%plot(t,Phi0-e_lat(:,4)*180/pi,'r',t,Phi0*180/pi+zeros(1,numel(t_lat)),'b')
%title('Roll angel Response with phi disturbance =2 degree and zero inputs')
%xlabel('time') 
%legend('Phi','Phi0')

%figure(5)
%plot(t,Epsi0-e_lat(:,5),'r',t,Epsi0+zeros(1,numel(t_lat)),'b')
%title('Epsi Response with Roll disturbance =2 degree and zero inputs')
%xlabel('time') 
%legend('Epsi','Epsi0')



%% Plotting responses for sideslip angle
% % Run response to initial condition
% lati=[v_initial; p_initial; r_initial; Phi_initial; Epsi_initial];
% [y,t,x] = initial(sys_lat, lati, 400);
% figure;
% plot(t,y(:,1));
% title('System Response to intial conditions');
% 
% % Run response after controller
% [y,t,x] = initial(sys_lat_C, lati, 400);
% figure;
% plot(t,y(:,1));
% title('System Response to intial conditions with LQR Controller');

