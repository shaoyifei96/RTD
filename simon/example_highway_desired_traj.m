clear
%% description
%
% Create and plot an example desired and braking trajectory for the rover.
%
% Author: Sean Vaskov
% Created: 06 March 2020
%
%% user parameters
t_f = 3;
w_0 = -0.2 ; % rad/s
psi_end = -0.2;
v_des = 20 ; % m/s
% t_plan = 0.5; %s
% t_stop = 6; %s

A = highway_cruising_6_state_agent;
%% automated from here
[T,U,Z] = make_highway_desired_trajectory(t_f,w_0,psi_end,v_des) ;
% [Tbrk,Ubrk,Zbrk] = make_highway_braking_trajectory(t_plan,t_f,t_stop,w_0,psi_end,v_des);
%x y h u v r
A.reset([0;0;0;v_des;0;w_0]);
% get the states of the trajectory
x = Z(1,:) ;
y = Z(2,:) ;
psi = Z(3,:) ;
figure(1); clf; hold on;axis equal
A.move(t_f,T,U,Z);
plot(Z(1,:),Z(2,:),'Color','k');
A.plot();
yline(-2);yline(2)
return
% v = Z(4,:) ;
% delta =  Z(5,:) ;

% get the states of the braking trajectory
% xbrk = Zbrk(1,:) ;
% ybrk = Zbrk(2,:) ;
% psibrk = Zbrk(3,:) ;
% vbrk = Zbrk(4,:) ;
% deltabrk =  Zbrk(5,:) ;

% plot
figure(1) ;
set(gca,'FontSize',15)
subplot(2,2,1)
plot(x,y,'b','LineWidth',1.5)
hold on
plot(xbrk,ybrk,'r--','LineWidth',1.5)
xlabel('x [m]')
ylabel('y [m]')


subplot(2,2,2)
plot(T,psi,'b','LineWidth',1.5)
hold on
plot(T,psibrk,'r--','LineWidth',1.5)
xlabel('t [s]')
ylabel('\psi [rad]')

subplot(2,2,3)
plot(T,v,'b','LineWidth',1.5)
hold on 
plot(T,vbrk,'r--','LineWidth',1.5)
xlabel('t [s]')
ylabel('v [m/s]')

subplot(2,2,4)
plot(T,delta,'b','LineWidth',1.5)
hold on
plot(T,deltabrk,'r--','LineWidth',1.5)
xlabel('t [s]')
ylabel('\delta [rad]')
legend({'desired traj.','braking traj.'})

