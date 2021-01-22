%% description
close all
clear
%
% This script computes the tracking error function "g" for the rover.
% See the paper "Bridging the Gap Between Safety and Real-Time Performance
% in Receding-Horizon Trajectory Design for Mobile Robots" for an
% explanation of the error function in Section 2.2.2. In particular, see
% Assumption 10 that defines the tracking error function.
%
% The paper is available here: https://arxiv.org/abs/1809.06746
%
% Author: Sean Vaskov
% Created: 24 March 2020
g_degree_x = 3;
g_degree_y = 3;

% number of samples in v0, w, and v
N_samples = 6;

% timing
t_sample = 0.01 ;

plotting = true;
save_data_flag = 1;
%% user parameters
% initial condition bounds (recall that the state is (x,y,h,v), but the
% robot's dynamics in SE(2) are position/translation invariant)
%time horizon
T = 6;

%velocity initial and desired bounds
u0_min = 5 ; % m/s
u0_max = 15; % m/s I

u_des_min = 5;
u_des_max = 15; % D

v0_min = -1 ; % m/s
v0_max = 1; % m/s %I


%script will loopthrough all of these combos, fit error functions for each
%then save as separate files

% delta0_combos = [-0.5, -0.3,  -0.15, -0.05, 0.05, 0.15, 0.3;...
%                  -0.3, -0.15, -0.05,  0.05, 0.15, 0.3,  0.5];


% command bounds
w0_des_min = -0.1; %D
w0_des_max =  0.1;

psi_end_min = 0; %rad  %D
psi_end_max = 0.5; %rad


load_const


%% automated from here

% for dcom = delta0_combos
%
% delta0_min = dcom(1);
% delta0_max = dcom(2);

% create roveragent
% RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
%dont need llc, agent is closed looped
A = highway_cruising_6_state_agent ;
A.integrator_type= 'ode45';     %x  y  h  u   v  r
% A. desired_initial_condition = [0; 0; 0; 10; 0; 0];

l = A.lr+ A.lf;
lr = A.lr;

% create initial condition vector for velocity
u0_vec = linspace(u0_min,u0_max,N_samples) ; %initial u
v0_vec = [0];%linspace(v0_min,v0_max,N_samples) ; %inital v

% create initial condition vector for wheelangle
% delta0_vec = linspace(delta0_min,delta0_max,N_samples) ;

% create psi0 commands
psiend_vec = [0];%linspace(psi_end_min,psi_end_max,N_samples);


% load timing
try
    disp('Loading r RTD planner timing info.')
    timing_info = load('.mat') ;
    t_plan = timing_info.t_plan ;
    t_stop = timing_info.t_stop ;
    t_f = timing_info.t_f ;
catch
    disp('Could not find timing MAT file. Setting defaults!')
    t_plan = tpk;
    t_stop = tpk + tbrk;
    t_f =  tpk + tbrk;
end

% initialize time vectors for saving tracking error; we use two to separate
% out the braking portion of the trajectory
T_data = unique([0:t_sample:T,T]) ;

% initialize arrays for saving tracking error; note there will be one row
% for every (v0,w_des,v_des) combination, so there are N_samples^3 rows
N_total = N_samples^5;

e_x_max_data = nan(N_total,length(T_data)) ;
e_x_min_data = nan(N_total,length(T_data)) ;
e_y_max_data = nan(N_total,length(T_data)) ;
e_y_min_data = nan(N_total,length(T_data)) ;

e_dxdt_max_data = nan(N_total,length(T_data)) ;
e_dxdt_min_data = nan(N_total,length(T_data)) ;
e_dydt_max_data = nan(N_total,length(T_data)) ;
e_dydt_min_data = nan(N_total,length(T_data)) ;

%parameters we want g to be a function of
psi_ref_data = nan(N_total,length(T_data));
v_des_data = nan(N_total,length(T_data));
w0_des_data = nan(N_total,length(T_data));
psi_end_data = nan(N_total,length(T_data));


%% tracking error computation loop
err_idx = 1 ;

tic
u_vec = linspace(u_des_min,u_des_max, N_samples) ;
% v0_vec = linspace(v_des_min,v_des_max, N_samples) ;
x_err = [];y_err= [];
%inital paramters: u0 v0
%desired parameters: u_des psi_des w0_des
% for each initial condition...
for u0 = u0_vec %  initial
    for v0 = v0_vec %initial
        
        % for each yaw and speed command...
        for u_des = u_vec %command
            if abs(u0-u_des) > 5
                continue;
            end
            %psi_end is always 0 since it alawys starts at heading 0 and
            %end with heading=0;
            for psi_end = psiend_vec % psiend_vec % two desired, psi always end with heading forward! negative of inital heading
                %still need a bunch of values for rotation
                % create the initial condition
                %                 z0 = [0;0;0;u0;delta0] ; % (x,y,h,v,delta)
                r0 = v0/(lr-Kvy*u0^2);
                %x  y  h  u   v  r
                z0= [0; 0; 0; u0; v0; r0];
                
                
                %create feasible initial yawrate commands from initial heading
                %                 w0_des_min_temp = max(w0_des_min, 1/0.5*psi_end-1);
                %                 w0_des_max_temp = min(w0_des_max, 1/0.5*psi_end+1);
                
                %                 w0_des_vec = linspace(w0_des_min_temp,w0_des_max_temp, N_samples-1);
                if u_des == 5
                    w0_des_vec = linspace(-0.1,0.1,N_samples-1);
                elseif u_des == 7
                    w0_des_vec = linspace(-0.08,0.08,N_samples-1);
                elseif u_des == 9
                    w0_des_vec = linspace(-0.06,0.06,N_samples-1);
                elseif u_des == 11
                    w0_des_vec = linspace(-0.055,0.055,N_samples-1);
                elseif u_des == 13
                    w0_des_vec = linspace(-0.05,0.05,N_samples-1);
                elseif u_des == 15
                    w0_des_vec = linspace(-0.05,0.05,N_samples-1);
                end
                for w0_des = w0_des_vec
                    
                    % create the desired trajectory
                    [T_ref,U_ref,Z_ref] = make_highway_desired_trajectory(t_f,w0_des,psi_end,u_des) ;
                    
                    % reset the robot
                    
                    A.reset(z0)
                    
                    % track the desired trajectory
                    A.move(t_f,T_ref,U_ref,Z_ref) ;
                    
%                     figure(1);clf; hold on;
%                     plot(A.state(1,:),A.state(2,:))
%                     plot(Z_ref(1,:),Z_ref(2,:))
                    %                     plot(U_ref')
                    % compute the error before t_plan
                    T = A.time ;
                    X = A.state(A.position_indices,:) ;
                
                    % interpolate the desired and realized trajectory to match
                    X_des = Z_ref(1:2,:) ;
                    X = match_trajectories(T_ref,T,X) ;
                
                    % compute the tracking error
                    pos_err = X - X_des ;
                
                    % collect the data
                    x_err = [x_err ; pos_err(1,:)] ;
                    y_err = [y_err ; pos_err(2,:)] ;
                    
                    
                    if mod(size(y_err,1),10) == 0
                        disp(['Iteration ',num2str(size(y_err,1)),' out of ',num2str(length(u0_vec)*length(v0_vec)*length(u_vec)*length(psiend_vec)*length(w0_des_vec))])
                    end
                end
            end
            
        end
    end
end
toc

%% fit tracking error function
% get max of absolute tracking error
x_err = abs(x_err) ;
y_err = abs(y_err) ;
x_max = max(x_err,[],1) ;
y_max = max(y_err,[],1) ;

% fit polynomial to the data
int_g_x_coeffs = polyfit(T_ref,x_max,g_degree_x) ;
int_g_y_coeffs = polyfit(T_ref,y_max,g_degree_y) ;

% take the time derivative of these to get the g functions in x and y
g_x_coeffs = polyder(int_g_x_coeffs) ;
g_y_coeffs = polyder(int_g_y_coeffs) ;

%% correct the fit to make it greater than the data
% evaluate g
int_g_x_coeffs = polyint(g_x_coeffs) ;
int_g_y_coeffs = polyint(g_y_coeffs) ;
int_g_x_vals = polyval(int_g_x_coeffs,T_ref) ;
int_g_y_vals = polyval(int_g_y_coeffs,T_ref) ;

% figure out the maximum ratio of the error data to the int g values
r_x_err = x_max ./ int_g_x_vals ;
r_x_max = max([1,r_x_err]) ;
r_y_err = y_max ./ int_g_y_vals ;
r_y_max = max([1,r_y_err]) ;

% multiply the g_x and g_y coefficients by the error data ratio
g_x_coeffs = r_x_max .* g_x_coeffs ;
g_y_coeffs = r_y_max .* g_y_coeffs ;

% re-integrate g with the new coefficients
int_g_x_coeffs = polyint(g_x_coeffs) ;
int_g_y_coeffs = polyint(g_y_coeffs) ;
int_g_x_vals = polyval(int_g_x_coeffs,T_ref) ;
int_g_y_vals = polyval(int_g_y_coeffs,T_ref) ;

%% save data
if save_data_flag
    filename = ['highway_error_functions_v_0_',...
        num2str(u0_min,'%0.1f'),'_to_',...
        num2str(u0_max,'%0.1f'),'.mat'] ;
    save(filename,'g_x_coeffs','g_y_coeffs') ;
end

%% plotting
figure(1) ; clf ;

% plot x error
subplot(2,1,1) ; hold on ;
plot(T_ref,x_err','k--')
g_x_handle =  plot(T_ref,int_g_x_vals,'r-','LineWidth',1.5) ;
title('tracking error vs. time')
ylabel('x error [m]')
legend(g_x_handle,'\int g_x(t) dt','Location','NorthWest')
set(gca,'FontSize',15)

% plot y error
subplot(2,1,2) ; hold on ;
plot(T_ref,y_err','k--')
g_y_handle = plot(T_ref,int_g_y_vals,'r-','LineWidth',1.5) ;
xlabel('time [s]')
ylabel('y error [m]')
legend(g_y_handle,'\int g_y(t) dt','Location','NorthWest')
set(gca,'FontSize',15)
