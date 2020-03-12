%% description
% This script computes a Forward-Reachable Set (FRS) for the Rover. The
% user specifies the range of initial speeds; all other info is loaded from
% the relevant .mat files.
%
% Author: Sean Vaskov
% Created: 08 March 2020
%
clear ; clc ; close all ;
%% user parameters
% degree of SOS polynomial solution
degree = 4 ; % this should be 4 or 6 unless you have like 100+ GB of RAM

% include tracking error or not (this slows down the computation)
include_tracking_error = true;

% speed range (uncomment one of the following)
v_0_range = [1.0, 2.0] ;

% whether or not to save output
save_result = true;

%% automated from here
% load timing
load('rover_timing.mat')

% load the error functions and distance scales
switch v_0_range(1)
    case 1.0
        load('rover_pos_error_functions_v0_1.0_to_2.0.mat')
        load('rover_FRS_scaling_v0_1.0_to_2.0.mat')
    otherwise
        error('Hey! You picked an invalid speed range for the RTD tutorial!')
end

% create agent to use for footprint
A = RoverAWD ;


%% set up the FRS computation variables and dynamics
% set up the indeterminates
t = msspoly('t', 1) ; % time t \in T
z = msspoly('z', 3) ; % state z = (x,y) \in Z
k = msspoly('k', 3) ; % parameters k \in K

%unscaled states
x = zscale(1)*z(1)-zoffset(1);
y = zscale(2)*z(2)-zoffset(2);
psi = zscale(3)*z(3)-zoffset(3);

%unscaled parameters
w0_des =   (w0_des_max-w0_des_min)/2*(k(1)+1)+w0_des_min;
psi_end =  (psi_end_max-psi_end_min)/2*(k(2)+1)+psi_end_min;
v_des =    (v_des_max-v_des_min)/2*(k(3)+1)+v_des_min;


% create polynomials that are positive on Z, and K, thereby
% defining them as semi-algebraic sets; h_T is automatically generated
hK = [(k+1).*(1-k);...
         w0_des-(-w0_des_min/psi_end_max*psi_end+w0_des_min);...
         (w0_des_max/-psi_end_min*psi_end+w0_des_max) - w0_des];
     

hZ = (z+1).*(1-z);

hZ0 = msspoly(zeros(2,1));
hZ0(1) = -x^2-psi^2;
hZ0(2) = -y^2-psi^2;

%% specify dynamics and error function
cos_psi = 1-psi^2/2;
sin_psi = psi-psi^3/6;

% create dynamics
scale = (t_f./zscale) ;

w_slope =  -2*(t_f*w0_des-psi_end)/t_f^2;

w_des = w_slope*t_f*t+w0_des;

f = scale.*[v_des*cos_psi-A.rear_axel_to_center_of_mass*w_des*sin_psi;...
            v_des*sin_psi+A.rear_axel_to_center_of_mass*w_des*cos_psi;...
            w_des] ;

% create tracking error dynamics; first, make msspoly functions for the
% velocity errors
g_v_cos = subs(g_v_cos,[t;z;k],[t_f*t;x;y;psi;w0_des;psi_end;v_des]);
g_v_sin = subs(g_v_sin,[t;z;k],[t_f*t;x;y;psi;w0_des;psi_end;v_des]);

g_vy_cos = subs(g_vy_cos,[t;z;k],[t_f*t;x;y;psi;w0_des;psi_end;v_des]);
g_vy_sin = subs(g_vy_sin,[t;z;k],[t_f*t;x;y;psi;w0_des;psi_end;v_des]);

g = [scale,scale].*[g_v_cos, -g_vy_sin;...
                    g_v_sin, g_vy_cos;...
                     0, 0] ;

%% create cost function
% this time around, we care about the indicator function being on Z x K
int_TZK{1} = boxMoments([t;z(1);k], [0;-ones(4,1)], ones(5,1));
int_TZK{2} = boxMoments([t;z(2);k], [0;-ones(4,1)], ones(5,1));


%% setup the problem structure for x and y
for i = 1:2
solver_input_problem(i).t = t ;
solver_input_problem(i).z = z([i, 3]) ;
solver_input_problem(i).k = k ;
solver_input_problem(i).f = f([i, 3]) ;
solver_input_problem(i).hZ = hZ([i,3]);
solver_input_problem(i).hZ0 = hZ0(i);
solver_input_problem(i).hK = hK ;
solver_input_problem(i).cost = int_TZK{i} ;
solver_input_problem(i).degree = degree ;
solver_input_problem(i).FRS_states = [t;z(i);k];
solver_input_problem(i).hFRS_states = [t*(1-t);1-z(i)^2;hK(1:3)];

if include_tracking_error
    solver_input_problem(i).g = g([i,3],:) ;
end

end

%% compute FRS for x and y positions
for i = 1:2
solver_output(i) = compute_FRS(solver_input_problem(i)) ;
end

%% extract FRS polynomial result
FRS_polynomial_x = solver_output(1).indicator_function ;
FRS_lyapunov_function_x = solver_output(1).lyapunov_function ;

FRS_polynomial_y = solver_output(2).indicator_function ;
FRS_lyapunov_function_y = solver_output(2).lyapunov_function ;


%% save result
if save_result
    % create the filename for saving
    filename = ['rover_FRS_rect_deg_',num2str(degree),'_v0_',...
                num2str(v0_min,'%0.1f'),'_to_',...
                num2str(v0_max,'%0.1f'),'.mat'] ;

    % save output
    disp(['Saving FRS output to file: ',filename])
    save(filename,'FRS_polynomial*','FRS_lyapunov_function*','t','z','k',...
        'f','g','t_plan','degree','solver_input_problem')
end