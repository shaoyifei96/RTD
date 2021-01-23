
clear; 
% TODO: insert loop stuff 


A = highway_cruising_6_state_agent ;
A.integrator_type= 'ode45';    


%% load g function 
load('highway_error_functions_lane_change.mat'); 


%% check g function as the bound 
verify_g_bound(A, g_x_coeffs, g_y_coeffs, vbls) ; 


%% scaling computations 



%% compute FRS 