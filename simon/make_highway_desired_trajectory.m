function [T,U,Z] = make_highway_desired_trajectory(t_f,w_0,psi_end,u_des)
% [T,U,Z] = make_rover_desired_trajectory(t_f,w_0,psi_end,v_des)
%
% Create a Lange Change as a full-state trajectory for the highway.
%
% The inputs are:
%   t_f      planning time horizon
%   w_0      desired_initial_yaw rate
%   psi_end  desired heading at t_f
%   v_des    desired_speed
%
% The outputs are:
%   T        timing for desired trajectory as a 1-by-N array
%   U        desired input (velocity command and wheel angle) as 2-by-N array
%   Z        desired trajectory (x,y,h,v,delta) as a 5-by-N array
%
% Author: Sean Vaskov
% Created: 06 March 2020
    % distance from the front and rear wheels to center of mass
    % car dimension is 4.8 x 2.0, put CoM much closer to the rare 
    load_const
%      l = lf + lr;
    
    % set up timing
    t_sample = 0.01 ;
    T = unique([0:t_sample:t_f,t_f]);
    N_t = length(T) ;
    
    % get inputs for desired trajectories
    w_slope = -2*(t_f*w_0-psi_end)/t_f^2;
    
    w_traj = w_0+w_slope*T;
    
    u_traj = u_des*ones(1,N_t) ;
    if u_des == 0
        w_traj = zeros(1,N_t);
    end
    v_traj = w_traj .* (lr-Kvy*u_traj.^2);
    
%     if u_des~=0
%         wheelangle_traj = atan(l*w_traj./u_traj);
%     else
%         wheelangle_traj = zeros(1,N_t);
%     end
    
    % compute desired trajectory
    z0 = zeros(3,1) ;
    k = [w_0;psi_end;u_des];
    
    [~,Z] = ode45(@(t,z) highway_trajectory_producing_model(t,z,k,t_f),T,z0) ;

    % append velocity and wheelangle to (x,y,h) trajectory to make it a full-state
    % trajectory for the rover
    Z = [Z' ; u_traj;v_traj;w_traj] ;
    
    % compute inputs for robot 
    %old: ud deltad
    %new:(ud vd rd uddot vddot )
    uddot = [diff(u_traj) 0]/t_sample;
    vddot = [diff(v_traj) 0]/t_sample;
    U = [u_traj;v_traj;w_traj;uddot;vddot] ;
    
end