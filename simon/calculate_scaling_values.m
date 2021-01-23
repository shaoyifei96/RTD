function [scaling] = calculate_scaling_values(A, g_x_coeffs, g_y_coeffs, vbls)


%% todo: add disturbances loop 
%% 

t_data = 0:0.1:vbls.t_f; 

for u0 = linspace(vbls.u0_min, vbls.u0_max, 3)
    for v0 = linspace(vbls.v0_min, vbls.v0_max, 1) 
        for u = u0 % mode 2 chosen in compute_xy_tracking_error_highway, so i emulated that here  
            for p = linspace(vbls.p_end_min, vbls.p_end_max, 1)
                for w = linspace(vbls.w_min, vbls.w_max, 3)
                 
                    k = [w; p; u];
                    
                    x_traj = []; y_traj = []; p_traj = [];
                    for d = [-1 1] 

                        [tout, zout] = ode45( @(t,z) planning_model(t,z,vbls,k,g_x_coeffs,g_y_coeffs,d), [0, vbls.t_f], [0;0;0]); 
                        
                        x_vec = interp1(tout',zout(:,1)',t_data);
                        y_vec = interp1(tout',zout(:,1)',t_data);
                        p_vec = interp1(tout',zout(:,1)',t_data);
                        
                        x_traj = [x_traj; x_vec]; 
                        y_traj = [y_traj; y_vec]; 
                        p_traj = [p_traj; p_vec]; 

                    end

                    %TODO: get necessary min and max vals here, calculate
                    %scaling 
                    x_max = max(zout(:,1)'); 
                    x_min = min(zout(:,
                    y_max = max(zout(:,2)'); 
                    p_max = max(zout(:,3)');   
                    
                    % TODO: store max scaling value using if statement 
                end
            end
        end
    end
end

% output final scaling values 
% code in option to save for future use 



                    fh2 = figure(2); set(0,'CurrentFigure', fh2); clf 
                    plot(tout', zout(:,1)'); hold on ;
                    plot(tout', zout(:,2)'); 

end
