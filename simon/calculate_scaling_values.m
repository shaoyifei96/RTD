function [scaling] = calculate_scaling_values(A, g_x_coeffs, g_y_coeffs, vbls)


%% todo: add disturbances loop 
%% 

t_data = 0:0.1:vbls.t_f; 

x_scale = 0; y_scale = 0; p_scale = 0; 

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
                        y_vec = interp1(tout',zout(:,2)',t_data);
                        p_vec = interp1(tout',zout(:,3)',t_data);
                        
                        x_traj = [x_traj; x_vec]; 
                        y_traj = [y_traj; y_vec]; 
                        p_traj = [p_traj; p_vec]; 

                    end


                    x_max = max(x_traj,[],'all'); 
                    x_min = min(x_traj,[],'all'); 
                    y_max = max(y_traj,[],'all'); 
                    y_min = min(y_traj,[],'all');
                    p_max = max(p_traj,[],'all');   
                    p_min = min(p_traj,[],'all'); 
                    
                    % TODO: store max scaling value using if statement 
                    x_del = abs(x_max - x_min); 
                    y_del = abs(y_max - y_min); 
                    p_del = abs(p_max - p_min); 
                    
                    if x_del > x_scale 
                        x_scale = x_del; 
                    end
                    
                    if y_del > y_scale 
                        y_scale = y_del; 
                    end
                    
                    if p_del > p_scale
                        p_scale = p_del; 
                    end
                    
                end
            end
        end
    end
end

scaling.x = x_scale ; 
scaling.y = y_scale ; 
scaling.p = p_scale ; 
scaling.t = t_data(end); 

end
