function [scaling, zoffset] = calculate_scaling_values(A, g_x_coeffs, g_y_coeffs, vbls,scale_value)


%% todo: add disturbances loop
%%

t_data = 0:0.1:vbls.t_f;

x_scale = 0; y_scale = 0; p_scale = 0;
z_range = zeros(3,2);
z_range(:,1) = inf;
z_range(:,2) = -inf;

for u0 = linspace(vbls.u0_min, vbls.u0_max, 3)
    %     for v0 = linspace(vbls.v0_min, vbls.v0_max, 3)
    for u = u0 % mode 2 chosen in compute_xy_tracking_error_highway, so i emulated that here
        for p = linspace(vbls.p_end_min, vbls.p_end_max, 3)
            for w = linspace(vbls.w_min, vbls.w_max, 3)
                
                k = [w; p; u];
                
                x_traj = []; y_traj = []; p_traj = [];
%                 clf; hold on;
                for d1 = linspace(-1,1,3)
                for d2 = linspace(-1,1,3)
                    
                    [tout, zout] = ode45( @(t,z) planning_model(t,z,vbls,k,g_x_coeffs,g_y_coeffs,[d1 d2]), [0, vbls.t_f], [0;0;0]);
                    %t,z,vbls,k,g_x_coeffs,g_y_coeffs,d
                    [T_ref,U_ref,Z_ref] = make_highway_desired_trajectory(vbls.t_f,w,p,u) ;
                    z0= [0; 0; 0; u0; randRange(vbls.v0_min,vbls.v0_max); w];
                    A.reset(z0);
                    A.move(vbls.t_f,T_ref,U_ref,Z_ref);
                    plot(A.state(1,:),A.state(2,:),'Color','r');

                    x_vec = interp1(tout',zout(:,1)',t_data);
                    y_vec = interp1(tout',zout(:,2)',t_data);
                    p_vec = interp1(tout',zout(:,3)',t_data);
                    scatter(x_vec, y_vec,5,'c');
                    x_traj = [x_traj; x_vec];
                    y_traj = [y_traj; y_vec];
                    p_traj = [p_traj; p_vec];
                end  
                end
                
                
                x_max = max(x_traj,[],'all');
                x_min = min(x_traj,[],'all');
                y_max = max(y_traj,[],'all');
                y_min = min(y_traj,[],'all');
                p_max = max(p_traj,[],'all');
                p_min = min(p_traj,[],'all');
                if x_min < z_range(1,1)
                    z_range(1,1) = x_min;
                end
                if y_min < z_range(2,1)
                    z_range(2,1) = y_min;
                end
                if p_min < z_range(3,1)
                    z_range(3,1) = p_min;
                end
                if x_max > z_range(1,2)
                    z_range(1,2) = x_max;
                end
                if y_max  > z_range(2,2)
                    z_range(2,2) = y_max ;
                end
                if p_max  > z_range(3,2)
                    z_range(3,2) = p_max ;
                end
                
                
                
            end
        end
        %         end
    end
end

% scaling.x = x_scale ;
% scaling.y = y_scale ;
% scaling.p = p_scale ;
% scaling.t = t_data(end);
% TODO: store max scaling value using if statement
scaling = (z_range(:,2) - z_range(:,1))/scale_value/2;
zoffset = -(z_range(:,2)+z_range(:,1))/2;


end
