function [] = verify_g_bound(A,g_x_coeffs, g_y_coeffs, vbls)

load_const
lr = A.lr;
t_f = vbls.t_f; 

x_err = []; y_err= [];
for u0 = linspace(vbls.u0_min, vbls.u0_max, 3)
    for v0 = linspace(vbls.v0_min, vbls.v0_max, 1)
        
        r0 = v0/(lr-Kvy*u0^2);
        z0= [0; 0; 0; u0; v0; r0];        
        
        for u = u0 % mode 2 chosen in compute_xy_tracking_error_highway, so i emulated that here  
            for p = linspace(vbls.p_end_min, vbls.p_end_max, 1)
                for w = linspace(vbls.w_min, vbls.w_max, 3)
                    
                    
                    [T_ref,U_ref,Z_ref] = make_highway_desired_trajectory(t_f,w,p,u) ;
                    A.reset(z0)
                    A.move(t_f,T_ref,U_ref,Z_ref) ;
                    
                    T = A.time ;
                    X = A.state(A.position_indices,:) ;                    
                    
                    X_des = Z_ref(1:2,:) ;
                    X = match_trajectories(T_ref,T,X) ;
                
                    % compute the tracking error
                    pos_err = abs( X - X_des );
                                    
                    % collect the data
                    x_err = [x_err ; pos_err(1,:)] ;
                    y_err = [y_err ; pos_err(2,:)] ;                    
                    
           
                end
            end
        end
    end
end

int_g_x_coeffs = polyint(g_x_coeffs);
int_g_y_coeffs = polyint(g_y_coeffs); 
int_g_x_val = polyval(int_g_x_coeffs, T_ref);
int_g_y_val = polyval(int_g_y_coeffs, T_ref); 


fh1 = figure(1); set(0,'CurrentFigure',fh1); clf; 
subplot(2,1,1); 
plot(T_ref,x_err,'k--'); hold on; 
plot(T_ref, int_g_x_val, 'r'); 
xlabel('time'); 
ylabel('x error (m)') ; 

subplot(2,1,2); 
plot(T_ref,y_err,'k--'); hold on; 
plot(T_ref, int_g_y_val, 'r'); 
xlabel('time'); 
ylabel('y error (m)'); 


end 


