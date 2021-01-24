clear; 


A = highway_cruising_6_state_agent ;
A.integrator_type= 'ode45';    


%% load g function 
load('highway_error_functions_lane_change.mat'); 


%% check g function as the bound 
verify_g_bound(A, g_x_coeffs, g_y_coeffs, vbls) ; 


%% scaling computations 
scaling = calculate_scaling_values(A, g_x_coeffs, g_y_coeffs, vbls); 


%% compute FRS

t = msspoly('t',1); 
z = msspoly('z',3); 
k = msspoly('k',3); 


%unscaled states
x = scaling.x*z(1);
y = scaling.y*z(2);
%psi = scaling.p*z(3);

%unscaled parameters
w_0 =   (vbls.w_max-vbls.w_min)/2*(k(1)+1)+vbls.w_min;
psi_end =  (vbls.p_end_min-vbls.p_end_max)/2*(k(2)+1)+vbls.p_end_min;
v_des =    (vbls.u_max-vbls.u_min)/2*(k(3)+1)+vbls.u_min;


%% dynamics 
t_f = vbls.t_f; 

w_slope = -2*(t_f*w_0-psi_end)/t_f^2;

w_des = w_0+w_slope*t;

psi = z(3);

cos_psi = 1-psi^2/2;
sin_psi = psi-psi^3/6;


scale_vec = [scaling.t/scaling.x, scaling.t/scaling.y, scaling.t/scaling.p]; 
f = scale_vec*[v_des*cos_psi-A.lr*w_des*sin_psi ;
               v_des*sin_psi+A.lr*w_des*cos_psi ;
               w_des] ;
  
%g_x = subs(g_x,[t;z;k],[t;x;y;psi;w_0;psi_end;v_des]);
%g_y = subs(g_y,[t;z;k],[t;x;y;psi;w_0;psi_end;v_des]);  

g_x = t.^[length(g_x_coeffs)-1:-1:0]*g_x_coeffs';
g_y = t.^[length(g_y_coeffs)-1:-1:0]*g_y_coeffs';

g = scale_vec*[g_x; g_y; 0] ;  
        
%% 
  
hK = [(k(1)+1)*(1-k(1));...
      (k(2)+1)*(1-k(2));...
      (k(3)+1)*(1-k(3))]; 
  
hZ = (z+1).*(1-z);

L = [min(A.footprint_vertices(1,:)), max(A.footprint_vertices(1,:))];
W = [min(A.footprint_vertices(2,:)), max(A.footprint_vertices(2,:))];

hZ0 = [(x-L(1))*(L(2)-x);(y-W(1))*(W(2)-y);-psi^2];
        
int_TZK = boxMoments([t;z;k], [0;-1;-1;-1;-1;-1;-1],[1;1;1;1;1;1;1]);
  
prob = struct; 
prob.t = t ;
prob.z = z ;
prob.k = k ;
prob.cost = int_TZK;
prob.hZ = hZ ; 
prob.hZ0 = hZ0; 
prob.hK = hK; 
prob.f = f; 
prob.g = g ;
prob.degree = 4; 

out = compute_FRS(prob); 