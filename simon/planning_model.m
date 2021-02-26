function [zd] = planning_model(t,z,vbls,k,g_x_coeffs,g_y_coeffs,d)

%extract parameters
w_0 = k(1);
psi_end = k(2);
v_des = k(3);

t_f = vbls.t_f; 

% distance from the ear wheels to center of mass
load_const

w_slope = -2*(t_f*w_0-psi_end)/t_f^2;

w_des = w_0+w_slope*t;

% extract states
psi = z(3);

% taylor approximation for sin and cos
cos_psi = 1-psi^2/2;
sin_psi = psi-psi^3/6;

% g functions 
gx = t.^[length(g_x_coeffs)-1:-1:0]*g_x_coeffs'; 
gy = t.^[length(g_y_coeffs)-1:-1:0]*g_y_coeffs';

% compute dynamics (wheel slip term will be incorporated into g function)
zd = [v_des*cos_psi-lr*w_des*sin_psi + gx*d(1);
      v_des*sin_psi+lr*w_des*cos_psi + gy*d(2);
      w_des] ;



end
