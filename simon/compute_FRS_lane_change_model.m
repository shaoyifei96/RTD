clear; 


A = highway_cruising_6_state_agent ;
A.integrator_type= 'ode45';    


%% load g function 
load('highway_error_functions_lane_change.mat'); 


%% check g function as the bound 
verify_g_bound(A, g_x_coeffs, g_y_coeffs, vbls) ; 


%% scaling computations 
[zscaling,zoffset] = calculate_scaling_values(A, g_x_coeffs, g_y_coeffs, vbls); 

maxz = zscaling-zoffset;
minz = -zscaling-zoffset;
Z0range = zeros(3,2);% intial condition

%% with max footprint
L = 4.8; W = 2;
maxx = maxz(1:2)+[L/2;W/2]*sqrt(2);
minx = minz(1:2)-[L/2;W/2]*sqrt(2);

xscale = (maxx-minx)/2;
xoffset = -minx-(maxx-minx)/2;
%% K range
Krange = [0, vbls.w_max;
%            vbls.p_end_min, vbls.p_end_max; % always 0
           vbls.u0_min, vbls.u0_max];
kscale = (Krange(:,2)-Krange(:,1))/2;
koffset = -Krange(:,1)-kscale;
%% compute FRS
%% extra x state for footprint
x = msspoly('x',2);

t = msspoly('t',1); 
z = msspoly('z',3); 
k = msspoly('k',2); 


%unscaled states
tunscaled = vbls.t_f*t;
zunscaled = zscaling.*z-zoffset;
xunscaled = xscale.*x-xoffset;
% kunscaled = kscale.*k-koffset;
%psi = scaling.p*z(3);

%unscaled parameters
w_0     = k(1);w0_uns= (vbls.w_max-vbls.w_min)/2*(k(1)+1)+vbls.w_min;
psi_end = 0; psi_uns=  0;%k(2);psi_uns=  (vbls.p_end_min-vbls.p_end_max)/2*(k(2)+1)+vbls.p_end_min;
v_des   = k(2);v_uns =  (vbls.u_max-vbls.u_min)/2*(k(2)+1)+vbls.u_min;


%% dynamics 
t_f = vbls.t_f; 

w_slope = -2*(t_f*w0_uns-psi_uns)/t_f^2;

w_des = w0_uns+w_slope*tunscaled;

psi = zunscaled(3);

cos_psi = 1-psi^2/2;
sin_psi = psi-psi^3/6;


scale_vec =  t_f./zscaling; 
f = scale_vec.*[v_uns*cos_psi-A.lr*w_des*sin_psi ;
               v_uns*sin_psi+A.lr*w_des*cos_psi ;
               w_des] ;
  
%g_x = subs(g_x,[t;z;k],[t;x;y;psi;w_0;psi_end;v_des]);
%g_y = subs(g_y,[t;z;k],[t;x;y;psi;w_0;psi_end;v_des]);  

g_x = t.^[length(g_x_coeffs)-1:-1:0]*g_x_coeffs';
g_y = t.^[length(g_y_coeffs)-1:-1:0]*g_y_coeffs';

g = scale_vec.*[g_x; g_y; 0] ;  
        
%% 
  
hK = [(k(1)+1)*(1-k(1));...
      (k(2)+1)*(1-k(2))]; 
  
hZ = (z+1).*(1-z);
hX = 1-x.^2;

intXK = boxMoments([x;k],-ones(4,1),ones(4,1));
hFtprint = (xunscaled-(zunscaled(1:2)-[L;W]/2)).*(zunscaled(1:2)+[L;W]/2-xunscaled);
FRSstates = [x;k];
hFRSstates = [hX;hFtprint];

% L = [min(A.footprint_vertices(1,:)), max(A.footprint_vertices(1,:))];
% W = [min(A.footprint_vertices(2,:)), max(A.footprint_vertices(2,:))];

% hZ0 = [(x-L(1))*(L(2)-x);(y-W(1))*(W(2)-y);-psi^2];
  hZ0 = (zunscaled-Z0range(:,1)).*(Z0range(:,2)-zunscaled);
  
%deprcated int_TZK = boxMoments([t;z;k], [0;-1;-1;-1;-1;-1;-1],[1;1;1;1;1;1;1]);
  
prob = struct; 
    prob.t = t ;
    prob.z = z ;
    prob.x = x;
    prob.k = k ;
    prob.cost = intXK;
    prob.hZ = hZ ; 
    prob.hZ0 = hZ0; 
    prob.hK = hK; 
    prob.f = f; 
% prob.g = g ;
    prob.degree = 5; 
    prob.FRS_states = FRSstates;
    prob.hFRS_states  = hFRSstates;
% prob.cost = boxMoments([z(1:2);k], -ones(5,1),ones(5,1));

out = compute_FRS(prob); 
%%
filename = 'highway_frs_deg_6_new_ft_no_g.mat';
save(filename)
%%
figure(1); clf;hold on;axis equal
%%
krand = [0.03;10]%randRange(Krange(:,1),Krange(:,2));
krandscaled = (krand+koffset)./kscale;
% krandscaled(2) = 0;
z0rand = randRange(Z0range(:,1),Z0range(:,2));

% test actual dynamics here
% [~,ztmp] = ode45(@(t,z)dubins(t,z,krand),[0 T],z0rand);

% plot(ztmp(:,1),ztmp(:,2),'k','LineWidth',1)
% hold on
% ftps = [];
% for i = 1:length(ztmp)
%     ftps = +[ftps,ztmp(i,1:2)'+[-L/2 L/2 L/2 -L/2 -L/2;-W/2 -W/2 W/2 W/2 -W/2],NaN(2,1)];
% end
% 
% plot(ftps(1,:),ftps(2,:),'k')

%plot contour
wk = subs(out.indicator_function,k,krandscaled);
plot_2D_msspoly_contour(wk,x,1,'Offset',-xoffset,'Scale',xscale,'Color',[0 0.75 0.25],'LineWidth',1)