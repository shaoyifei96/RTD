load highway_error_functions_dir_change_u0=5_k2=0.19619.mat
load highway_FRS_dir_change_u0=5_k2=0.19619.mat

t_f = 3;
A = highway_cruising_6_state_agent ;

Krange = [vbls.w_min, vbls.w_max;
           vbls.p_end_min, vbls.p_end_max; % always 0
           vbls.u0_min, vbls.u0_max];
       
figure(1); clf;hold on;axis equal
krand = randRange(Krange(:,1),Krange(:,2));
%         krand = [0.25;12]%
krandscaled = (krand+koffset)./kscale;
yline(2,'LineWidth',3)
yline(-2,'LineWidth',3)
yline(6,'LineWidth',3)
% krandscaled(2) = 0;
% z0rand = randRange(Z0range(:,1),Z0range(:,2));
z0= [0; 0; 0; krand(3); krand(1); 0];
% test actual dynamics here
[T_ref,U_ref,Z_ref] = make_highway_desired_trajectory(t_f,krand(1),krand(2),krand(3)) ;

A.reset(z0);

                    % track the desired trajectory
A.move(t_f,T_ref,U_ref,Z_ref) ;
T = A.time ;
X = A.state(A.position_indices,:) ;

% hold on
% ftps = [];
% for i = 1:length(ztmp)
%     ftps = +[ftps,ztmp(i,1:2)'+[-L/2 L/2 L/2 -L/2 -L/2;-W/2 -W/2 W/2 W/2 -W/2],NaN(2,1)];
% end
% 
% plot(ftps(1,:),ftps(2,:),'k')

%plot contour
wk = subs(out.indicator_function,out.input_problem.k,krandscaled);
plot_2D_msspoly_contour(wk,out.input_problem.x,1,'Offset',-xoffset,'Scale',xscale,'Color',[0 0.75 0.25],'LineWidth',1)
plot(X(1,:),X(2,:));
drawnow