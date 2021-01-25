% w_0 = k(1);
% psi_end = k(2);
% v_des = k(3);
k_opt = [0.5; 0; 10];

saved_info = load('highway_frs_deg_4');
res = saved_info.out;
scaling = saved_info.scaling;
% F = P.FRS{P.latest_plan.current_FRS_index};
k = saved_info.k;
z = saved_info.z(1:2);

% pose0 = P.latest_plan.agent_state(1:3);
z0 = [];%todo define this


% psiend_k2 = (-pose0(3)-F.psi_end_min)*2/(F.psi_end_max-F.psi_end_min)-1;
% psiend_k2 = bound_values(psiend_k2,1);


wx = subs(saved_info.out.w,k,k_opt);

h = get_2D_contour_points(wx,z,1,'Scale',[scaling.x; scaling.y]);


% if ~check_if_plot_is_available(P,'FRS_contour')
    P.plot_data.FRS_contour = line(h(1,:),h(2,:),'Color',[0 0.75 0.25],'LineWidth',1.0);