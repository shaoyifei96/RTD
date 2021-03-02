%% description
close all
clear
%
% This script computes the tracking error function "g" for the rover.
% See the paper "Bridging the Gap Between Safety and Real-Time Performance
% in Receding-Horizon Trajectory Design for Mobile Robots" for an
% explanation of the error function in Section 2.2.2. In particular, see
% Assumption 10 that defines the tracking error function.
%
% The paper is available here: https://arxiv.org/abs/1809.06746
%
% Author: Sean Vaskov
% Created: 24 March 2020

g_degree_x = 1;
g_degree_y = 1;

% lane_change_info = load('lane_change_Ay_info.mat');
dir_change_info = load('dir_change_Ay_info.mat');

% number of samples in v0, w, and v
N_samples = 6;
consider_footprint = 0;
recalc_error_fun = 1;
% timing
t_sample = 0.01 ;

plotting = true;
save_data_flag = 1;
T_len = 3;
t0idx = 1;
%% dir change
% initial condition bounds (recall that the state is (x,y,h,v), but the
% robot's dynamics in SE(2) are position/translation invariant)
u0vec =5:2:27;
load_const
%velocity initial and desired bounds
for uidx = 1: length(u0vec)
    
    
    u0_select = u0vec(uidx);
    u0min = u0_select-1;
    u0max = u0_select+1;
    u0vec_info = dir_change_info.u0_vec;
    u0info_idx = find(u0vec_info == u0_select);
%     Ay = dir_change_info.Ay_vec(u0info_idx);
    
    % del_y_arr = linspace(0,Ay,9);%    del_y_arr = -0.8:0.2:0;
    % del_y_arr = del_y_arr(2:2:end);
    % delta_y   = (del_y_arr(2)-del_y_arr(1))/2;
%     del_y_arr = [Ay];
%     syms t_dummy
%     w = 1/1.1*del_y_arr*exp(-3*(t_dummy - 3/2).^2); %this is ref for dir change
%     delta_h = double(int(w,t_dummy,0,T_len));
%     k2_arr = linspace(0,delta_h,9);
%     k2_arr = k2_arr(2:2:end);
%     k2_delta = (k2_arr(2)-k2_arr(1))/2;
    k2_min = 0;
    k2_max = 1; Z_ref=[0;0;0];
    while abs(Z_ref(2,end) - 4) > 0.05 
        avg_k2 = 0.5*(k2_min + k2_max);
        [T_ref,U_ref,Z_ref] = make_highway_desired_trajectory(T_len,0,avg_k2,u0_select);
        if Z_ref(2,end) > 4
            k2_max =avg_k2;
        else
            k2_min =avg_k2;
        end
    end
    k2_arr = linspace(0,avg_k2,9);
    k2_arr = k2_arr(2:2:end);
    k2_delta = (k2_arr(2)-k2_arr(1))/2;
    % b = delta_h'\del_y_arr';
    % figure();clf;hold on;
    % plot(del_y_arr,delta_h,del_y_arr,0.9.*del_y_arr);
    for k2idx = 1:4 %should consider r0 in range of
        r_vec = dir_change_info.r0v0_limit(u0info_idx);
        % v_vec = dir_change_info.v_value_vec(u0info_idx,:);
        r0v0limit = dir_change_info.r0v0_limit(u0info_idx);
        %script will loopthrough all of these combos, fit error functions for each
        %then save as separate files
        
        % delta0_combos = [-0.5, -0.3,  -0.15, -0.05, 0.05, 0.15, 0.3;...
        %                  -0.3, -0.15, -0.05,  0.05, 0.15, 0.3,  0.5];
        
        
        
        
            
        
        %% automated from here
        
        % for dcom = delta0_combos
        %
        % delta0_min = dcom(1);
        % delta0_max = dcom(2);
        
        % create roveragent
        % RoverLLC = rover_PD_LLC('yaw_gain',5,'yaw_rate_gain',0.5);
        %dont need llc, agent is closed looped
        A = highway_cruising_6_state_agent ;
        A.integrator_type= 'ode45';     %x  y  h  u   v  r
        % A. desired_initial_condition = [0; 0; 0; 10; 0; 0];
        if recalc_error_fun 
        l = A.lr+ A.lf;
        lr = A.lr;
        
        % create initial condition vector for velocity
        u0_samples = linspace(u0min,u0max,N_samples) ; %initial u
        v0_samples = linspace(-r0v0limit,r0v0limit,N_samples) ; %inital v
        r0_samples = linspace(-r0v0limit,r0v0limit,N_samples) ; %inital v
        psi_end_min = k2_arr(k2idx) - k2_delta;
        psi_end_max = k2_arr(k2idx) + k2_delta;
        psiend_vec = linspace(psi_end_min,psi_end_max,N_samples);
        %since it we start facing stright,it does not have v.
        % create initial condition vector for wheelangle
        % delta0_vec = linspace(delta0_min,delta0_max,N_samples) ;
        %         t_data = 0:0.1:T_len;
        
%         z_range = zeros(3,2);
%         z_range(:,1) = inf;
%         z_range(:,2) = -inf;
        
        %since want to end up straight, does not have psi.
        
        % load timing
%         try
%             disp('Loading r RTD planner timing info.')
%             timing_info = load('.mat') ;
%             t_plan = timing_info.t_plan ;
%             t_stop = timing_info.t_stop ;
%             t_f = timing_info.t_f ;
%         catch
%             disp('Could not find timing MAT file. Setting defaults!')
        t_plan = tpk_dir;
        t_stop = tpk_dir + tbrk;
        t_f =  tpk_dir ;
%         end
        
        % initialize time vectors for saving tracking error; we use two to separate
        % out the braking portion of the trajectory
        T_data = unique([0:t_sample:T_len,T_len]) ;
        
        % initialize arrays for saving tracking error; note there will be one row
        % for every (v0,w_des,v_des) combination, so there are N_samples^3 rows
        N_total = N_samples^5;
        
        
        
        %parameters we want g to be a function of
        psi_ref_data = nan(N_total,length(T_data));
        v_des_data = nan(N_total,length(T_data));
        w0_des_data = nan(N_total,length(T_data));
        psi_end_data = nan(N_total,length(T_data));
        
        
        %% tracking error computation loop
        err_idx = 1 ;
        
        tic
        % v0_vec = linspace(v_des_min,v_des_max, N_samples) ;
        x_err = [];y_err= [];
        %inital paramters: u0 v0
        %desired parameters: u_des psi_des w0_des
        % for each initial condition...
        %% !!!!!        fast            slower              slowest
        % FRS mode: Spd Change = 1 , Lane change = 2 , Compute_all_together = 0
        %these are generaetd seperately, if doing spd change, y error should be
        %very minimal.
        %But if doing lane change, x error seems bigger than all the data for some
        %reason.
        %         mode = 2;
        u_des_rec = [];
        for u0 = u0_samples %  initial
            for v0 = v0_samples %initial
                for r0 = r0_samples
                    % for each yaw and speed command...
                    %                 if mode == 2
                    u_vec = u0;
                    %                 end
                    for u_des = u_vec %command
                        
                        u_des_rec = [u_des_rec; u_des];
                        %psi_end is always 0 since it alawys starts at heading 0 and
                        %end with heading=0;
                        for psi_end = psiend_vec % psiend_vec % two desired, psi always end with heading forward! negative of inital heading
                            %still need a bunch of values for rotation
                            % create the initial condition
                            %                 z0 = [0;0;0;u0;delta0] ; % (x,y,h,v,delta)
                            %x  y  h  u   v  r
                            z0= [0; 0; 0; u0; v0; r0];
                            
                            
                            %create feasible initial yawrate commands from initial heading
                            %                 w0_des_min_temp = max(w0_des_min, 1/0.5*psi_end-1);
                            %                 w0_des_max_temp = min(w0_des_max, 1/0.5*psi_end+1);
                            
                            %                 w0_des_vec = linspace(w0_des_min_temp,w0_des_max_temp, N_samples-1);
                            %                 if u_des == 5
                            %                     w0_des_vec = linspace(-0.1,0.1,N_samples-1);%make sure it is odd so center is there
                            %                 elseif u_des == 7
                            %                     w0_des_vec = linspace(-0.08,0.08,N_samples-1);
                            %                 elseif u_des == 9
                            %                     w0_des_vec = linspace(-0.06,0.06,N_samples-1);
                            %                 elseif u_des == 11
                            %                     w0_des_vec = linspace(-0.055,0.055,N_samples-1);
                            %                 elseif u_des == 13
                            %                     w0_des_vec = linspace(-0.05,0.05,N_samples-1);
                            %                 elseif u_des == 15
                            %                         w0_des_vec = linspace(0.05,0.3,N_samples-1);
                            %                 end
                            %                         if mode == 1
                            %                             w0_des_vec = 0;
                            %                         end
                            %                         for w0_des = w0_des_vec
                            
                            % create the desired trajectory
                            [T_ref,U_ref,Z_ref] = make_highway_desired_trajectory(t_f,r0,psi_end,u_des) ;
                            
                            % reset the robot
                            
                            A.reset(z0)
                            
                            % track the desired trajectory
                            A.move(t_f,T_ref,U_ref,Z_ref) ;
                            
                            %                     figure(1);clf; hold on;
                            %                     plot(A.state(1,:),A.state(2,:))
                            %                     plot(Z_ref(1,:),Z_ref(2,:))
                            %                     plot(U_ref')
                            % compute the error before t_plan
                            T = A.time ;
                            X = A.state(A.position_indices,:) ;
                            
                            % interpolate the desired and realized trajectory to match
                            X_des = Z_ref(1:2,:) ;
                            X = match_trajectories(T_ref,T,X) ;
                            
                            % compute the tracking error
                            pos_err = X - X_des ;
                            
                            % collect the data
                            x_err = [x_err ; pos_err(1,:)] ;
                            y_err = [y_err ; pos_err(2,:)] ;
                            
%                             X_for_scaling = A.state(1:3,:);
%                             x_max = max(X_for_scaling(1,:),[],'all');
%                             x_min = min(X_for_scaling(1,:),[],'all');
%                             y_max = max(X_for_scaling(2,:),[],'all');
%                             y_min = min(X_for_scaling(2,:),[],'all');
%                             p_max = max(X_for_scaling(3,:),[],'all');
%                             p_min = min(X_for_scaling(3,:),[],'all');
%                             if x_min < z_range(1,1)
%                                 z_range(1,1) = x_min;
%                             end
%                             if y_min < z_range(2,1)
%                                 z_range(2,1) = y_min;
%                             end
%                             if p_min < z_range(3,1)
%                                 z_range(3,1) = p_min;
%                             end
%                             if x_max > z_range(1,2)
%                                 z_range(1,2) = x_max;
%                             end
%                             if y_max  > z_range(2,2)
%                                 z_range(2,2) = y_max ;
%                             end
%                             if p_max  > z_range(3,2)
%                                 z_range(3,2) = p_max ;
%                             end
                            
                            if mod(size(y_err,1),10) == 0
                                disp(['Iteration ',num2str(size(y_err,1)),' out of ',num2str(length(u0_samples)*length(v0_samples)*length(u_vec)*length(psiend_vec)*length(r0_samples))])
                            end
                            %                         end
                        end
                        
                    end
                end
            end
        end
        toc
        
        %% fit tracking error function
        % get max of absolute tracking error
        x_err = abs(x_err) ;
        y_err = abs(y_err) ;
        x_max = max(x_err,[],1) ;
        y_max = max(y_err,[],1) ;
        
        % fit polynomial to the data
        int_g_x_coeffs = polyfit(T_ref,x_max,g_degree_x) ;
        int_g_y_coeffs = polyfit(T_ref,y_max,g_degree_y) ;
        
        % take the time derivative of these to get the g functions in x and y
        g_x_coeffs = polyder(int_g_x_coeffs) ;
        g_y_coeffs = polyder(int_g_y_coeffs) ;
        
        %% correct the fit to make it greater than the data
        % evaluate g
        int_g_x_coeffs = polyint(g_x_coeffs) ;
        int_g_y_coeffs = polyint(g_y_coeffs) ;
        int_g_x_vals = polyval(int_g_x_coeffs,T_ref) ;
        int_g_y_vals = polyval(int_g_y_coeffs,T_ref) ;
        
        % figure out the maximum ratio of the error data to the int g values
        r_x_err = x_max ./ int_g_x_vals ;
        r_x_max = max([1,r_x_err]) ;
        r_y_err = y_max ./ int_g_y_vals ;
        r_y_max = max([1,r_y_err]) ;
        
        % multiply the g_x and g_y coefficients by the error data ratio
        g_x_coeffs = r_x_max .* g_x_coeffs ;
        g_y_coeffs = r_y_max .* g_y_coeffs ;
        
        % re-integrate g with the new coefficients
        int_g_x_coeffs = polyint(g_x_coeffs) ;
        int_g_y_coeffs = polyint(g_y_coeffs) ;
        int_g_x_vals = polyval(int_g_x_coeffs,T_ref) ;
        int_g_y_vals = polyval(int_g_y_coeffs,T_ref) ;
        
        %% save data
        
        
        vbls.u0_min = min(u0_samples);
        vbls.u0_max = max(u0_samples);
        vbls.v0_min = min(v0_samples);
        vbls.v0_max = max(v0_samples);
        vbls.u_min  = min(u_des_rec);
        vbls.u_max  = max(u_des_rec);
        vbls.p_end_min = min(psiend_vec);
        vbls.p_end_max = max(psiend_vec);
        vbls.w_min = min(r0_samples);
        vbls.w_max = max(r0_samples);
        vbls.t_f = t_f;
        
        
        filename = ['highway_error_functions_dir_change_u0=',num2str(u0_select),'_k2=',num2str(k2_arr(k2idx)),'.mat'] ;
        save(filename,'g_x_coeffs','g_y_coeffs','vbls') ;
        
        figure(1) ; clf ;
        
        % plot x error
        subplot(2,1,1) ; hold on ;
        plot(T_ref,x_err','k--')
        g_x_handle =  plot(T_ref,int_g_x_vals,'r-','LineWidth',1.5) ;
        title('tracking error vs. time')
        ylabel('x error [m]')
        legend(g_x_handle,'\int g_x(t) dt','Location','NorthWest')
        set(gca,'FontSize',15)
        
        % plot y error
        subplot(2,1,2) ; hold on ;
        plot(T_ref,y_err','k--')
        g_y_handle = plot(T_ref,int_g_y_vals,'r-','LineWidth',1.5) ;
        xlabel('time [s]')
        ylabel('y error [m]')
        legend(g_y_handle,'\int g_y(t) dt','Location','NorthWest')
        set(gca,'FontSize',15)
        filename_fig = ['highway_error_functions_dir_change_u0=',num2str(u0_select),'_k2=',num2str(k2_arr(k2idx)),'.png'] ;
        saveas(gcf,filename_fig);
        
       %%
        else
            load(['highway_error_functions_dir_change_u0=',num2str(u0_select),'_k2=',num2str(k2_arr(k2idx)),'.mat']);

        end
       for scale_value = [0.6]
        figure(1);clf;hold on
        [zscaling,zoffset] = calculate_scaling_values(A, g_x_coeffs, g_y_coeffs, vbls,scale_value);
        filename_fig = ['highway_scaling_function_dir_change_u0=',num2str(u0_select),'_k2=',num2str(k2_arr(k2idx)),'_scale=',num2str(scale_value),'.png'] ;
        saveas(gcf,filename_fig);

        maxz = zscaling-zoffset;
        minz = -zscaling-zoffset;
        Z0range = zeros(3,2);% intial condition
        
        %% with max footprint
        L = 4.8; W = 2; max_ft_len = sqrt((L/2)^2+(W/2)^2);
        maxx = maxz(1:2)+[max_ft_len; max_ft_len];
        minx = minz(1:2)-[max_ft_len; max_ft_len];
        
        xscale = (maxx-minx)/2;
        xoffset = -minx-(maxx-minx)/2;
        %% K range
        Krange = [vbls.w_min, vbls.w_max;
            vbls.p_end_min, vbls.p_end_max; % always 0
            vbls.u0_min, vbls.u0_max];
        kscale = (Krange(:,2)-Krange(:,1))/2;
        koffset = -Krange(:,1)-kscale;
        %% compute FRS
        %% extra x state for footprint
        x = msspoly('x',2);
        
        t = msspoly('t',1);
        z = msspoly('z',3);
        k = msspoly('k',3);
        
        
        %unscaled states
        
        tunscaled = vbls.t_f*t;
        zunscaled = zscaling.*z-zoffset;
        xunscaled = xscale.*x-xoffset;
        kunscaled = kscale.*k-koffset;
        %psi = scaling.p*z(3);
        
        %unscaled parameters
        w_0     = k(1);%w0_uns= (vbls.w_max-vbls.w_min)/2*(k(1)+1)+vbls.w_min;
        psi_end = k(2);%psi_uns=  (vbls.p_end_min-vbls.p_end_max)/2*(k(2)+1)+vbls.p_end_min;
        v_des   = k(3);%v_uns =  (vbls.u_max-vbls.u_min)/2*(k(2)+1)+vbls.u_min;
        
        w0_uns = kunscaled(1);
        psi_uns = kunscaled(2);
        v_uns = kunscaled(3);
        
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
        
        g_x = (t*t_f).^[length(g_x_coeffs)-1:-1:0]*g_x_coeffs';
        g_y = (t*t_f).^[length(g_y_coeffs)-1:-1:0]*g_y_coeffs';
        
        g = [g_x*scale_vec(1) 0 0; 0 g_y*scale_vec(2) 0; 0 0 0];
        
        %%
        
        hK = [(k(1)+1)*(1-k(1));...
            (k(2)+1)*(1-k(2));(k(3)+1)*(1-k(3))];
        
        hZ = (z+1).*(1-z);
        hX = 1-x.^2;
        if consider_footprint
            hFtprint = (xunscaled-(zunscaled(1:2)-[L;W]/2)).*(zunscaled(1:2)+[L;W]/2-xunscaled);
            FRSstates = [x;k];
            hFRSstates = [hX;hFtprint;hK];
            cost = boxMoments([x;k],-ones(5,1),ones(5,1));
        else
%             int_ZK = boxMoments([z;k], [Z_range(:,1);K_range(:,1)], [Z_range(:,2);K_range(:,2)]);
            hFtprint = 1 - ((xunscaled(1) - zunscaled(1))/max_ft_len).^2 - ((xunscaled(2) - zunscaled(2))/max_ft_len).^2 ;
            FRSstates = [x;k];
            hFRSstates = [hX;hFtprint;hK];
            cost = boxMoments([x;k],-ones(5,1),ones(5,1));
        end
        
        
        
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
        prob.cost = cost;
        prob.hZ = hZ ;
        prob.hZ0 = hZ0;
        prob.hK = hK;
        prob.f = f;
        prob.g = g ;
        prob.degree = 6;
        prob.FRS_states = FRSstates;
        prob.hFRS_states  = hFRSstates;
        % prob.cost = boxMoments([z(1:2);k], -ones(5,1),ones(5,1));
        
        [out,sol] = compute_FRS(prob);
        
        %% plotting
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
        wk = subs(out.indicator_function,k,krandscaled);
        plot_2D_msspoly_contour(wk,x,1,'Offset',-xoffset,'Scale',xscale,'Color',[0 0.75 0.25],'LineWidth',1)
        plot(X(1,:),X(2,:));
        drawnow
        filename_fig = ['highway_FRS_dir_change_u0=',num2str(u0_select),'_k2=',num2str(k2_arr(k2idx)),'_scale=',num2str(scale_value),sol.info.solverInfo.itr.prosta,sol.info.solverInfo.itr.solsta,'.png'] ;
        saveas(gcf,filename_fig);
        %%
        filename = ['highway_FRS_dir_change_u0=',num2str(u0_select),'_k2=',num2str(k2_arr(k2idx)),'_scale=',num2str(scale_value),sol.info.solverInfo.itr.prosta,sol.info.solverInfo.itr.solsta,'.mat'] ;
        save(filename,'sol','out','zscaling','zoffset','xscale','xoffset','kscale','koffset');
       end
    end
end

