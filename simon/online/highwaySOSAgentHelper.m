classdef highwaySOSAgentHelper < highwayAgentHelper
    properties
        w_polynomial_info = [];
        arc_point_spacing = 0;
        point_spacing = 0;
        FRS_buffer = 00.01;
        scale_factor = 0.5;
        trajopt_problem;
        start_tic;
    end
    methods
        function AH = highwaySOSAgentHelper(A,FRS_path,HLP,varargin)
            AH@highwayAgentHelper(A,FRS_path,HLP,varargin{:});
            [r,a] = compute_point_spacings(AH.A.footprint,AH.FRS_buffer);
            AH.point_spacing = r ;
            AH.arc_point_spacing = a ;
        end
        function [T, U, Z]=gen_ref(AH, K, real_reference_flag)
            % generate reference based on parameter and states
            agent_info = AH.get_agent_info();
            agent_state = agent_info.state(:,end);
            u_cur = agent_state(4) ;
            %             h_cur = agent_state(3) ;
            y_cur = agent_state(2) ;
            h_cur = agent_state(3) ;
            x_cur = agent_state(1) ;
            Au = K(1);%K = [agent_state(4); K;t0_idx;type_manu];
            Ay = K(2);
            t0_idx = 1;% K(3); there is no t0_idx for sos
            
            t0 = (t0_idx-1)*AH.t_plan;
            type_manu = K(4);
            load_const
            %not symbolic mode time argument will be ignored
            %del_y,Au,u0,t,symbolic_flag, scale_Ay_flag                t0
            if type_manu == 3
%                 [T, U,Z] = gaussian_T_parameterized_traj_with_brake(t0,Ay,Au,u_cur,[],0,1);
            else
                [T, U,Z] = make_highway_desired_trajectory(3,agent_state(6),Ay,Au) ;

            end
            
            if ~exist('real_reference_flag','var')
                real_reference_flag = 1;
            end
            if real_reference_flag
                AH.ref_Z=[AH.ref_Z;x_cur+Z(1,:);y_cur+Z(2,:)];% for plotting
                AH.t_real_start = [AH.t_real_start;AH.A.time(end)];
            else
                AH.proposed_ref_Z=[AH.proposed_ref_Z;x_cur+Z(1,:);y_cur+Z(2,:)];% for plotting
                AH.t_proposed_start = [AH.t_proposed_start;AH.A.time(end)];
            end
            Z = [x_cur+Z(1,:);y_cur+Z(2,:);h_cur+Z(3,:)];
            
        end
        function [avaliable_action_set] = find_optimal_cont(AH,O,agent_state,FRS,mirror_flag,AuAyflag, zono_c, zono_g,x_des)
%             for i = 1:length(FRS.delta_force)
%                 FRS.delta_force{i} =  deleteAligned(FRS.delta_force{i});
%             end
            avaliable_action_set =[];
            AH.start_tic = tic;

            O = AH.process_world_info(O,FRS,agent_state,mirror_flag);
            AH.create_constraints(O);
            AH.create_cost_function(x_des,FRS,agent_state);
            
            initial_guess = 0;
            try
            if ~isempty(AH.trajopt_problem.nonlcon_function)
                [k_opt,~,exitflag] = fmincon(@(x) AH.trajopt_problem.cost_function(x),...
                    initial_guess,...
                    [],...
                    [],...
                    [],[],... % linear equality constraints
                    AH.trajopt_problem.k_bounds(:,1),...
                    AH.trajopt_problem.k_bounds(:,2),...
                    @(x) AH.trajopt_problem.nonlcon_function(x),...
                    AH.fminconopt) ;
            else
               [k_opt,~,exitflag] = fmincon(@(x) AH.trajopt_problem.cost_function(x),...
                    initial_guess,...
                    [],...
                    [],...
                    [],[],... % linear equality constraints
                    AH.trajopt_problem.k_bounds(:,1),...
                    AH.trajopt_problem.k_bounds(:,2),...
                    [],...
                    AH.fminconopt) ;
            end 
            catch
                exitflag = -1;
                warning ('optimization times out')
            end
            if exitflag == 1 || exitflag == 2
                k_opt = k_opt * FRS.kscale(2)- FRS.koffset(2);
                avaliable_action_set = k_opt;
                toc(AH.start_tic)
            end
            
%             try
%                 [k,fval,exitflag,~] = fmincon(cost, initial_guess, [], [],...
%                     [], [], lb, ub, cons,AH.fminconopt) ;
%             catch
%                 exitflag = -1;%timeout
%                 warning ('optimization times out')
%             end
% 
%             if exitflag == 1 || exitflag == 2 
%                 avaliable_action_set = k;
%                 toc(start_tic)
%             else
%                 avaliable_action_set = [];
%             end
        end
        function plot_selected_parameter_FRS(AH,K,type_manu,FRS,mirror_flag,agent_state,multiplier)
             if ~isempty(K)
            F = FRS.out;
            % process the w polynomial from the rover's FRS; this just
            % extracts the msspoly's powers and coefficients to speed up
            % evaluating the polynomial online
            k = F.input_problem.k;
            w = F.w;
            
            u0_k = (agent_state(4) + FRS.koffset(3))/FRS.kscale(3);
            u0_k = bound_values(u0_k,1);
            w0_k = (agent_state(6) + FRS.koffset(1))/FRS.kscale(1);
            w0_k = bound_values(w0_k,1);
            psi_k = (K*multiplier + FRS.koffset(2))/FRS.kscale(2);
            
            %set second parameter to negative heading (end trajectory plan
            %aligned with road
%             psiend_k = (-.agent_state(3)-F.psi_end_min)*2/(F.psi_end_max-F.psi_end_min)-1;
            %free to choose psiend k with the fmincon
% psiend_k = bound_values(psiend_k,1);
            w0_k,psi_k,u0_k
            w = msubs(w,[k(1);k(2);k(3)],[w0_k;psi_k;u0_k]);
            % viz make sure things are ook:
            if AH.plot_flag
                if mirror_flag
                    figure(1);subplot(3,1,3)
                else
                    figure(1);subplot(3,1,2)
                end
               
                plot_2D_msspoly_contour(w,F.input_problem.x,1,'Offset',-FRS.xoffset,'Scale',FRS.xscale,'Color',[0 1 0],'LineWidth',1)

            end

             end
        end
        function create_cost_function(AH,x_des,FRS,agent_state)
            u = agent_state(4);
            w0= agent_state(6);
            psi_max = FRS.vbls.p_end_max;
            psi_min = FRS.vbls.p_end_min;

            AH.trajopt_problem.cost_function = @(k2) AH.cost_highway_fmincon(k2,psi_min,psi_max,w0,u,x_des(1),x_des(2),AH.start_tic);

        end
        function [c, gc] = cost_highway_fmincon(AH,k2,psi_min,psi_max,w0,u,x_des,y_des,start_tic)     
            
            c = highway_cost(k2,psi_min,psi_max,w0,u,x_des,y_des,1,1);
            gc = highway_cost_grad(k2,psi_min,psi_max,w0,u,x_des,y_des,1,1);
            
            if toc(start_tic) > AH.t_plan
                error('Trajopt timed out!')
            end
        end
        function create_constraints(AH,O)
            % get the processed obstacles
            if ~isempty(O)
                % remove NaNs
                O_log = isnan(O(1,:)) ;
                O = O(:,~O_log) ;

                % plug in to w polynomial to get list of constraints w(x,k) for
                % each x \in O
                w = AH.w_polynomial_info ;

                wk = sub_z_into_w(w,O) ;
                wkcoef = wk.wkcoef ;
                wkpows = wk.wkpows ;

                % create Jacobian of the constraints
                [Jcoef, Jpows] = diff_wk_wrt_k(wk) ;
                N = wk.N ;

                % create constraint function
                AH.trajopt_problem.nonlcon_function = @(k) AH.nonlcon_highway(k,wkcoef,wkpows,Jcoef,Jpows,N,AH.start_tic) ;
            else
                % if there are no obstacles then we don't need to consider
                % any constraints
                AH.trajopt_problem.nonlcon_function = [] ;
            end
           
            

            AH.trajopt_problem.k_bounds = [-1,1];
        

            
        end
        function [n, neq, gn, gneq] = nonlcon_highway(AH,k,wkcoef,wkpows,Jcoef,Jpows,N,start_tic)
            % constraint is active when p(k) > 0
            n = eval_w(k,wkcoef,wkpows) ;
            neq = zeros(size(n)) ;
            gn = eval_J(k,Jcoef,Jpows,N)' ;
            gneq = zeros(size(gn)) ;
           
            if toc(start_tic) > AH.t_plan
                error('Trajopt timed out!')
            end
        end
        function O_FRS = process_world_info(AH,O,FRS,agent_state,mirror_flag)
            
            % get the FRS from the current planning iteration
            %k1 = w0 k2 = psi_des k3 = u0
            %x y z h u v r
            
            F = FRS.out;
            % process the w polynomial from the rover's FRS; this just
            % extracts the msspoly's powers and coefficients to speed up
            % evaluating the polynomial online
            k = F.input_problem.k;
            w = F.w;
            
            %shift to 0 [0,1];
            w_full = w - 1.0001 ; % this is so (x,k) \in FRS => w(x,k) > 0
            u0_k = (agent_state(4) + FRS.koffset(3))/FRS.kscale(3);
            u0_k = bound_values(u0_k,1);
            w0_k = (agent_state(6) + FRS.koffset(1))/FRS.kscale(1);
            w0_k = bound_values(w0_k,1);

            %set second parameter to negative heading (end trajectory plan
            %aligned with road
%             psiend_k = (-.agent_state(3)-F.psi_end_min)*2/(F.psi_end_max-F.psi_end_min)-1;
            %free to choose psiend k with the fmincon
% psiend_k = bound_values(psiend_k,1);
            
            w = msubs(w_full,[k(1);k(3)],[w0_k;u0_k]);
            % viz make sure things are ook:
            if AH.plot_flag
                if mirror_flag
                    figure(1);subplot(3,1,3)
                else
                    figure(1);subplot(3,1,2)
                end
                w_plot1 = msubs(w,[k(2)], [1]);
                w_plot2 = msubs(w,[k(2)], [0]);
                w_plot3 = msubs(w,[k(2)],[-1]);
                plot_2D_msspoly_contour(w_plot1,F.input_problem.x,0,'Offset',-FRS.xoffset,'Scale',FRS.xscale,'Color',[0 0 1],'LineWidth',1)
                plot_2D_msspoly_contour(w_plot2,F.input_problem.x,0,'Offset',-FRS.xoffset,'Scale',FRS.xscale,'Color',[0 0 1],'LineWidth',1)
                plot_2D_msspoly_contour(w_plot3,F.input_problem.x,0,'Offset',-FRS.xoffset,'Scale',FRS.xscale,'Color',[0 0 1],'LineWidth',1)

            end

            AH.w_polynomial_info = decompose_w_polynomial(w,F.input_problem.x,k(2)) ;
           
            % buffer and discretize the obstacles
            if ~isempty(O)
                O = buffer_box_obstacles(O,AH.FRS_buffer,'a',AH.arc_point_spacing) ;
                O_center = interpolate_polyline_with_spacing(O,AH.point_spacing) ;
                %boundary point already included in O
                %add boundary points
               
                
                % move obstacles into FRS coordinates for generating nonlinear
                % constraints with the w polynomial
                % already in reference frame of ego
                %get rid of obstacles that are unreachable because they lie
                %outside of a polygon that contains the reachable set for
                %all trajecotry parameters
                FRS_box = (make_box(FRS.xscale*2));
                L = inpolygon(O_center(1,:)',O_center(2,:)',FRS_box(1,:)',FRS_box(2,:)');
                O_center = O_center(:,L);
                if AH.plot_flag
                    scatter(O_center(1,:),O_center(2,:))
                end
                O_FRS = (O_center+FRS.xoffset)./FRS.xscale;


                % get rid of obstacle points that are definitely unreachable
                % because they lie outside of the unit circle in the FRS
                % coordinate frame (this is because of how we create the SOS
                % program to compute the FRS)
                O_FRS = crop_points_outside_region(0,0,O_FRS,AH.scale_factor) ;
%                 O_mirrored_FRS = crop_points_outside_region(0,0,O_mirrored_FRS,0.9);
            else
                O_FRS = [] ;
            end    
            
        end
    end
    
end