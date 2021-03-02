% Test to see if g function encompass things
clear
files = dir;
for i=1:length(files)
    if contains(files(i).name,'highway_error_functions') && contains(files(i).name,'u0=15_k2=0.030273')
        load(files(i).name)
        A = highway_cruising_6_state_agent ;
        figure(1);clf;hold on;
        
        [zscaling,zoffset] = calculate_scaling_values(A, g_x_coeffs, g_y_coeffs, vbls);
        %         ylim ([-2,6]);axis equal;
        text(0,0,['u0 = ',num2str(vbls.u0_min)],'FontSize',13);
        drawnow
        %% test 2
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
        Krange = [vbls.w_min, vbls.w_max;
            vbls.p_end_min, vbls.p_end_max; % always 0
            vbls.u0_min, vbls.u0_max];
        kscale = (Krange(:,2)-Krange(:,1))/2;
        koffset = -Krange(:,1)-kscale;
        
        syms t z1 z2 z3 x1 x2 k1 k2 k3
        tunscaled = vbls.t_f*t;
        zunscaled1 = zscaling(1).*z1-zoffset(1);
        zunscaled2 = zscaling(2).*z2-zoffset(2);
        zunscaled3 = zscaling(3).*z3-zoffset(3);
        xunscaled1 = xscale(1).*x1-xoffset(1);
        xunscaled2 = xscale(2).*x2-xoffset(2);
        kunscaled1 = kscale(1).*k1-koffset(1);
        kunscaled2 = kscale(2).*k2-koffset(2);
        kunscaled3 = kscale(3).*k3-koffset(3);
        %psi = scaling.p*z(3);
        
        %unscaled parameters
        w_0     = k1;%w0_uns= (vbls.w_max-vbls.w_min)/2*(k(1)+1)+vbls.w_min;
        psi_end = k2;%psi_uns=  (vbls.p_end_min-vbls.p_end_max)/2*(k(2)+1)+vbls.p_end_min;
        v_des   = k3;%v_uns =  (vbls.u_max-vbls.u_min)/2*(k(2)+1)+vbls.u_min;
        
        w0_uns = kunscaled1;
        psi_uns = kunscaled2;
        v_uns = kunscaled3;
        
        %% dynamics
        t_f = vbls.t_f;
        
        w_slope = -2*(t_f*w0_uns-psi_uns)/t_f^2;
        
        w_des = w0_uns+w_slope*tunscaled;
        
        psi = zunscaled3;
        
        cos_psi = 1-psi^2/2;
        sin_psi = psi-psi^3/6;
        
        
        scale_vec =  t_f./zscaling;
        f = scale_vec.*[v_uns*cos_psi-A.lr*w_des*sin_psi ;
            v_uns*sin_psi+A.lr*w_des*cos_psi ;
            w_des] ;
        g_x = (t*t_f).^[length(g_x_coeffs)-1:-1:0]*g_x_coeffs';
        g_y = (t*t_f).^[length(g_y_coeffs)-1:-1:0]*g_y_coeffs';
        
        g = scale_vec.*[g_x 0 0;0 g_y 0; 0 0 0] ;
        %%
        
        for j = 1:60
            if j >30
            d_rand = -1*ones(3,1);
            for rand_d_idx = 1:3
                if rand > 0.5
                d_rand(rand_d_idx) = 1;
                end
            end
            else
                d_rand = 0*ones(3,1);

            end
        k_rand = randRange(Krange(:,1),Krange(:,2));
        k_rand = (k_rand + koffset)./kscale; 
        Z = (zeros(3,1)+zoffset)./zscaling;
        dt = 0.01;
        for t_now = 0:dt:1
            dz = subs(g,t,t_now)*d_rand+double(subs(f,[z1,z2,z3,k1,k2,k3,t],[Z(1,end),Z(2,end),Z(3,end),k_rand(1),k_rand(2),k_rand(3),t_now]));
            Z(:,end+1) = dz*dt+Z(:,end);
        end
%         figure(2);
        if j >30
            color = 'g';
        else
            color = 'k';
        end
        plot(Z(1,:)*zscaling(1)-zoffset(1),Z(2,:)*zscaling(2)-zoffset(2),'Color',color);
        end
        f_int = int(f,t,0,1);%about k, z, evaluate on a k
    end
end
%% Test to see if scaled dynamics evolve correctly

% function plot_minmax(agentZ,movmeansize, window_size,stop_eps,color)
% % epsrewZ       = movmean(agentZ.savedAgentResultStruct.TrainingStats.AverageReward,movmeansize);
% %% This can also be used for plotting min/max shading, but doesn't show really well so omitted.
% epsrewZstd = movstd(agentZ.savedAgentResultStruct.TrainingStats.AverageReward,window_size);
% % epsrewZmin = movmin(agentZ.savedAgentResultStruct.TrainingStats.AverageReward,window_size);
% x_data = 1:length(epsrewZ);
% x2 = [x_data, fliplr(x_data)];
% inBetween = [(epsrewZ-epsrewZstd)', fliplr((epsrewZ+epsrewZstd)')];
% pf = fill(x2, inBetween, color);
% pf.FaceAlpha = 0.3;
% pf.EdgeAlpha = 0;
% endZ  = min([stop_eps length(epsrewZ)]);
% plot(epsrewZ(1:endZ),'LineWidth',3,'Color',color);
% % pZminmax =plot(x_data(1:endZ*2),epsrewZminmax(1:endZ*2),'--','LineWidth',5,'Color',color);
% % pZmin =plot(epsrewZmin(1:endZ),'--','LineWidth',1,'Color',Z_color);
% end