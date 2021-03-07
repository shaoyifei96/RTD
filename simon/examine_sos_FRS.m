



clear;
files = dir('./frs_deg8_upto11mps/*.mat');
% err_files = dir('./dir_error_fun_deg1/*.mat');
dir_change_info = load('dir_change_Ay_info.mat');

% lane_info = load('lane_change_Ay_info.mat');
% dir_info = load('dir_change_Ay_info.mat');
dir_time_bin_num = 1;
lane_time_bin_num = 1;
u0_vec = 5:2:11;
dir_num = 4;
lan_num = 4;
M_mega = cell(0);
load_const
T_len = 3;
%velocity initial and desired bounds
k2_table = [];
t0_idx = 1;
for uidx = 1: length(u0_vec)
    
    
    u0_select = u0_vec(uidx);
    u0min = u0_select-1;
    u0max = u0_select+1;
    u0vec_info = dir_change_info.u0_vec;
    u0info_idx = find(u0vec_info == u0_select);
%     Ay = dir_change_info.Ay_vec(u0info_idx);
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
    k2_table = [k2_table; k2_arr];
end

for u0=u0_vec
    M = containers.Map;
    M(char("Autb"))= [];  %keep this one loose since the number of actions may be different for each spd
    M(char("Au"))= cell(0);
    
    M(char("dirtb"))= -1*ones(4,dir_num,dir_time_bin_num); %see slides for dimention help
    %     M(char("dirtb_0.75"))= [];
    %     M(char("dirtb_1.5"))= [];
    %     M(char("dirtb_2.25"))= [];
    
    M(char("dir"))= cell(dir_num,dir_time_bin_num);
    %     M(char("dir_0.75"))= cell(0);
    %     M(char("dir_1.5"))= cell(0);
    %     M(char("dir_2.25"))= cell(0);
    
    M(char("lantb"))= -1*ones(4,lan_num,lane_time_bin_num);
    %     M(char("lantb_0.75"))= [];
    %     M(char("lantb_1.5"))= [];
    %     M(char("lantb_2.25"))= [];
    %     M(char("lantb_3"))= [];
    %     M(char("lantb_3.75"))= [];
    %     M(char("lantb_4.5"))= [];
    %     M(char("lantb_5.25"))= [];
    
    M(char("lan"))= cell(lan_num,lane_time_bin_num);
    %     M(char("lan_0.75"))= cell(0);
    %     M(char("lan_1.5"))= cell(0);
    %     M(char("lan_2.25"))= cell(0);
    %     M(char("lan_3"))= cell(0);
    %     M(char("lan_3.75"))= cell(0);
    %     M(char("lan_4.5"))= cell(0);
    %     M(char("lan_5.25"))= cell(0);
    M_mega{end+1} = M;
    
end

for i=1:length(files)
    %     if contains(files(i).name,'_u=5_')
    display(files(i).name)
    if contains(files(i).name,'FRS')
    FRS = load (["./frs_deg8_upto11mps/"+files(i).name]);
    idxbegin = strfind(files(i).name,'u0=');
    idxend   = strfind(files(i).name,'_scale=');
    err_file = load(["./frs_deg8_upto11mps/highway_error_functions_dir_change_"+files(i).name(idxbegin:idxend-1)+'.mat']);
    idx1 = strfind(files(i).name,'_u0=');
    idx2 = strfind(files(i).name,'_k2');
    u0 = str2double(files(i).name(idx1+4:idx2-1));
    
    if u0 > 11
        continue;
    end
    mega_idx = find(u0 == u0_vec);
    M = M_mega{mega_idx};
    
    
    if (contains(files(i).name,'lane_change') || contains(files(i).name,'dir_change'))
        
        idx1 = strfind(files(i).name,'_k2=');
        idx2 = strfind(files(i).name,'_scale=');
        idx3 = strfind(files(i).name,'PRIMAL_');
        k2 = str2double(files(i).name(idx1+4:idx2-1));
        scale     = str2double(files(i).name(idx2+7:idx3-1));
        [~,k2_idx]=min(abs(k2_table(mega_idx,:)-k2));
        FRS = rmfield(FRS,'sol');
        FRS.vbls = err_file.vbls;
%         t0 = str2double(files(i).name(idxt1+3:idxt2-1));
%         t0_idx = t0/dt+1;
     
        if contains(files(i).name,'lane_change')
            lantb = M(char("lantb"));
            lantb(:,k2_idx,t0_idx) = [Ay;dx;dy;dh];
            M(char("lantb")) = lantb;
            
            MAy = M(char("lan"));
            MAy{k2_idx,t0_idx} = FRS;
            M("lan") = MAy;
            
            
        else %contains(files(i).name,'dir_change')
            dirtb = M(char("dirtb"));
            [tout, zout] = ode45( @(t,z) planning_model(t,z,err_file.vbls,[0;k2;u0],err_file.g_x_coeffs,err_file.g_y_coeffs,[0;0]), [0, err_file.vbls.t_f], [0;0;0]);
            dirtb(:,k2_idx,t0_idx) = [k2;zout(end,1);zout(end,2);zout(end,3)];
            M(char("dirtb")) = dirtb;
            
            MAy = M(char("dir"));
            MAy{k2_idx,t0_idx} = FRS;
            M("dir") = MAy;
        end
    elseif contains(files(i).name,'spd_change')%under construction
        idx1 = strfind(files(i).name,'_Au=');
        idx2 = strfind(files(i).name,'.mat');
        Au = str2double(files(i).name(idx1+4:idx2-1))
        M(char("Autb")) =  [M(char("Autb")) [Au;dx;dy;0]];
        
        MAu = M(char("Au"));
        MAu{end+1} = FRS;
        M('Au') = MAu;
    end
    end
end
% save('FRS_full_lane_dir_spd_large_forces.mat','M_mega')
%%
save('FRS_SOS_dir_change_deg8.mat','M_mega','-v7.3')
return
%% test FRS
clear;clc;close all
load('FRS_SOS_dir_change.mat');

for i = 1:4
    M = M_mega{i}('dir');
    for j = 1:4
%         msubs(M{j}.out.w
    end
end