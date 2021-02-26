classdef highway_rover_LLC < low_level_controller
    properties
        % control gains
        yaw_gain = 1.5;
        yaw_rate_gain =3;
        lateral_position_gain = 0.8;
        longitudinal_position_gain = 1;
        Ku = 3;
    end
    
    methods
        %% constructor
        function LLC = highway_rover_LLC(varargin)
            n_agent_states = 6 ;
            n_agent_inputs = 2 ;
            
            LLC = parse_args(LLC,'n_agent_states',n_agent_states,...
                'n_agent_inputs',n_agent_inputs,varargin{:}) ;
        end
        
        %% get control inputs
        function [Fxf,Fyf] = get_control_inputs(LLC,A,t_cur,z_cur,T_des,U_des,Z_des,Fxr,Fyr)
            %Fxf Fyf
            h_cur = z_cur(3);
            u = z_cur(4);
            v = z_cur(5);
            r = z_cur(6);
%             if t_cur >1
%             a= 1
%             end
%             ud =interp1(T,U(1,:),t,'linear') ;% desired lon.velocity(u)
%             uddot =interp1(T,U(4,:),t,'linear');
%             
%             vd = interp1(T,U(2,:),t,'linear') ; % desired heading change,try gaussian
%             vddot =interp1(T,U(5,:),t,'linear')  ;
%             
%             rd = interp1(T,U(3,:),t,'linear') ; % desired heading change,try gaussian
%             hd = interp1(T,Z(3,:),t,'linear') ; % desired heading change,try gaussian
            % get current state
            [u_des,z_des] = match_trajectories(t_cur,T_des,U_des,T_des,Z_des,'previous') ;
            %z_des = xd yd hd
            %u_des = ud rd
            h_des = z_des(3) ;
            rd    = u_des(2) ;

            local_position = rotation_matrix_2D(h_des)'*(z_cur(A.position_indices)-z_des(A.position_indices));
%             Fxf = -Fxr - A.m*v*r/2 - A.m/2*A.Ku*(u-ud) + A.m/2*uddot;
            ud = u_des(1)-LLC.longitudinal_position_gain*local_position(1); 
            Fxf = -Fxr - A.m*v*r/2  + A.m/2*A.Ku*(ud-u);


            k_h = LLC.yaw_gain ;
            k_w = LLC.yaw_rate_gain ;
            k_y = LLC.lateral_position_gain;

            %convert yawrate to wheelangle

            rcmd = rd-k_h*wrapToPi(h_cur-h_des)-k_y*local_position(2)-k_w*(r-rd);
            Fyf  = -Fyr + A.m*u*r/2 - A.Kr*A.m/2*(r-rcmd);%- A.m/2*A.Kv*(v-vd)
%             if v_cur ~=0
%                 delta_des = atan(w_cmd*A.wheelbase/v_cur);
%             else
%                 delta_des = 0;
%             end
% 
%             U = [v_des;delta_des];
            
           
        end
    end
end