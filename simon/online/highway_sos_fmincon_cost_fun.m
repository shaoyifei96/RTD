function cost = highway_sos_fmincon_cost_fun(K,agent_statex_des)
%K,agent_state, slice_idx,FRS.vehRS_save{FRS.brake_idx(1)}{1},x_des
cost = highway_cost_fun(c(1:3),x_des);
end
