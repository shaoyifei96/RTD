%clear; 
%close all;
% plot flag


plot_sim_flag = 1;
plot_AH_flag = 0;
AH_debug_flag = 1;

%% set up required objects for simulation
% agent -0.7 2m 6m 10m 0.7m  
% bounds = [0,100,-0.7,30] ;
bounds = [-500,0,-0.7,12.7] ;
goal_radius = 2 ;
world_buffer = 1 ; % this is used to make start and goal locations that are not too close to the robot or boundary
HLP_buffer = 0.1;
RTD_HLP_grow_tree_mode ='seed';
% planner
buffer = 0.3 ; % [m] distance used to buffer obstacles for planning, must be larger than the agent's footprint
t_plan = 2.0 ; % if t_plan = t_move, then real time planning is enforced
t_move =0.2;
t_failsafe_move = 0.75;
Ts = t_move;
Tf = Ts*150;

% verbose level for printing
verbose_level = 0;

% automated from here
A3 = highway_cruising_6_state_agent;
A3.integrator_type= 'ode4';
A3. desired_initial_condition = [bounds(1)+10; 6; 0;10; 0; 0];
W = dynamic_car_world('bounds',bounds,'buffer',world_buffer,...
    'verbose',verbose_level,'goal_radius',goal_radius,'num_cars',8) ;

HLP = highway_HLP;%RRT_star_HLP('timeout',0.5,'bounds',bounds,'grow_tree_mode',RTD_HLP_grow_tree_mode,'verbose',verbose_level);
AH = highwaySOSAgentHelper(A3,'FRS_SOS_dir_change.mat',HLP,'t_move',t_move,'t_failsafe_move',t_failsafe_move,'eps',0.001,'verbose',verbose_level,'plot_flag',AH_debug_flag);
S = rlsimulator(AH,W,'plot_sim_flag',plot_sim_flag, 'safety_layer','A','plot_AH_flag',plot_AH_flag);
% automatic mode: A: using a High level planner to generate waypoints, use a
% optimization program(sample) to select best parameter
% (Broke) replacement mode: Z: edit the user selected parameter to something safe
% No safe mode: N: no safe stuff..
random_inputs = true; %true for user inputs using arrow keys, false for random inputs
%%
  S.eval = 1; %turn on evaluation so summary will be saved
% to the episode number

if true
    for j = 1
        S.reset();
        for i = 1: 100
            if random_inputs
                value = 114;
            else
                figure(1)
                k = waitforbuttonpress;
                value = double(get(gcf,'CurrentCharacter'));
            end
            
            % 28 leftarrow
            % 29 rightarrow
            % 30 uparrow
            % 31 downarrow
            
            if value == 28
                [~,~,IsDone,LoggedSignal]=S.step([-2/3;0]);
            elseif value == 29
                [~,~,IsDone,LoggedSignal]=S.step([1;0]);
            elseif value == 30
                [~,~,IsDone,LoggedSignal]=S.step([1/3;1]);
            elseif value == 31
                [~,~,IsDone,LoggedSignal]=S.step([1/3;-1]);
            elseif value == 114
                [~,~,IsDone,LoggedSignal]=S.step([rand*2-1;rand*2-1]);
            else
                [~,~,IsDone,LoggedSignal]=S.step([1/3;0]);
            end
            
            if IsDone == 1 || IsDone == 3 || IsDone == 4 || IsDone == 5
                %crash       
                %      crash with safety layer on
                %                      safely stopped but stuck
                %                                           reached goal!
                break
            end
            
        end
    end
end

done = 'Setup Complete'