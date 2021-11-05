clc;  clear; close all
%% Initialisation
lane_width = 3.5;  % Lane width
lane_length = 60;  % Lane length
y_right_border = - lane_width;  % Y coordinate of the right border of the road
y_left_border = lane_width;  % Y coordinate of the left border of the road
veh_width = 1.7;  % Vehicle width
veh_length = 4.7;  % Vehicle length
road_x = 0;  % X coordinate of the road bottom left corner
ego_rr_x = 1;  % Initial X coordinate of the right rear tire of the ego vehicle
ego_rr_y = -lane_width/2 - veh_width/2;  % Initial Y coordinate of right rear tire of the ego vehicle

% Initial position of COG of the ego vehicle [x y]
P_init = [5, -1.75]; 
% Initial yaw angle of the ego vehicle
alpha_init = 10/180*pi;

% Position of the goal
Pg = [lane_length-2, 1.75];


d_goal = 0.1;  % The desired maximum distance between the goal and the ego vehicle

% Position and lane of obstacles
dist_obs = 15;
car1 = [ego_rr_x+15, ego_rr_y+veh_width/2];
car2 = [car1(:,1)+dist_obs, -car1(:,2)];
car3 = [car2(:,1)+dist_obs, -car2(:,2)];
% Pobs = [car1;car2;car3];
Pobs = [car1;car2;car3];

% Longitudinal distance between the ego vehicle and obstacles for changing lane
dist_lon = 8;

% Step for the movement of ego vehicle in longitudinal direction
step = 0.1;

%% Simulation 
P_ego = P_init; 
alpha = alpha_init;
i_loop = 0;
imax = 20;
Path_cog = P_ego;
flag_obs = 0;
control_points = [];

tic; % Start timer
while norm(P_ego - Pg)>=d_goal && i <=imax
    
    Pobs_tem = [];  % Obstacles that in front of the ego vehicle
    
    for j = 1 : size(Pobs, 1)
        % If there are obstacles, whose x coordinate is bigger than the x
        % coordinate of the ego vehicle, then it is considered in front of the ego vehicle
        if Pobs(j, 1) - P_ego(1) > 0
            Pobs_tem = [Pobs_tem; Pobs(j,:)];
            flag_obs = 1;
        else
            flag_obs = 0;
        end             
    end     
    
    if flag_obs == 0  
        % No obstacles in front of the ego vehicle, then calculate the control
        % points for the Bezier curve from the position of the ego vehicle
        % and the position of the goal. The Bezier curve begins from the
        % current position of the ego vehicle and ends at the goal, which
        % means these control points will only be calculated once, to avoid
        % discontinuity. 
        % The Bezier curve will be calculated in the for loop (see the end of this while loop)
        P0 = P_ego;
        P1 = [(P_ego(1) + Pg(1))/2, (Pg(1) - P_ego(1))/2  * tan(alpha) + P_ego(2)];
        P2 = [P1(1), Pg(2)];
        P3 = Pg;
        control_points = [control_points; P0; P1; P2; P3];
        
    else  % There are obstacles in front of the ego vehicle       
         [x_obs, index] = min(Pobs_tem(:, 1));  % Get the closest obstacle in front of the ego vehicle          
         if x_obs - P_ego(1) <= dist_lon            
         % If the distance from the ego vehicle to the closest obstacle is smaller than the threshold, then change the lane.    
         % Note that the control points are desiged that this Bezier curve 
         % will begin from the position, where the distance of the ego vehicle
         % to the obstable smaller than the thereshold and ends at 1 m away from
         % the x coordinate of the obstacle, so that these points will only be
         % calculated once during the lane change to avoid discontinuity.
         % The Bezier curve will be calculated in the for loop (see the end of this while loop)
            P0 = P_ego;
            P1 = [(P_ego(1) + x_obs)/2, (x_obs - P_ego(1))/2 * tan(alpha) + P_ego(2)];
            if Pobs_tem(index, 2) <= 0
                P2 = [P1(1), lane_width/2];                
            else
                P2 = [P1(1), -lane_width/2];
            end
            P3 = [x_obs+1, P2(2)];
            alpha = 0;  % This is only valid for straight road. For a curve, the yaw angle is given by the
                        % cavature of road.
                        
            control_points = [control_points; P0; P1; P2; P3];
         else  % The distance from the ego vehicle to the closest obstacle is bigger than the threshold
               % The ego vehicle just goes straight forward with the
               % predefined step 
             P_ego = P_ego + step*[cos(alpha), sin(alpha)];
             Path_cog = [Path_cog; P_ego];
             continue  % It goes direct back to the start point of the loop, the Bezier curve will
                       % not be calculated
         end
    end
    
    % Calculate the Bezier curve accroding to the control points
    for t = 0:step:1  
        P_ego = (1-t)^3 * P0 + 3 * t * (1-t)^2 * P1 + 3 * t^2 * (1-t) * P2 + t^3 * P3;
        Path_cog = [Path_cog; P_ego];
    end
    
    P_ego = Path_cog(end,:);
end

toc;  % Stop timer
%% Plot
figure

% Road
% Coordinates begin from bottom left corner with clockwise direction
fig_road = [road_x, -lane_width-0.5;...
                  road_x, lane_width+0.5;...
                  road_x+lane_length, lane_width+0.5;...
                  road_x+lane_length, -lane_width-0.5];  
fill(fig_road(:,1), fig_road(:,2), [0.5 0.5 0.5]);
hold on

% Ego vehicle
fig_veh = [P_init(1)-veh_length/2, P_init(2)-veh_width/2;...
           P_init(1)+veh_length/2, P_init(2)-veh_width/2;...
           P_init(1)+veh_length/2, P_init(2)+veh_width/2;...
           P_init(1)-veh_length/2, P_init(2)+veh_width/2];
fill(fig_veh(:,1), fig_veh(:,2), [0 0.4470 0.7410]);

% Lines
plot([road_x, lane_length], [0, 0], 'w--', 'LineWidth', 2);  % Dashed line
plot([road_x, lane_length], [lane_width, lane_width], 'w', 'LineWidth', 2);  % Left solid line
plot([road_x, lane_length], [-lane_width, -lane_width], 'w', 'LineWidth', 2);  % Right solid line

% Axis settings
axis equal
set(gca, 'XLim', [road_x, lane_length]);
set(gca, 'YLim', [-4, 4]);
title('Path planning using Bezier curve')

% Plot obstacles
cars = [];
for i = 1 : size(Pobs,1)
rr_x = Pobs(i,1) - veh_length/2;
rr_y = Pobs(i,2) - veh_width/2;
rl_x = rr_x;
rl_y = rr_y + veh_width;
fr_x = rr_x + veh_length;
fr_y = rr_y;
fl_x = rr_x + veh_length;
fl_y = rr_y + veh_width;
car = [rr_x,rr_y;...
       rl_x,rl_y;...
       fl_x,fl_y;...
       fr_x,fr_y];
fill(car(:,1), car(:,2), 'yellow')
end
% scatter(Pobs(:,1), Pobs(:,2), '+')

% Plot the goal
scatter(Pg(1), Pg(2), 400, 'p', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r'); 

% Plot the control points
scatter(control_points(:,1), control_points(:,2), 80, 'filled', 'MarkerFaceColor','r', 'MarkerEdgeColor', 'r')



% Plot the path of COG, front left wheel and front right wheel of ego
% vehicle
angle_fl = atan2(veh_width/2, veh_length/2);
angle_fr = angle_fl;
L= sqrt((veh_width/2)^2 + (veh_length/2)^2);
alpha_plot = alpha_init;
beta_fl = angle_fl;
beta_fr = angle_fr;
Path_fl = [ego_rr_x+veh_length, ego_rr_y+veh_width];
Path_fr = [ego_rr_x+veh_length, ego_rr_y];


for i = 2:size(Path_cog,1)
 
    % The calculation of alpha based on the slope of two adjacent points
     alpha_plot = [alpha_plot; atan2(Path_cog(i, 2) - Path_cog(i-1, 2),...
                                        Path_cog(i, 1) - Path_cog(i-1, 1))];
     
     beta_fl = [beta_fl, alpha_plot(i) + angle_fl];
     beta_fr = [beta_fr, alpha_plot(i) - angle_fr];
     Path_fl = [Path_fl; [Path_cog(i, 1) + L * cos(beta_fl(i)),...
                                    Path_cog(i, 2) + L * sin(beta_fl(i))] ];
     Path_fr = [Path_fr; [Path_cog(i, 1) + L * cos(beta_fr(i)),...
                          Path_cog(i, 2) + L * sin(beta_fr(i))]];
end


% Path of the COG of the  ego vehicle
plot(Path_cog(:, 1), Path_cog(:,2), 'color','r', 'linewidth',1.5); 

% Path of the front left wheel of the ego vehicle
plot(Path_fl(:, 1), Path_fl(:,2), 'y', 'linewidth',1.5); 
% Path of the front right wheel of the ego vehicle
plot(Path_fr(:,1), Path_fr(:,2), 'y', 'linewidth', 1.5);  

% % % Animation
% Path_al = animatedline('color','r', 'linewidth', 1.5); 
% 
% 
% 
% 
% Path_fr_al = animatedline('color','blue', 'linewidth', 2);
% Path_fl_al = animatedline('color','blue', 'linewidth', 2);
% for k =1 : size(Path_cog(:,1),1)
%     
%     addpoints(Path_al, Path_cog(k, 1), Path_cog(k,2));
%     addpoints(Path_fr_al, Path_fr(k, 1), Path_fr(k,2));
%     addpoints(Path_fl_al, Path_fl(k, 1), Path_fl(k,2));
%     drawnow
%     pause(0.1)
% end





























