% Starter-code of P1 author: Jose Guivant

% Edited and completed by: z5260252, Gajendra Jayasekera
% Date: 20 / 04 / 2022
% Purpose: This file is the main program for solving Project 2, Part A
% Course: MTRN4010.T1.2022

% =========================================================================

function PartA(file)

% load data, to be played back.
%file = './data015a.mat'; % one of the datasets we can use (simulated data, noise free, for Project 1).
load(file); % will load a variable named data (it is a structure)  
ExploreData(data);
end

function ExploreData(data)
% Here, sensors' measurements are ordered by timestamp.
% as they had occurred, chronologically, when the data was collected/generated
% (in a real experiment o in simulation).
% Each line of this table contains an entry which describes a particular event (e.g. a measurement):
% --> The sampling time of the measurement, the sensor type, and an index to a record in the list of the recorded measurements of that
% particular sensor. So, we use that index to read to the specific measurement.

% Y you may initialize your program, before iterating through the list of events.
hh = InitCertainPartOfMyProgram(data);

X = data.pose0; % platform's initial pose; [x0 (m); y0 (m); heading0 (rads)]
ne = data.n; % how many events?
table = data.table; % table of events.
event = table(:,1); % first event.
t0 = 0.0001 * double(event(1)); % initial time.
vw = [0;0];  % To keep last [speed,heading rate] measurement.
XX = zeros(3,ne,'single'); % a buffer for my results. size=3xne.
etc = data.LidarCfg; % Info about LiDAR installation 
Trail_X = [X(1)];
Trail_Y = [X(2)];
Prev_P_EKF = zeros(3,3); % Initial covariance matrix and place to update it
Estimation_EKF_X = [X(1)]; 
Estimation_EKF_Y = [X(2)];

% Loop: read entries, one by one, for sequential processing.
for i = 1:ne

    XX(:,i) = X;
    
    event = table(:,i);
    sensorID = event(3); % source (i.e. which sensor?)
    t = 0.0001 * double(event(1)); % when was that measurement taken?
    dt = t - t0; % dt since last event (needed for predictions steps). 
    t0 = t;
    here = event(2); % where to read it, from that sensor recorder.
    
    % P1 Part A - Function call
    X = VehiclePose(X, vw, dt); %vw model's inputs (speed and gyroZ)

    % Update plots - cart pose and cart heading
    Trail_X = [Trail_X X(1)];
    Trail_Y = [Trail_Y X(2)];
    set(hh(3),'xdata', Trail_X, 'ydata', Trail_Y); % trail of cart
    heading = [X(1), cos(X(3))+X(1); X(2), sin(X(3))+X(2)];
    set(hh(4), 'xdata', heading(1, :), 'ydata', heading(2, :)); % heading
    
    switch sensorID % measurement from
        case 1 % LiDAR scan
        ranges = data.scans(:,here);

        % P1 Part B/C - Function call
        %tic;
        OOI = ProcessLiDAR(hh, X, ranges, etc, data.Landmarks);
        %toc;
        
        % P2 Part A - Function call
        if OOI.N > 0
            [X, P] = EKF(X, OOI, vw, dt, Prev_P_EKF);
            Prev_P_EKF = P;
            Estimation_EKF_X = [Estimation_EKF_X X(1)];
            Estimation_EKF_Y = [Estimation_EKF_Y X(2)];
            set(hh(6),'xdata', Estimation_EKF_X, 'ydata', Estimation_EKF_Y);
        end

        %{
        % P1 Part D - Function Call
        if OOI.N >= 3
            EP = EstimateVehiclePose(OOI);
            % Update plots - Estimated position of cart
            Estimation_X = [Estimation_X EP(2)];
            Estimation_Y = [Estimation_Y EP(3)];
            set(hh(6),'xdata', Estimation_X, 'ydata', Estimation_Y);
        end
        %}

        pause(dt/10);
        
        %fprintf('LiDAR scan at t=[%d],dt=[%d]\n',t,dt);
        continue;  %"next!"
        
        case 2 % Speed encoder + gyro (they are packed together)
        vw = data.vw(:,here);    % speed and gyroZ, last updated copy.
        %fprintf('new measurement: v=[%.2f]m/s,w=[%.2f]deg/sec\n',vw.*[1;180/pi]);
        continue;
        
        otherwise  % Unknown sensors 
        %fprintf('unknown sensor, type[%d], at t=[%d]\n',sensorID, t);         
        continue;
    end
end

%plot( data.verify.poseL(1,:), data.verify.poseL(2,:),'m+');
disp('Loop of events ends.');
disp('Showing ground truth (you would not achieve that, exactly.)');
ShowVerification1(data);
end

% =============================== Plotting ================================
% Intialise things needed for dynamic plotting

function hh = InitCertainPartOfMyProgram(data)

% For local CF.
figure(10); clf();
r0 = zeros(321,1);       % dummy lidar scan.
h1 = plot(r0,'.');       % h: handle to this graphic object, for subsequent use.  
axis([1,321,0,15]);  % my region of interest, to show.
hold on;     plot([1,321],[10,10],'--r');  % just some line.
zoom on;     % by default, allow zooming in/out
title('LiDAR scans (polar)');  
ylabel('ranges (m)');

% For global CF.
figure(11); clf(); hold on;

% show the map landmarks and, if it is of interest to verify your solution, the
% walls/infrastructure present there.
% (All of them are provided in Global CF)

Landmarks = data.Landmarks;
% plot centres of landmarks. 
%plot(Landmarks(1,:),Landmarks(2,:),'+');
plot(Landmarks(1,:),Landmarks(2,:),'o' ,'color',0*[0,1/3,0])

% some pixels will appear close to some of these crosses. It means he LiDAR scan is
% detecting the associated poles (5cm radius).

% plot interior of walls (they have ~20cm thickness; but the provided info just includes the ideal center of the walls
% your LiDAR scans will appear at ~ 10cm from some of those lines.    
% Wall transversal section:  :  wall left side [<--10cm-->|<--10cm-->] the other  wall side. 
hold on;
plot(data.Walls(1,:),data.Walls(2,:),'color',[0,1,0]*0.7,'linewidth',3);
legend({'Centers of landmarks','Walls (middle planes)'});

title('Global CF');
xlabel('X (m)');
ylabel('Y (m)');
p0 = data.pose0;
plot(p0(1),p0(2),'bs');

h2 = plot(0, 0, '.k'); % lidar scans
h3 = plot(0, 0, '.b'); % cart pose - dead reckoning
h4 = plot(0, 0, '-b'); % cart heading
h5 = plot(0, 0, '+r'); % OOI
h6 = plot(0, 0, '*c'); % estiamted pose - EKF

legend({'Landmarks','Walls (middle planes)','Initial pose','LiDAR scans', ...
    'Cart position','Cart heading','OOI','Estimated EKF Position'});

hh = [h1 h2 h3 h4 h5 h6]; % array of handles you may want to use in other parts of the program.
end

% =========================== Verification Plot ===========================
% Plots locations of real objects ie walls/OOIs to verify user scans and
% processing

function ShowVerification1(data)

% plot some provided verification points (of platfom's pose).
% those are the ground truth.
% Do not expect your solution path to intesect those points, as those are
% the real positions, and yours are approximate ones, based on
% predictions, and using sampled inputs. 
% The discrepancy should be just fraction of cm, as the inputs are not
% polluted by noise, and the simulated model is the nominal analog model.
% The errors are mostly due to time discretization and sampled inputs.
% Inputs were sampled @100Hz (10ms) (you can infer that from "dt".
figure(11)
hold on;
p = data.verify.poseL;
plot(p(1,:),p(2,:),'r.');
h = legend({'Landmarks','Walls (middle planes)','Initial pose','Ground truth (subsampled)'});
end

% =============================== P1 Part A ===============================
% Implements a dead-reckoning process which generates predicition of the 
% platforms position based on a kinematic model which takes speed and 
% angluar rate/yaw rate measurements as inputs

function X = VehiclePose(pose_curr, vw, dt)
    % X - [x; y; heading], vw - [speed, angular rate]
    h = pose_curr(3);
    dpose = [vw(1)*cos(h); vw(1)*sin(h); vw(2)];
    X = pose_curr + dt*dpose;
end

% ============================= Part B and C ==============================
% Processes the LiDAR scans, getting the polar coordinates, converting
% polar to cartesian, translating the point from the LiDARs LCF to the Cars
% LCF to the GCF and identify the CoG of OOIS in cartesian in LiDAR CF.

function OOI = ProcessLiDAR(hh, X, ranges, etc, Landmarks)
    % Process scans
    [rr, aa] = get_polar(ranges);
    set(hh(1),'ydata',rr); % Update LiDAR polar
    lp = polar_to_cartesian(rr, aa);
    cp(1, :) = lp(1, :) + etc.Lx; % LiDAR CF -> Cart CF
    cp(2, :) = lp(2, :) + etc.Ly;
    gp = local_to_global(X, cp); % Cart CF -> GCF
    set(hh(2), 'xdata', gp(1, :), 'ydata', gp(2, :)); % Update LiDAR GCF
    
    % Extract OOI coordinates in cartesian in LiDAR CF
    OOI = ExtractOOIs(lp(1, :), lp(2, :), etc);

    if OOI.N > 0
        % Convert LiDAR CF OOIs to GCF
        OOI.CoG(1, :) = OOI.CoG(1, :) + etc.Lx;
        OOI.CoG(2, :) = OOI.CoG(2, :) + etc.Ly;
        OOI.CoG = local_to_global(X, OOI.CoG);

        % Data association process - 
        % add labels to OOI and remove incorrect OOI
        OOI = LabelOOIs(OOI, Landmarks);
        % Double check that OOIs arent 0 after filtering then update OOIs CoG
        if OOI.N > 0
            %fprintf('Currently detecting %d landmark(s)\n',OOI.N);
            %disp('With detected CoG(s)(X; Y) at');
            %disp(OOI.CoG);
            %disp('Associated to Landmark(s)');
            %disp(OOI.Label(1,:));
            %disp('With known CoG(s)(X; Y) at');
            %disp(OOI.Label(2:3,:));
            %disp('Distances');
            %disp(OOI.Distance_to_OOI);
            
            set(hh(5), 'xdata', OOI.CoG(1,:), 'ydata', OOI.CoG(2,:)); 
        end
    else
        set(hh(5), 'xdata', [], 'ydata', []);
    end
end

% Extract OOI coordinates in cartesian in LiDAR CF
% Performs some basic filtering based on expected diameter of landmarks
% Returns OOI which is a struct
function OOI = ExtractOOIs(X, Y, etc)
    % Struct to store info on OOIs in scans
    OOI.N = 0; % number of OOIs detected
    OOI.CoG = []; % CoG of the OOIs detected
    OOI.Distance_to_OOI = []; % distance from CART to OOI (includes the [0;0.4] translation)

    threshold = 0.6; % threshold distance for point clustering

    cluster_X = X(1); % to store X points in cluster
    cluster_Y = Y(1); % to store y points in cluster
    
    % For the length of the scan
    for i = 2:length(X)
        % Check that adjacent points are within the threshold
        % if within add to current cluster
        if sqrt((X(i) - X(i-1))^2 + (Y(i) - Y(i-1))^2) < threshold
            cluster_X = [cluster_X X(i)];
            cluster_Y = [cluster_Y Y(i)];
        % if not within threshold, a jump is detected 
        else
            % Process the clusters
            % if its only 1 point in hte cluster as it is for OOI that are
            % further away this takes the COG from the point
            if size(cluster_X, 2) == 1
                OOI.N = OOI.N + 1;
                OOI.CoG(:,OOI.N) = [cluster_X; cluster_Y];
            % midpoint method for 2 points in cluster
            elseif size(cluster_X, 2) == 2
                OOI.N = OOI.N + 1;
                OOI.CoG(:,OOI.N) = [(cluster_X(2) + cluster_X(1))/2; (cluster_Y(2) + cluster_Y(1))/2];
            elseif size(cluster_X, 2) >= 3
                [xc, yc, R] = circfit(cluster_X,cluster_Y);
                % Check if cluster fits the defined size of 5-20cm diameter
                % for an OOI
                % changed it to be a bit lower
                if (2 * R > 0.00) && (2 * R < 0.2)
                    OOI.N = OOI.N + 1;
                    OOI.CoG(:,OOI.N) = [xc; yc];
                end
            end

            % Get distance from lidar to detected OOI
            if OOI.N > 0
                OOI.Distance_to_OOI(OOI.N) = [sqrt((-etc.Lx - OOI.CoG(1,OOI.N))^2 + (etc.Ly - OOI.CoG(2,OOI.N))^2)];
            end
             % Reset for new cluster
            cluster_X = [X(i)];
            cluster_Y = [Y(i)];
        end
    end
end

% Label OOI with associated landmarks and filter out false poles
function OOI = LabelOOIs(OOI, Landmarks)
    OOI.Label = []; % to store the index of the Landmark from data
    Keep = 0;
    threshold = 0.4; % This was 0.05 in P1, was changed to this to be less strict in P2
    for i = OOI.N:-1:1
        for j = 1:length(Landmarks)
            % add a label
            if sqrt((Landmarks(1,j) - OOI.CoG(1,i))^2 + (Landmarks(2,j) - OOI.CoG(2,i))^2) < threshold
                OOI.Label(:,i) = [j; Landmarks(1,j); Landmarks(2,j)];
                Keep = 1;
            end
        end
        % remove if no label
        if ~Keep
            if ~isempty(OOI.Label)
                OOI.Label(:,i) = [];
            end
            OOI.Distance_to_OOI(i) = [];
            OOI.CoG(:,i) = [];
            OOI.N = OOI.N - 1;
        end
        Keep = 0;      
    end
end

% ====================== P1 Part D - NOT USED IN P2 =======================
% Estimating the platformEs pose (or at least its position) based on 
% trilateration and/ or triangulation, when, from a LiDAR scan, enough 
% number of OOIs are detected and associated to map landmarks.

%{
function EP = EstimateVehiclePose(OOI)
    P = OOI.Label(2:3,:);
    S = OOI.Distance_to_OOI;
    EP = Trilateration(P, S);
end
%}

% =============================== P2 Part A ===============================
% Implements a EKF based localizer based on kinematic model and LiDAR 
% measurements.

% NOTE: Most of the code for this section was adapted from TUT 7.4 and TUT 
% 8.1 code found at bit.ly/MTRN4010 by Tutor Janzen Choi

function [X, P] = EKF(X, OOI, vw, dt, Prev_P_EKF)
    % Standard Deviations
    % Observations
    sd_range = 0.1; % m 
    sd_bearing = deg2rad(2);
    % Inputs, U
    sd_v = 0.05; % m/s
    sd_w = deg2rad(1); % rad/s
  
    % Covariance Matrices
    P = Prev_P_EKF;            % State vector covariance
    Pu = 50*[sd_v^2,     0;    % Inputs noise covariance - Times 50 to get better prediction
                  0, sd_w^2];  %                           without it, predictions stop after turning

    % Additional Parameters
    v = vw(1);
    % Predicted OOI
    xx_p = OOI.CoG(1,:);
    yy_p = OOI.CoG(2,:);
    % Actual OOI
    xx_a = OOI.Label(2,:);
    yy_a = OOI.Label(3,:);

    % Conduct EKF
    J = get_J(X, dt, v);
    Ju = get_Ju(X, dt);
    P = predict_P(J, Ju, P, Pu);
    [X, P] = update_EKF(X, P, xx_p, yy_p, xx_a, yy_a, sd_range, sd_bearing);

    % Get Jacobian of process model and state vector, dF/dX, N x N
    function J = get_J(X, dt, v)
        J = [1, 0, -dt*v*sin(X(3)); 
             0, 1,  dt*v*cos(X(3)); 
             0, 0,              1];
    end

    % Get Jacobian of process model and inputs, dF/dU, N x M
    function Ju = get_Ju(X, dt)
        Ju = [dt*cos(X(3)), 0; 
              dt*sin(X(3)), 0; 
                         0  dt];
    end

    % Predict covariance matrix P
    function P = predict_P(J, Ju, P, Pu)
        Q = J*P*J';
        Qu = Ju*Pu*Ju';
        P = Q + Qu;
    end

    % Updates the EKF
    function [X, P] = update_EKF(X, P, xx_p, yy_p, xx_a, yy_a, sd_range, sd_bearing)
        for i = 1:length(xx_p)
            % Get distance and angle from predicted pole
            dx_p = xx_p(i) - X(1);
            dy_p = yy_p(i) - X(2);
            range_p = sqrt(dx_p^2 + dy_p^2);
            bearing_p = atan2(dy_p, dx_p) - X(3);
            
            % Get distance and angle from actual pole
            dx_a = xx_a(i) - X(1);
            dy_a = yy_a(i) - X(2);
            range_a = sqrt(dx_a^2 + dy_a^2);
            bearing_a = atan2(dy_a, dx_a) - X(3);
            
            % Compute Jacobian
            H = [-dx_a /   range_a, -dy_a /   range_a, 0;
                  dy_a / range_a^2, -dx_a / range_a^2, -1];
    
            % Get discrepancy vector
            d_range = range_p - range_a;
            d_angle = wrapToPi(bearing_p - bearing_a);
            Z = [d_range;
                 d_angle];
            
            % Get covariance of observational model noise
            R = [sd_range^2,  0;
                 0, sd_bearing^2];
            
            % Perform the KF Update
            S = R + H*P*H';
            K = P*H'*inv(S);
            P = P - K*H*P;
            X = X + K*Z;
        end
    end
end

% =========================== Helper Functions ============================

% Get polar coordinates of LiDAR scan -
function [rr, aa] = get_polar(ranges)
    rr = single(ranges)*0.01; % cm to m
    aa = deg2rad(-80:0.5:80); % 321 points
    ii = find((rr > 1) & (rr < 20));
    rr = rr(ii)';
    aa = aa(ii);
end

% Convert polar to cartesian -
function lp = polar_to_cartesian(rr, aa)
    xx = rr.*cos(aa);
    yy = rr.*sin(aa);
    lp = [xx; yy];
end

% Converts local frame to global frame -
function pp = local_to_global(pose, pp)
    h = pose(3);
    R = [[cos(h), -sin(h)]; [sin(h), cos(h)]];
    pp = R * pp;
    pp(1, :) = pp(1, :) + pose(1);
    pp(2, :) = pp(2, :) + pose(2);
end

% ========================= Functions by others ===========================

% By: Izhak Bucher 25/Oct/1991,
% https://au.mathworks.com/matlabcentral/fileexchange/5557-circle-fit
% Given a few points, finds the best CoG for a circle -
function [xc,yc,R] = circfit(x,y)
   x = x(:); y=y(:);
   a = [x y ones(size(x))]\[-(x.^2+y.^2)];
   xc = -.5*a(1);
   yc = -.5*a(2);
   R = sqrt((a(1)^2+a(2)^2)/4-a(3));
end

% By: Abdelmoumen Norrdine 09/Jun/2016,
% https://au.mathworks.com/matlabcentral/fileexchange/54680-trilateration-code
% An approach for solving nonlinear problems on the example of
% trilateration/multilateration is presented -

% NOTE: This function has been heavily altered from its original form
% Note: NOT USED IN P2

%{
function [N1] = Trilateration(P,S)
    A = []; b = [];
    for i1 = 1:size(P,2)
        x = P(1,i1); y = P(2,i1);
        s = S(i1);
        A = [A ; 1 -2*x  -2*y]; 
        b = [b ; s^2-x^2-y^2 ];
    end
    % Take gaussian elimations if = 3
    if size(P,2) == 3
        N1 = A\b;
    % Multilateration approach
    else
        N1 = pinv(A)*b; % Solution without Weights Matrix
    end
    % the matrix pinv(A) depend only on the reference points it could be computed only once
end
%}