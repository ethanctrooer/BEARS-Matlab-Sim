% Modified from https://www.mathworks.com/help/driving/ug/sensor-fusion-using-synthetic-radar-and-vision-data.html#d126e35144

%helper functions:
function filter = initSimDemoFilter(detection)
% Use a 2-D constant velocity model to initialize a trackingKF filter.
% The state vector is [x;vx;y;vy]
% The detection measurement vector is [x;y;vx;vy]
% As a result, the measurement model is H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]
H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
filter = trackingKF('MotionModel', '2D Constant Velocity', ...
    'State', H' * detection.Measurement, ...
    'MeasurementModel', H, ...
    'StateCovariance', H' * detection.MeasurementNoise * H, ...
    'MeasurementNoise', detection.MeasurementNoise);
end

function BEP = createDemoDisplay(egoBike, sensors)
    % Make a figure
    hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Sensor Fusion with Synthetic Data Example');
    movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top

    % Add a car plot that follows the ego vehicle from behind
    hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
    hCarPlot = axes(hCarViewPanel);
    chasePlot(egoBike, 'Parent', hCarPlot);

    % Add a car plot that follows the ego vehicle from a top view
    hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
    hCarPlot = axes(hTopViewPanel);
    chasePlot(egoBike, 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 90);

    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');

    % Create bird's-eye plot for the ego vehicle and sensor coverage
    hBEVPlot = axes(hBEVPanel);
    frontBackLim = 60;
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);

    % Plot the coverage areas for radars
    for i = 1:2%1:6
        cap = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
        if isa(sensors{i},'drivingRadarDataGenerator')
            plotCoverageArea(cap, sensors{i}.MountingLocation(1:2),...
                sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(1), sensors{i}.FieldOfView(1));
        else
            plotCoverageArea(cap, sensors{i}.SensorLocation,...
                sensors{i}.MaxRange, sensors{i}.Yaw, sensors{i}.FieldOfView(1));
        end
    end

    % Plot the coverage areas for vision sensors
    for i = 3%:4 %only plot front facing for now
        cap = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
        if isa(sensors{i},'drivingRadarDataGenerator')
            plotCoverageArea(cap, sensors{i}.MountingLocation(1:2),...
                sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(1), 45);
        else
            plotCoverageArea(cap, sensors{i}.SensorLocation,...
                sensors{i}.MaxRange, sensors{i}.Yaw, 45);
        end
    end

    % Create a vision detection plotter put it in a struct for future use
    detectionPlotter(BEP, 'DisplayName','vision', 'MarkerEdgeColor','blue', 'Marker','^');

    % Combine all radar detections into one entry and store it for later update
    detectionPlotter(BEP, 'DisplayName','radar', 'MarkerEdgeColor','red');

    % Add road borders to plot
    laneMarkingPlotter(BEP, 'DisplayName','lane markings');

    % Add the tracks to the bird's-eye plot. Show last 10 track updates.
    trackPlotter(BEP, 'DisplayName','track', 'HistoryDepth',10);

    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-40 40]);

    % Add an outline plotter for ground truth
    outlinePlotter(BEP, 'Tag', 'Ground truth');
end

function updateBEP(BEP, egoBike, detections, confirmedTracks, psel, vsel)
    % Update road boundaries and their display
    [lmv, lmf] = laneMarkingVertices(egoBike);
    plotLaneMarking(findPlotter(BEP,'DisplayName','lane markings'),lmv,lmf);

    % update ground truth data
    [position, yaw, length, width, originOffset, color] = targetOutlines(egoBike);
    plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);

    % update barrier data
    [bPosition,bYaw,bLength,bWidth,bOriginOffset,bColor,numBarrierSegments] = targetOutlines(egoBike, 'Barriers');
    plotBarrierOutline(findPlotter(BEP,'Tag','Ground truth'),numBarrierSegments,bPosition,bYaw,bLength,bWidth,...
                       'OriginOffset',bOriginOffset,'Color',bColor);

    % Prepare and update detections display
    N = numel(detections);
    detPos = zeros(N,2);
    isRadar = true(N,1);
    for i = 1:N
        detPos(i,:) = detections{i}.Measurement(1:2)';
        if detections{i}.SensorIndex > 6 % Vision detections
            isRadar(i) = false;
        end
    end
    plotDetection(findPlotter(BEP,'DisplayName','vision'), detPos(~isRadar,:));
    plotDetection(findPlotter(BEP,'DisplayName','radar'), detPos(isRadar,:));

    % Remove all object tracks that are unidentified by the vision detection
    % generators before updating the tracks display. These have the ObjectClassID
    % parameter value as 0 and include objects such as barriers.
    isNotBarrier = arrayfun(@(t)t.ObjectClassID,confirmedTracks)>0;
    confirmedTracks = confirmedTracks(isNotBarrier);

    % Prepare and update tracks display
    trackIDs = {confirmedTracks.TrackID};
    labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
    [tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
    tracksVel = getTrackVelocities(confirmedTracks, vsel);
    plotTrack(findPlotter(BEP,'DisplayName','track'), tracksPos, tracksVel, tracksCov, labels);
end

%% START Main Script

% Define an empty scenario.
scenario = drivingScenario;
scenario.SampleTime = 0.01;

roadCenters = [0 0; 50 0; 100 0; 250 20; 500 40];
mainRoad = road(scenario, roadCenters, 'lanes',lanespec(2));
%barrier(scenario,mainRoad);

% Create the ego vehicle that travels at 25 m/s along the road.  Place the
% vehicle on the right lane by subtracting off half a lane width (1.8 m)
% from the centerline of the road.
egoBike = actor(scenario, 'ClassID', 1, 'Length',1.73 ,'Width',1.1,'Height',1.1);
trajectory(egoBike, roadCenters(2:end,:) - [0 1.8], 25); % On right lane

% Add a car in front of the ego vehicle
leadCar = vehicle(scenario, 'ClassID', 1);
trajectory(leadCar, [70 0; roadCenters(3:end,:)] - [0 1.8], 25); % On right lane
% SCENARIO 1 BELOW - front brake
%waypoints = [70 -1.8; 100 -1.8; 250 18.2; 500 38.2];
%speed = [25 25 10 0];
%trajectory(leadCar, waypoints, speed); % On right lane

% Add a car that travels at 35 m/s along the road and passes the ego vehicle
passingCar = vehicle(scenario, 'ClassID', 1);
%waypoints = [0 -1.8; 50 1.8; 100 1.8; 250 21.8; 400 32.2; 500 38.2];
% SCENARIO 2 BELOW - close rear pass
%waypoints = [0 -1.8; 50 1.8; 100 1.8; 250 20; 400 32.2; 500 38.2];
% SCENARIO 3 BELOW - rear takeover collision
waypoints = [0 -1.8; 50 1.8; 100 1.8; 275 17.2; 400 32.2; 500 38.2];
trajectory(passingCar, waypoints, 35);

% Add a car behind the ego vehicle
chaseCar = vehicle(scenario, 'ClassID', 1);
trajectory(chaseCar, [25 0; roadCenters(2:end,:)] - [0 1.8], 25); % On right lane

sensors = cell(4,1);
% Front-facing long-range radar sensor at the center of the front bumper of the car.
%sensors{1} = drivingRadarDataGenerator('SensorIndex', 1, 'RangeLimits', [0 55], ...
%    'MountingLocation', [egoCar.Wheelbase + egoCar.FrontOverhang, 0, 0.2], 'FieldOfView', [100, 120]);
sensors{1} = drivingRadarDataGenerator('SensorIndex', 1, 'RangeLimits', [0 55], ...
    'MountingLocation', [0.85, 0, 1], 'FieldOfView', [100, 120]);


% Rear-facing long-range radar sensor at the center of the rear bumper of the car.
%sensors{2} = drivingRadarDataGenerator('SensorIndex', 2, 'MountingAngles', [180 0 0], ...
%    'MountingLocation', [-egoCar.RearOverhang, 0, 0.2], 'RangeLimits', [0 55], 'FieldOfView', [100, 120]);
sensors{2} = drivingRadarDataGenerator('SensorIndex', 2, 'MountingAngles', [180 0 0], ...
    'MountingLocation', [-0.85, 0, 1], 'RangeLimits', [0 55], 'FieldOfView', [100, 120]);

% NOTE - the vision sensors power the tracks, NOT THE RADAR, in the simulation!
% Front-facing camera located at front windshield.
sensors{3} = visionDetectionGenerator('SensorIndex', 7, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.75 0], 'Height', 1.1);

% Rear-facing camera located at rear windshield.
sensors{4} = visionDetectionGenerator('SensorIndex', 8, 'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.2 0], 'Height', 1.1, 'Yaw', 180);

% Register actor profiles with the sensors.
profiles = actorProfiles(scenario);
for m = 1:numel(sensors)
    if isa(sensors{m},'drivingRadarDataGenerator')
        sensors{m}.Profiles = profiles;
    else
        sensors{m}.ActorProfiles = profiles;
    end
end

tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationThreshold', [4 5]);
positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

% Create the display and return a handle to the bird's-eye plot
BEP = createDemoDisplay(egoBike, sensors);

toSnap = true;
while advance(scenario) && ishghandle(BEP.Parent)
    % Get the scenario time
    time = scenario.SimulationTime;

    % Get the position of the other vehicle in ego vehicle coordinates
    ta = targetPoses(egoBike);

    % Simulate the sensors
    detectionClusters = {};
    isValidTime = false(1,2);
    for i = 1:4
        [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
        if numValidDets
            for j = 1:numValidDets
                % Vision detections do not report SNR. The tracker requires
                % that they have the same object attributes as the radar
                % detections. This adds the SNR object attribute to vision
                % detections and sets it to a NaN.
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end

                % Remove the Z-component of measured position and velocity
                % from the Measurement and MeasurementNoise fields
                sensorDets{j}.Measurement = sensorDets{j}.Measurement([1 2 4 5]);
                sensorDets{j}.MeasurementNoise = sensorDets{j}.MeasurementNoise([1 2 4 5],[1 2 4 5]);
            end
            detectionClusters = [detectionClusters; sensorDets]; %#ok<AGROW>
        end
    end

    % Update the tracker if there are new detections
    if any(isValidTime)
        if isa(sensors{1},'drivingRadarDataGenerator')
            vehicleLength = sensors{1}.Profiles.Length;
        else
            vehicleLength = sensors{1}.ActorProfiles.Length;
        end
        confirmedTracks = updateTracks(tracker, detectionClusters, time);

        %do the thing; this should really be in updateBEP but i can't be bothered lmao
        mostImpObj = findMostImportantObject(confirmedTracks, positionSelector, velocitySelector);
        disp(mostImpObj)
        %disp(strcat(num2str(mostImpObj.ObjectID), " is ", num2str(mostImpObj.Dist), "m away - ", mostImpObj.ThreatColor))
        threatColor = mostImpObj.ThreatColor;
        objDir = mostImpObj.objectDirection;
        %if strcmp(threatColor,'yellow') | strcmp(threatColor,'red')
        if strcmp(threatColor,'red')
            x = mostImpObj.Dist;
            beepFreq = (0.15)/(1+exp(-(0.9*x-5))); % beep faster if obj closer
            % https://www.desmos.com/calculator/ngvjwjsb4n
            if (strcmp(objDir,'front')) % change beeping sound based on front/back
                customBeep(beepFreq, 800);
            else
                customBeep(beepFreq, 1250);
            end
        end

        % Update bird's-eye plot
        updateBEP(BEP, egoBike, detectionClusters, confirmedTracks, positionSelector, velocitySelector);
    end

    % Snap a figure for the document when the car passes the ego vehicle
    if ta(1).Position(1) > 0 && toSnap
        toSnap = false;
        snapnow
    end
end
%% Next bit, modified from bottom of
%https://www.mathworks.com/help/driving/ug/forward-collision-warning-using-sensor-fusion.html
function mostImportantObject = findMostImportantObject(confirmedTracks,positionSelector,velocitySelector)

    % Initialize outputs and parameters
    MIO = [];               % By default, there is no MIO
    trackID = [];           % By default, there is no trackID associated with an MIO
    FCW = 3;                % By default, if there is no MIO, then FCW is 'safe'
    threatColor = 'green';  % By default, the threat color is green
    maxX = 100;  % Far enough forward so that no track is expected to exceed this distance
    gAccel = 9.8; % Constant gravity acceleration, in m/s^2
    maxDeceleration = 0.4 * gAccel; % Euro NCAP AEB definition
    delayTime = 1.2; % Delay time for a driver before starting to brake, in seconds
    objectDirection = [];   % By default, there is no Direction [EC custom added]

    positions = getTrackPositions(confirmedTracks, positionSelector);
    velocities = getTrackVelocities(confirmedTracks, velocitySelector);

    for i = 1:numel(confirmedTracks)
        x = abs(positions(i,1)); % modified to use absolute positions, for rear
        y = abs(positions(i,2));
        
        if confirmedTracks(i).State(1) > 0 % positive if vehicle is AHEAD
            relSpeed = velocities(i,1); % The relative speed between the cars, along the lane
            tempObjectDirection = 'front';
        else % negative if vehicle is BEHIND
            relSpeed = -1*velocities(i,1); % Make negative to make math work (SKETCH)
            tempObjectDirection = 'rear';
        end

        if x < maxX && x > 0 % No point checking otherwise (ONLY FOR FRONT CAR)
            yleftLane  = polyval(1.8,  x); %NOT SURE ABOUT THESE!~!!!!~!
            yrightLane = polyval(-1.8, x);
            if (yrightLane <= y) && (y <= yleftLane)
                maxX = x;
                trackID = i;
                MIO = confirmedTracks(i).TrackID;
                if relSpeed < 0 % Relative speed indicates object is getting closer
                    % Calculate expected braking distance according to
                    % Euro NCAP AEB Test Protocol
                    d = abs(relSpeed) * delayTime + relSpeed^2 / 2 / maxDeceleration;
                    if x <= d % 'warn'
                        FCW = 1;
                        threatColor = 'red';
                        objectDirection = tempObjectDirection; % save direction for mostImp
                    else % 'caution'
                        FCW = 2;
                        threatColor = 'yellow';
                    end
                end
            end
        end
    end
    mostImportantObject = struct('ObjectID', MIO, 'TrackIndex', ...
        trackID, 'Warning', FCW, 'ThreatColor', threatColor, 'Dist', maxX, ...
        'objectDirection', objectDirection);
end
