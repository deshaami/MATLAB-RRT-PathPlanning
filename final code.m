function phase1_demo_v5_fixed()
% Phase-1 demo v5 (corrected): Image-based RRT* + LIDAR + Local Path Following
% Robot (red) tightly follows RRT* path (blue) with mild reactive avoidance.

clc; clear; close all;

disp('🌍 Select a floor-plan image (PNG/JPG/BMP)...');
[filename, pathname] = uigetfile({'*.png;*.jpg;*.bmp','Image Files'});
if isequal(filename,0)
    disp('❌ No image selected.'); return;
end

%% Load and preprocess image
imgPath = fullfile(pathname, filename);
img = imread(imgPath);
if size(img,3)==3, grayImg = rgb2gray(img); else, grayImg = img; end
grayImg = double(grayImg)/255;
bw = grayImg < 0.5;
bw = imresize(bw, [300 400]);
disp('✅ Map loaded successfully.');

figure; imshow(~bw); title('Processed Binary Occupancy Map');
disp('📍 Select START point'); [sx, sy] = ginput(1);
disp('🎯 Select GOAL point'); [gx, gy] = ginput(1);
start = [sx, sy]; goal = [gx, gy];
fprintf('Start = [%.1f %.1f], Goal = [%.1f %.1f]\n', start, goal);

[rows, cols] = size(bw);
mapSize = [0 cols; 0 rows];
xgv = linspace(mapSize(1,1), mapSize(1,2), cols);
ygv = linspace(mapSize(2,1), mapSize(2,2), rows);
isFree = @(pt) ~isOccupied(pt, bw, xgv, ygv);

%% RRT* path planning
disp('🧭 Running RRT*...');
path = rrtstar(start, goal, isFree, [mapSize(1,:) mapSize(2,:)], 1500, 10, 20);
if isempty(path)
    warning('No path found; using straight line.'); path = [start; goal];
end
path = smoothdata(path,'gaussian',7);

%% Simulation setup
robotPose = [start, atan2(path(2,2)-path(1,2), path(2,1)-path(1,1))];
traj = robotPose(1:2);
dt = 0.2; v_nom = 5; maxSteps = 1000;
numBeams = 72; lidarRange = 40;

figure('Position',[100 100 1000 600]);

% === 🎥 VIDEO SETUP ===
v = VideoWriter('phase1_demo_final.mp4','MPEG-4');
v.FrameRate = 30;
open(v);

for step=1:maxSteps
    % Simulated LIDAR
    angles = linspace(-pi,pi,numBeams);
    for i=1:numBeams
        a = angles(i)+robotPose(3);
        r=0;
        while r<lidarRange
            r=r+2;
            pt = robotPose(1:2)+r*[cos(a),sin(a)];
            if ~isFree(pt), break; end
        end
        lidarPts(i,:) = robotPose(1:2)+r*[cos(a),sin(a)];
    end

    % Visualization
    clf; imshow(~bw); hold on;
    plot(path(:,1),path(:,2),'-b','LineWidth',2);
    plot(goal(1),goal(2),'g*','MarkerSize',10);
    plot(traj(:,1),traj(:,2),'-r','LineWidth',1.5);
    plot(lidarPts(:,1),lidarPts(:,2),'.y','MarkerSize',3);
    theta = linspace(0,2*pi,40);
    plot(robotPose(1)+5*cos(theta), robotPose(2)+5*sin(theta),'Color',[0 0.6 0.2],'LineWidth',1);
    title(sprintf('Step %d',step));
    drawnow;

    % === 🎥 Save each frame ===
    frame = getframe(gcf);
    writeVideo(v, frame);

    %% Path-following controller (pure pursuit)
    lookahead = 20;
    dists = vecnorm(path - robotPose(1:2),2,2);
    [~,nearestIdx] = min(dists);
    targetIdx = min(nearestIdx+round(lookahead/5), size(path,1));
    target = path(targetIdx,:);
    dirVec = target - robotPose(1:2);

    desiredTheta = atan2(dirVec(2), dirVec(1));
    angErr = wrapToPi(desiredTheta - robotPose(3));

    % Reactive obstacle avoidance (very mild)
    frontAngles = abs(angles) < pi/6;
    frontDists = vecnorm(lidarPts(frontAngles,:) - robotPose(1:2),2,2);
    minFront = min(frontDists);
    avoidTerm = 0;
    if minFront < 20
        avoidTerm = sign(mean(angles(frontAngles))) * (20 - minFront)/20;
    end

    wcmd = 0.5*angErr + 0.3*avoidTerm;
    vcmd = v_nom * exp(-abs(angErr));

    % Update pose
    nextPose = robotPose;
    nextPose(1) = robotPose(1) + vcmd*dt*cos(robotPose(3));
    nextPose(2) = robotPose(2) + vcmd*dt*sin(robotPose(3));
    nextPose(3) = wrapToPi(robotPose(3) + wcmd*dt);

    if isOccupied(nextPose(1:2),bw,xgv,ygv)
        vcmd=0; nextPose(3)=wrapToPi(robotPose(3)+0.2*randn);
    end

    robotPose=nextPose;
    traj=[traj; robotPose(1:2)];

    if norm(robotPose(1:2)-goal)<10
        disp('🎯 Goal reached!'); break;
    end
end

% === Close video ===
close(v);
disp('✅ Simulation complete. Video saved as phase1_demo_final.mp4');

end

%% Helper: Occupancy check
function occ=isOccupied(pt,bw,xgv,ygv)
[rows,cols]=size(bw);
xi=round((pt(1)-xgv(1))/(xgv(end)-xgv(1))*(cols-1))+1;
yi=round((pt(2)-ygv(1))/(ygv(end)-ygv(1))*(rows-1))+1;
if xi<1||xi>cols||yi<1||yi>rows, occ=true; else, occ=bw(yi,xi)==1; end
end

%% Helper: RRT* path planner
function path=rrtstar(start,goal,mapFcn,bounds,nSamples,stepSize,neighR)
nodes=start; parent=0; cost=0;
for iter=1:nSamples
    if rand<0.1, sample=goal+0.5*(rand(1,2)-0.5);
    else, sample=[bounds(1)+rand*(bounds(2)-bounds(1)), bounds(3)+rand*(bounds(4)-bounds(3))];
    end
    dists=sqrt(sum((nodes-sample).^2,2));
    [~,idx]=min(dists);
    direction=sample-nodes(idx,:);
    if norm(direction)==0, continue; end
    extension=nodes(idx,:)+stepSize*direction/norm(direction);
    if ~mapFcn((nodes(idx,:)+extension)/2)||~mapFcn(extension), continue; end
    nodes=[nodes;extension];
    parent(end+1)=idx;
    cost(end+1)=cost(idx)+norm(extension-nodes(idx,:));

    neighIdx=find(sqrt(sum((nodes-extension).^2,2))<neighR);
    for ni=neighIdx'
        potentialCost=cost(end)+norm(nodes(ni,:)-extension);
        if potentialCost<cost(ni)&&mapFcn((nodes(ni,:)+extension)/2)
            parent(ni)=size(nodes,1);
            cost(ni)=potentialCost;
        end
    end

    if norm(extension-goal)<stepSize&&mapFcn((extension+goal)/2)&&mapFcn(goal)
        nodes=[nodes;goal];
        parent(end+1)=size(nodes,1)-1;
        break;
    end
end

if norm(nodes(end,:)-goal)<1e-6
    p=size(nodes,1); path=nodes(p,:);
    while parent(p)>0, p=parent(p); path=[nodes(p,:); path]; end
else, path=[];
end
end
