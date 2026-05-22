clc;
clear all;
close all;

%% generate path
%{.
dis = 1.0;
angle = 27;
deltaAngle = angle / 3;
scale = 0.65;

pathStartAll = zeros(4, 0);
pathAll = zeros(5, 0);
pathList = zeros(5, 0);
pathID = 0;
groupID = 0;

figure;
hold on;
box on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');

fprintf('\nGenerating paths\n');

for shift1 = -angle : deltaAngle : angle
    wayptsStart = [0, 0, 0;
                   dis, shift1, 0];
    
    pathStartR = 0 : 0.01 : dis;
    pathStartShift = spline(wayptsStart(:, 1), wayptsStart(:, 2), pathStartR);
    
    pathStartX = pathStartR .* cos(pathStartShift * pi / 180);
    pathStartY = pathStartR .* sin(pathStartShift * pi / 180);
    pathStartZ = zeros(size(pathStartX));
    
    pathStart = [pathStartX; pathStartY; pathStartZ; ones(size(pathStartX)) * groupID];
    pathStartAll = [pathStartAll, pathStart];
    
    for shift2 = -angle * scale + shift1 : deltaAngle * scale : angle * scale + shift1
        for shift3 = -angle * scale^2 + shift2 : deltaAngle * scale^2 : angle * scale^2 + shift2
                waypts = [pathStartR', pathStartShift', pathStartZ';
                          2 * dis, shift2, 0;
                          3 * dis - 0.001, shift3, 0;
                          3 * dis, shift3, 0];

                pathR = 0 : 0.01 : waypts(end, 1);
                pathShift = spline(waypts(:, 1), waypts(:, 2), pathR);

                pathX = pathR .* cos(pathShift * pi / 180);
                pathY = pathR .* sin(pathShift * pi / 180);
                pathZ = zeros(size(pathX));

                path = [pathX; pathY; pathZ; ones(size(pathX)) * pathID; ones(size(pathX)) * groupID];
                pathAll = [pathAll, path];
                pathList = [pathList, [pathX(end); pathY(end); pathZ(end); pathID; groupID]];
                
                pathID = pathID + 1;

                plot3(pathX, pathY, pathZ);
        end
    end
    
    groupID = groupID + 1
end

pathID

fileID = fopen('startPaths.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathStartAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d\n', pathStartAll);
fclose(fileID);

fileID = fopen('paths.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathAll, 2));
fprintf(fileID, 'property float x\n');
fprintf(fileID, 'property float y\n');
fprintf(fileID, 'property float z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathAll);
fclose(fileID);

fileID = fopen('pathList.ply', 'w');
fprintf(fileID, 'ply\n');
fprintf(fileID, 'format ascii 1.0\n');
fprintf(fileID, 'element vertex %d\n', size(pathList, 2));
fprintf(fileID, 'property float end_x\n');
fprintf(fileID, 'property float end_y\n');
fprintf(fileID, 'property float end_z\n');
fprintf(fileID, 'property int path_id\n');
fprintf(fileID, 'property int group_id\n');
fprintf(fileID, 'end_header\n');
fprintf(fileID, '%f %f %f %d %d\n', pathList);
fclose(fileID);

pause(1.0);
%}

%% find correspondence
%{.
voxelSize = 0.02;
searchRadius = 0.45;     % voxel-grid extent; must match localPlanner.cpp searchRadius
offsetX = 3.2;
offsetY = 4.5;
voxelNumX = 161;
voxelNumY = 451;

% --- Vehicle footprint for collision checking --------------------------------
% The voxel->path corridor is an oriented rectangle (the robot body) swept along
% each path. The footprint is baked inflated by 1/minPathScale so the effective
% world-frame footprint (baked * pathScale) equals the true footprint at the
% minimum path scale and stays conservative above it.
% Keep these in sync with the local_planner configuration:
%   vehicleLength / vehicleWidth -> robot.yaml  vehicle.length / vehicle.width
%   footprintMargin              -> local_planner.launch  vehicleFootprintMargin
%   minPathScale                 -> local_planner.launch  minPathScale
% Changing any of them requires regenerating correspondences.txt.
vehicleLength   = 0.75;
vehicleWidth    = 0.4;
footprintMargin = 0.05;
minPathScale    = 0.675;

bakeHalfL  = (vehicleLength / 2 + footprintMargin) / minPathScale;
bakeHalfW  = (vehicleWidth  / 2 + footprintMargin) / minPathScale;
bakeRadius = sqrt(bakeHalfL^2 + bakeHalfW^2);   % rectangle bounding circle

fprintf('\nPreparing voxels\n');

indPoint = 1;
voxelPointNum = voxelNumX * voxelNumY;
voxelPoints = zeros(voxelPointNum, 2);
for indX = 0 : voxelNumX - 1
    x = offsetX - voxelSize * indX;
    scaleY = x / offsetX + searchRadius / offsetY * (offsetX - x) / offsetX;
    for indY = 0 : voxelNumY - 1
        y = scaleY * (offsetY - voxelSize * indY);

        voxelPoints(indPoint, 1) = x;
        voxelPoints(indPoint, 2) = y;
        
        indPoint  = indPoint + 1;
    end
end

plot3(voxelPoints(:, 1), voxelPoints(:, 2), zeros(voxelPointNum, 1), 'k.');
pause(1.0);

fprintf('\nComputing path sample headings\n');

% Per-sample heading = local path tangent (forward difference within a path).
% The last sample of each path reuses the preceding sample's heading.
numSamples = size(pathAll, 2);
heading = zeros(1, numSamples);
for c = 1 : numSamples - 1
    if pathAll(4, c + 1) == pathAll(4, c)
        heading(c) = atan2(pathAll(2, c + 1) - pathAll(2, c), ...
                           pathAll(1, c + 1) - pathAll(1, c));
    elseif c > 1
        heading(c) = heading(c - 1);
    end
end
if numSamples > 1
    heading(numSamples) = heading(numSamples - 1);
end

fprintf('\nCollision checking (oriented-rectangle sweep)\n');

% Broad phase: candidate path samples within the rectangle's bounding circle.
[ind, ~] = rangesearch(pathAll(1 : 2, :)', voxelPoints, bakeRadius);

fprintf('\nSaving correspondences\n');

fileID = fopen('correspondences.txt', 'w');

for i = 1 : voxelPointNum
    fprintf(fileID, '%d ', i - 1);

    cand = ind{i};
    if ~isempty(cand)
        % Narrow phase: keep candidates whose oriented body rectangle, placed at
        % the path sample and aligned with the local tangent, contains the voxel.
        rx = voxelPoints(i, 1) - pathAll(1, cand);
        ry = voxelPoints(i, 2) - pathAll(2, cand);
        ch = cos(heading(cand));
        sh = sin(heading(cand));
        xLocal =  ch .* rx + sh .* ry;
        yLocal = -sh .* rx + ch .* ry;
        inRect = abs(xLocal) <= bakeHalfL & abs(yLocal) <= bakeHalfW;
        hitPaths = unique(pathAll(4, cand(inRect)));
        for j = 1 : numel(hitPaths)
            fprintf(fileID, '%d ', hitPaths(j));
        end
    end
    fprintf(fileID, '-1\n');

    if mod(i, 1000) == 0
        i
    end
end

fclose(fileID);

fprintf('\nProcessing complete\n');
%}
