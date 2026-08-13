% desktop_prototype_static.m
% Tests ArUco marker detection on a static photo before building Simulink model

testImageFile = 'test_marker.jpg';
if ~exist(testImageFile, 'file')
    error('File %s not found. Take a photo of the printed marker and save as %s.', testImageFile, testImageFile);
end

I = imread(testImageFile);

% Load camera parameters or use fallback intrinsics
if exist('cameraParams.mat', 'file')
    load('cameraParams.mat', 'cameraParams');
    intrinsics = cameraParams.Intrinsics;
else
    warning('cameraParams.mat not found. Using default camera intrinsics [530, 530].');
    intrinsics = cameraIntrinsics([530.0, 530.0], [320.0, 240.0], [size(I,1), size(I,2)]);
end

markerSizeMeters = 0.15; % 15 cm physical marker

[ids, locs] = readArucoMarker(I, "DICT_ARUCO_ORIGINAL");

if ~isempty(ids)
    [ids, locs, poses] = readArucoMarker(I, "DICT_ARUCO_ORIGINAL", intrinsics, markerSizeMeters);
else
    poses = [];
end

fprintf('\n=== ArUco Detection Results ===\n');
fprintf('Found %d marker(s)\n', length(ids));

figure('Name', 'Static Image ArUco Test');
imshow(I); hold on;

for i = 1:length(ids)
    tvec = poses(i).Translation;
    fprintf('Marker ID %d -> x: %.3f m, y: %.3f m, z (distance): %.3f m\n', ...
        ids(i), tvec(1), tvec(2), tvec(3));
    
    corners = squeeze(locs(i, :, :));
    plot([corners(:,1); corners(1,1)], [corners(:,2); corners(1,2)], 'g-', 'LineWidth', 2);
    text(corners(1,1), corners(1,2)-10, sprintf('ID:%d x:%.2fm y:%.2fm', ids(i), tvec(1), tvec(2)), ...
        'Color', 'yellow', 'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'black');
end
hold off;
