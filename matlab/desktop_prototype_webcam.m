% desktop_prototype_webcam.m
% Real-time ArUco detection and pose estimation using a live desktop webcam

try
    cam = webcam;
catch ME
    error('Could not connect to webcam. Ensure a webcam is plugged in and Image Acquisition Toolbox is active.');
end

if exist('cameraParams.mat', 'file')
    load('cameraParams.mat', 'cameraParams');
    intrinsics = cameraParams.Intrinsics;
else
    warning('cameraParams.mat not found. Using default camera intrinsics [530, 530].');
    sampleFrame = snapshot(cam);
    intrinsics = cameraIntrinsics([530.0, 530.0], [320.0, 240.0], [size(sampleFrame,1), size(sampleFrame,2)]);
end

markerSizeMeters = 0.15; % 15 cm physical marker

figure('Name', 'Live Webcam ArUco Prototype (Press Ctrl+C in Command Window to stop)');

while true
    frame = snapshot(cam);
    
    % Step 1: Detect IDs safely (2 outputs does not trigger internal pose calculation crash on empty frames)
    [ids, locs] = readArucoMarker(frame, "DICT_ARUCO_ORIGINAL");

    imshow(frame); hold on;
    if ~isempty(ids)
        % Step 2: Request 3D poses only when markers are detected
        [ids, locs, poses] = readArucoMarker(frame, "DICT_ARUCO_ORIGINAL", intrinsics, markerSizeMeters);
        
        for i = 1:length(ids)
            corners = squeeze(locs(i, :, :));
            plot([corners(:,1); corners(1,1)], [corners(:,2); corners(1,2)], 'g-', 'LineWidth', 2);
            if ~isempty(poses) && i <= length(poses)
                tvec = poses(i).Translation;
                text(corners(1,1), corners(1,2)-10, ...
                    sprintf('ID:%d  x:%.2f y:%.2f z:%.2f', ids(i), tvec(1), tvec(2), tvec(3)), ...
                    'Color', 'yellow', 'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'black');
            end
        end
    end
    hold off;
    drawnow;
end
