function [x_offset, y_offset, marker_id] = detectAndEstimate(R, G, B)
%#codegen
%
% Detects ArUco markers in the camera frame and estimates the position
% of the highest-priority marker relative to the camera center.
%
% Inputs:
%   R, G, B - uint8 [480×640] matrices from V4L2 Video Capture
%
% Outputs:
%   x_offset  - double. Horizontal offset in meters (positive = right of center)
%   y_offset  - double. Vertical offset in meters (positive = below center / forward)
%   marker_id - double. Detected marker ID (101, 102) or 0 if none found

    % ── Combine R, G, B planes into a 3D RGB image ──
    % Handle potential dimension transpose from V4L2 block [640x480] vs [480x640]
    if size(R, 1) == 640 && size(R, 2) == 480
        R_plane = R';
        G_plane = G';
        B_plane = B';
    else
        R_plane = R;
        G_plane = G;
        B_plane = B;
    end
    frame = cat(3, R_plane, G_plane, B_plane);

    % ── Camera intrinsics (from calibration) ──
    focalLength    = [530.0, 530.0];     % [fx, fy] in pixels
    principalPoint = [320.0, 240.0];     % [cx, cy] in pixels — center of image
    imageSize      = [size(frame, 1), size(frame, 2)]; % [rows, cols]

    intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

    % ── Marker parameters ──
    markerSizeMeters = 0.15;  % 15 cm physical marker size

    % ── Default output: no detection ──
    x_offset  = 0.0;
    y_offset  = 0.0;
    marker_id = 0.0;

    % ── Detect ArUco markers ──
    % Step 1: Check for detected IDs safely
    ids = readArucoMarker(frame, "DICT_ARUCO_ORIGINAL");

    if isempty(ids)
        return;  % No markers found
    end

    % Step 2: Request 3D poses now that at least 1 marker is present
    [ids, ~, poses] = readArucoMarker(frame, "DICT_ARUCO_ORIGINAL", intrinsics, markerSizeMeters);

    % ── Priority selection ──
    % Marker 102 (landing target) has higher priority than 101 (takeoff pad).
    targetIdx = 0;

    % Look for marker 102 first
    for i = 1:length(ids)
        if ids(i) == 102
            targetIdx = i;
            break;
        end
    end

    % If 102 not found, look for marker 101
    if targetIdx == 0
        for i = 1:length(ids)
            if ids(i) == 101
                targetIdx = i;
                break;
            end
        end
    end

    if targetIdx == 0
        return;  % No relevant markers found (ignore unknown IDs)
    end

    % ── Extract translation vector ──
    % poses(i).Translation = [tx, ty, tz] in meters
    % In camera coordinate frame:
    %   tx = right of camera center (positive = right)
    %   ty = below camera center (positive = down)
    %   tz = forward from camera (positive = distance)
    tvec = poses(targetIdx).Translation;

    x_offset  = tvec(1);                 % Right of center (meters)
    y_offset  = tvec(2);                 % Below center (meters)
    marker_id = double(ids(targetIdx));  % 101 or 102
end
