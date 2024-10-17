% Initialize RealSense pipeline
close; clc; clear;
pipe = realsense.pipeline();
config = realsense.config();
config.enable_stream(realsense.stream.color, 1920, 1080, realsense.format.rgb8, 30);
profile = pipe.start(config);

% Define camera intrinsics
focalLength = [966.176, 966.176];
principalPoint = [972.006, 503.704];
imageSize = [1080, 1920];
Intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Create a UDP object for communication with ESP32
esp32IP = '192.168.167.99'; % Replace with the IP address of your ESP32
esp32Port = 8888;
u = udp(esp32IP, 'RemotePort', esp32Port);
fopen(u);

errprev = 0;

figure;

while true
    fs = pipe.wait_for_frames();
    color_frame = fs.get_color_frame();
    data = color_frame.get_data();
    img = permute(reshape(data', [3, color_frame.get_width(), color_frame.get_height()]), [3 2 1]);
    [id, loc, pose] = readAprilTag(img, 'tag36h11', Intrinsics, 0.141);

    imshow(img);
    hold on;

    idx0 = find(id == 0);

    if ~isempty(idx0)
        R0 = pose(idx0).Rotation;
        angle_z0 = atan2d(R0(2,1), R0(1,1));

        for i = 1:length(id)
            if id(i) ~= 0 && id(i) ~= 3 && id(i) ~= 4 % Only new markers
                R = pose(i).Rotation;
                angle_z = atan2d(R(2,1), R(1,1)) - angle_z0;

                t_global = pose(i).Translation;
                t_local = t_global - pose(idx0).Translation;

                plot(loc(:,1,i), loc(:,2,i), 'g-', 'LineWidth', 2);
                center = mean(loc(:,:,i), 1);
                plot(center(1), center(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

                text(center(1), center(2), sprintf('ID: %d\nX: %.2f\nY: %.2f\nZ: %.2f\nAngle: %.2f°', id(i), t_local(1), t_local(2), t_local(3), angle_z), 'Color', 'yellow', 'FontSize', 12);

                % Send angle to ESP32
                kp = 10;
                kd = 5;
                error = -90-angle_z;
                errd = error-errprev;
                pwm = (-90-angle_z)*kp + kd*errd;
                errprev = error;
                fwrite(u, num2str(pwm));
                % fwrite(u, single(angle_z), 'single');
                disp(pwm);
            end
        end
    end

    hold off;
    pause(0.01);
end

pipe.stop();
fclose(u);
delete(u);
clear u;
