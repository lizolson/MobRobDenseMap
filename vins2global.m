close all;
% 
figure;
pcshow(VFbrief(1:end,7:9));
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('GLOBAL POSE');
% 
% figure;
% pcshow(GPSVIOWGPSTWVIO(1:end,6:8));
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('VIO POSE');
% 
% figure;
% hold on;
% pcshow(VFpointcloudexpanded(1:end,7:9));
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Local Point Cloud');

%% quaternion => Rotation matrix
% TransformedVins = zeros(size(GPSVIOWGPSTWVIO,1),3);
% 
% for i = 1:size(GPSVIOWGPSTWVIO,1)
%     q = GPSVIOWGPSTWVIO(i,12:15);
%     t = GPSVIOWGPSTWVIO(i,9:11)';
%     trans = quaternion(q, t);
%     
%     pos = (trans) * [GPSVIOWGPSTWVIO(i,6:8), 1]';
%     TransformedVins(i,:) = pos(1:3)';
% end
% 
% disp('done!')
%%
% figure;
% pcshow(VFpointcloudexpanded(1:end,7:9));
% hold on;
% % pcshow(TransformedVins);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('New and VIO POSE');
% axis([-450,450,-100,550]);

%%
% m1 = size(VFpointcloudexpanded,1);
% m2 = size(GPSVIOWGPSTWVIO,1);
% color_mat1 = [zeros(m1,2), 255* ones(m1,1)];
% color_mat2 = [255* ones(m1,1), zeros(m1,2)];
% % ptCloudxyz = pointCloud(VFpointcloudexpanded(1:end,7:9), 'Color', color_mat1); 
% ptCloudVins = pointCloud(VFpointcloudexpanded(1:end,7:9), 'Color', color_mat2); 
% % 
% figure;
% % pcshow(ptCloudxyz);
% pcshow(GPSVIOWGPSTWVIO(1:end,3:5));
% hold on;
% % % hold on;
% pcshow(ptCloudVins);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('New and VIO POSE');
% axis([-200,550,-200,550]);

%% Transform pointtclouds
% TransformedVins = zeros(size(VFpointcloudexpanded,1),3);
% 
% for i = 1:size(VFpointcloudexpanded,1)
%     q = VFpointcloudexpanded(i,19:22);
%     t = VFpointcloudexpanded(i,16:18)';
%     trans = quaternion(q, t);
%     
%     pos = (trans) * [VFpointcloudexpanded(i,7:9), 1]';
%     TransformedVins(i,:) = pos(1:3)';
% end
% 
% disp('done!')

%%
% m1 = size(VFpointcloudexpanded,1);
% m2 = size(GPSVIOWGPSTWVIO,1);
% color_mat1 = [255* ones(m1,1), zeros(m1,2)];
% color_mat2 = [zeros(m1,2), 255* ones(m1,1)];%[255* ones(m2,1), zeros(m2,2)];
% % ptCloudxyz = pointCloud(GPSVIOWGPSTWVIO(1:end,3:5), 'Color', color_mat2); 
% ptCloudVins = pointCloud(TransformedVins, 'Color', color_mat1); 
% 
% figure;
% pcshow(ptCloudVins);
% 
% hold on;
% % hold on;
% pcshow(GPSVIOWGPSTWVIO(1:end,3:5));
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% % title('New and VIO POSE');
%  axis([-200,550,-200,600]);
