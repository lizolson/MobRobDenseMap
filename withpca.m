% close all;
% 
% % plyfile = dir('C:\Users\adios\Desktop\530project\output\*.ply');
% % [fileNum, ~] = size(plyfile);
% % 
% % for i = 1:fileNum
% %     newply = pcread(plyfile(i).name);
% %     location = reshape(newply.Location, [376, 1241, 3]);
% %     color = reshape(newply.Color, [376, 1241, 3]);
% %     
% %     ptFrame{i} = pointCloud(location, 'Color', color);
% % end
% 
% i=2
% 
% [m,n,k] = size(ptFrame{i}.Location);
% pos = reshape(ptFrame{i}.Location, [m*n, 3]);
% 
% 
% 
% mu_x = mean(pos((pos(:,1) ~= 0),1));
% mu_y = mean(pos((pos(:,2) ~= 0),2));
% mu_z = mean(pos((pos(:,3) ~= 0),3));
% 
% direction = pca(pos);
% a = 5;
% figure;
% pcshow(ptFrame{i});
% x = [mu_x, mu_x + a*direction(1,1)];
% y = [mu_y, mu_y + a*direction(2,1)];
% z = [mu_z, mu_z + a*direction(3,1)];
% 
% x2 = [mu_x, mu_x + a*direction(1,2)];
% y2 = [mu_y, mu_y + a*direction(2,2)];
% z2 = [mu_z, mu_z + a*direction(3,2)];
% 
% hold on;
% plot3(x,y,z,'b','LineWidth',3);
% plot3(x2,y2,z2,'r','LineWidth',3);
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

