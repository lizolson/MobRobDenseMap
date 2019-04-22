% close all;
% 
% plyfile = dir('C:\Users\adios\Desktop\output_no_move\output\*.ply');
% [fileNum, ~] = size(plyfile);
% 
% for i = 1:fileNum
%     newply = pcread(plyfile(i).name);
% %     location = reshape(newply.Location, [376, 1241, 3]);
% %     color = reshape(newply.Color, [376, 1241, 3]);
%     
%     ptFrame{i} = newply;    
% end
% 
transf = [1 0 0 0;
          0 1 0 0;
          0 0 1 0
          0 0 0 1];
targetpt = ptFrame{1};

num = str2num(plyfile(1).name(9:10));
indfind = find(VFpointcloudexpanded(:,1) == num);
index_last = indfind(1);

for i = 2:fileNum
    
    num = str2num(plyfile(i).name(9:10));
    indfind = find(VFpointcloudexpanded(:,1) == num);
    index = indfind(1);
    
    glopos_last = VFpointcloudexpanded(index(1) - 1,10:12);
    glopos_current = VFpointcloudexpanded(index(1),10:12);
    distance = sqrt(sum((glopos_last - glopos_current).^2));
    distance
    transf(3,4) = transf(3,4) + distance;
    sourceLoc = transf * [ptFrame{i}.Location'; ones(1, size(ptFrame{i}.Location, 1))];
    merge_grid = 0.01;
    ptsource = pointCloud(sourceLoc(1:3,:)', 'Color', ptFrame{i}.Color);
    targetpt = pcmerge(targetpt, ptsource, merge_grid);
    index_last = index(1);
end
% 
figure;
pcshow(targetpt)
xlabel('X')
ylabel('Y')
zlabel('Z')
view([0 -90])
pcwrite(targetpt,'cheat','PLYFormat','binary');