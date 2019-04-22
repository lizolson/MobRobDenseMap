close all;

plyfile = dir('C:\Users\adios\Desktop\dispnet\dispnetply500\*.ply');
[fileNum, ~] = size(plyfile);

for i = 1:fileNum
    newply = pcread(plyfile(i).name);
    ptFrame{i} = newply;    
end

angleStore = [];
disStore = [];
orientation = [];

initial = 1;

transf = eye(4);
      
targetpt = ptFrame{initial};

num = str2num(plyfile(initial).name(8:10));
indfind = find(VFpointcloudexpanded(:,1) == num);
index_last = indfind(1);
delta_x = VFpointcloudexpanded(index_last,10) - VFpointcloudexpanded(index_last - 1,10);
delta_y = VFpointcloudexpanded(index_last,11) - VFpointcloudexpanded(index_last - 1,11);
delta_z = VFpointcloudexpanded(index_last,12) - VFpointcloudexpanded(index_last - 1,12);

arrayLast = [delta_x, delta_y, delta_z];
alpha = atan2(delta_y, delta_x);


for i = initial+1: fileNum
%     if i == 78
%         continue;
%     end

    disp('another thing')
    i
    
    num = str2num(plyfile(i).name(8:10));
    
    indfind = find(VFpointcloudexpanded(:,1) == num);
    if ~isempty(indfind)
    index = indfind(1);
    
    glopos_last = VFpointcloudexpanded(index(1) - 1,10:12);
    glopos_current = VFpointcloudexpanded(index(1),10:12);
    distance = sqrt(sum((glopos_last - glopos_current).^2));
    distance
    delta_x = VFpointcloudexpanded(index(1),10) - VFpointcloudexpanded(index_last,10);
    delta_y = VFpointcloudexpanded(index(1),11) - VFpointcloudexpanded(index_last,11);
    delta_z = VFpointcloudexpanded(index(1),12) - VFpointcloudexpanded(index_last,12);

    arrayCurrent = [delta_x, delta_y, delta_z];
    
    verticle = cross(arrayLast, arrayCurrent);
    theta = asin(norm(verticle) / norm(arrayLast) / norm(arrayCurrent));
    if verticle(3) < 0
        theta = -theta;
    end
    angle = theta
    
%     if abs(angle) > 0.3
%         disp('something')
%  
%     else
    %     theta = atan2(delta_y, delta_x);
%     angle = theta - alpha
%     disStore = [disStore; distance];
    if i >= 4
        rot1 = [cos(-angle) 0 sin(-angle) 0;
                0 1 0 0;
                -sin(-angle) 0 cos(-angle) 0;
                0 0 0 1];
        angleStore = [angleStore; angle];
%         rot1 = quaternion([abs(cos(angle/2)), verticle(1)*sin(angle/2)/norm(verticle), verticle(2)*sin(angle/2)/norm(verticle), verticle(3)*sin(angle/2)/norm(verticle)], [0;0;0])
    else
        rot1 = eye(4);
%         angleStore = [angleStore; 0];
    end
    trans1 = [1 0 0 0;
             0 1 0 0;
             0 0 1 distance;
             0 0 0 1];

    sourceLoc = transf * rot1 * trans1 * [ptFrame{i}.Location'; ones(1, size(ptFrame{i}.Location, 1))];
%     merge_grid = 0.01;
    ptsource = pointCloud(sourceLoc(1:3,:)', 'Color', ptFrame{i}.Color);
%     targetpt = pcmerge(targetpt, ptsource, merge_grid);
    index_last = index(1);
    alpha = theta;
    arrayLast = arrayCurrent;
    transf = transf * rot1 * trans1;
    
    if abs(angle) < 0.3

    pcwrite(ptsource, plyfile(i).name(1:10),'PLYFormat','binary');
    end
    end
 end

% figure;
% pcshow(targetpt)
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% view([0 -90])
% pcwrite(targetpt, 'withodometry', 'PLYFormat', 'binary');