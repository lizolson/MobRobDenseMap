test1 = pcread('0000000016.ply');
figure;
pcshow(test1);
xlabel('X')
ylabel('Y')
zlabel('Z')
view([0 -90])

close all;
plyfile = dir('*.ply');
[fileNum, ~] = size(plyfile);

for i = 1:2 %fileNum
    name = plyfile(1).name;
    time = str2double(name(end-5:end-4));
    newply = pcread(plyfile(i).name);
    
    ptFrame{i} = pointCloud(location, 'Color', color);
end