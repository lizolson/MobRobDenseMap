close all;

plyfile = dir('C:\Users\adios\Desktop\output_new_normal\output/*.ply');
[fileNum, ~] = size(plyfile);

for i = 1:fileNum
    ptFrame{i} = pcread(plyfile(i).name);
%     ptFrame{i} = pointCloud(location, 'Color', color);
end
ptCloudScene = ptFrame{1};
accumTform = eye(4);
flag = 1;

for j = 1:10
    j

%     newptFramelocation = ptFrame{j}.Location(all(ptFrame{j}.Location ~= 0, 2),:);
%     newptFramecolor = ptFrame{j}.Color(all(ptFrame{j}.Location ~= 0, 2),:);
%     newptFramelocation2 = ptFrame{j+1}.Location(all(ptFrame{j+1}.Location ~= 0, 2),:);
%     newptFramecolor2 = ptFrame{j+1}.Color(all(ptFrame{j+1}.Location ~= 0, 2),:);
% 
%     targetFrame = pointCloud(newptFramelocation, 'Color', newptFramecolor);
%     sourceFrame = pointCloud(newptFramelocation2, 'Color', newptFramecolor2);
    
    targetFrame = ptFrame{j};
    sourceFrame = ptFrame{j+1};

    targetNum = str2num(plyfile(j).name(9:10));
    tarIndfind = find(VFbrief(:,1) == targetNum);

    sourceNum = str2num(plyfile(j+1).name(9:10));
    sourceIndfind = find(VFbrief(:,1) == sourceNum);

    targetFeatureId = VFbrief(tarIndfind,4);
    sourceFeatureId = VFbrief(sourceIndfind,4);

    sameId = [];
    for i = 1:length(targetFeatureId)
        if sum(sourceFeatureId == targetFeatureId(i))
            sameId = [sameId; targetFeatureId(i)];
        end
    end

    targetLocation = [];
    targetColor = [];
    sourceLocation = [];
    sourceColor = [];

    for i = 1:length(sameId)


        sameIndexTargetindind = find(VFbrief(tarIndfind, 4) == sameId(i));
        sameIndexSourceindind = find(VFbrief(sourceIndfind, 4) == sameId(i));

        sameIndexTarget = tarIndfind(sameIndexTargetindind);
        sameIndexSource = sourceIndfind(sameIndexSourceindind);

        pixelTarget = [floor(VFbrief(sameIndexTarget, 6)), floor(VFbrief(sameIndexTarget, 5))];
        pixelSource = [floor(VFbrief(sameIndexSource, 6)), floor(VFbrief(sameIndexSource, 5))];

        arrpos1 = (pixelTarget(1) - 1) * 376 + pixelTarget(2);
        arrpos2 = (pixelSource(1) - 1) * 376 + pixelSource(2);

        targetptCloudLocation = targetFrame.Location(arrpos1,:);
        targetptCloudColor = targetFrame.Color(arrpos1,:);

        sourceptCloudLocation = sourceFrame.Location(arrpos2,:);
        sourceptCloudColor = sourceFrame.Color(arrpos2,:);

        if (sum(targetptCloudLocation ~= 0)~=0) && (sum(sourceptCloudLocation ~= 0)~=0)
            targetLocation = [targetLocation; targetptCloudLocation];
            targetColor = [targetColor; targetptCloudColor];
            sourceLocation = [sourceLocation; sourceptCloudLocation];
            sourceColor = [sourceColor; sourceptCloudColor];
        end
%         
%         arrpos1 = (pixelTarget(1)) * 376 + pixelTarget(2)+1;
%         arrpos2 = (pixelSource(1)) * 376 + pixelSource(2)+1;
% 
%         targetptCloudLocation = targetFrame.Location(arrpos1,:);
%         targetptCloudColor = targetFrame.Color(arrpos1,:);
% 
%         sourceptCloudLocation = sourceFrame.Location(arrpos2,:);
%         sourceptCloudColor = sourceFrame.Color(arrpos2,:);
% 
%         if (sum(targetptCloudLocation ~= 0)~=0) && (sum(sourceptCloudLocation ~= 0)~=0)
%             targetLocation = [targetLocation; targetptCloudLocation];
%             targetColor = [targetColor; targetptCloudColor];
%             sourceLocation = [sourceLocation; sourceptCloudLocation];
%             sourceColor = [sourceColor; sourceptCloudColor];
%         end
% 
    end
%     
    if size(targetLocation,1) <=4
        disp('alert');
    end
% 
    targetPt = pointCloud(targetLocation, 'Color', targetColor);
    sourcePt = pointCloud(sourceLocation, 'Color', sourceColor);
% 
    tform = pcregistericp(targetPt, sourcePt);
% %     tform = pcregistericp(targetFrame, sourceFrame);
    if flag == 1
        flag = 0;
        accumTform = tform;
    else
        accumTform = affine3d(tform.T * accumTform.T);
    end
    
    ptCloudTformed = pctransform(sourceFrame,accumTform);
    
    gridStep = 0.01;
    ptCloudScene = pcmerge(ptCloudScene, ptCloudTformed, gridStep);
%     

end


% figure;
% pcshow(targetFrame);
% hold on;
% pcshow(sourceFrame);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% 
% figure;
% pcshow(targetFrame);
% hold on;
% pcshow(ptCloudTformed);
% xlabel('X');
% ylabel('Y');
% zlabel('Z');

figure;
pcshow(ptCloudScene)