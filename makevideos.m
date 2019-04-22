close all;

plyfile = dir('C:\Users\adios\Desktop\output_no_move\output/*.ply');
[fileNum, ~] = size(plyfile);

for i = 1:fileNum
    newply = pcread(plyfile(i).name);
    ptFrame{i} = newply;    
end

pauseLen = 0.0;
makeVideo = true;

if makeVideo
    try
        votype = 'avifile';
        vo = avifile('video.avi', 'fps', min(10, 1/pauseLen));
    catch
        votype = 'VideoWriter';
        vo = VideoWriter('video', 'MPEG-4');
        set(vo, 'FrameRate', min(10, 1/pauseLen));
        open(vo);
    end
end

figure(1); set(gcf, 'position', [30 30 700 1500]);
for i = 1:fileNum
    figure(1);
    pcshow(ptFrame{i});
    view([0, 0])
    hold on;
    axis([-25 100 -5 5 0 260])
    
    

        drawnow;
    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end

    if makeVideo
        F = getframe(gcf);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
end


if makeVideo
    fprintf('Writing video...');
    switch votype
      case 'avifile'
        vo = close(vo);
      case 'VideoWriter'
        close(vo);
      otherwise
        error('unrecognized votype');
    end
    fprintf('done\n');
end