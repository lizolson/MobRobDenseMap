close all;
ptcloud = pcread('meshptcloud.ply');
% pcshow(ptcloud)
pL = ptcloud.Location;
pC = single(ptcloud.Color)./255;
sigma = 0.01;

m = length(pL);
xlim = ptcloud.XLimits(2) - ptcloud.XLimits(1);
ylim = ptcloud.YLimits(2) - ptcloud.YLimits(1);
zlim = ptcloud.ZLimits(2) - ptcloud.ZLimits(1);

pL2 = ptcloud.Location(1:50000,:);


alpha = 0:0.01:0.1;
c = zeros(length(alpha),1);
c2 = zeros(length(alpha),1);
for i = 1:length(alpha)
    
    pL2(1:50000,1) = pL2(1:50000,1) + alpha(i)*randn(50000,1)/xlim;
    pL2(1:50000,2) = pL2(1:50000,2) + alpha(i)*randn(50000,1)/ylim;
    pL2(1:50000,3) = pL2(1:50000,3) + alpha(i)*randn(50000,1)/zlim;
    tic;
    c(i) = crispness(pL2(1:10000,:), sigma);
    toc;    

end
figure;
plot(alpha,c,'LineWidth',2);
xlabel('noise');
ylabel('crispness');
title('Crispness');
figure;
plot(alpha,-log(c),'LineWidth',2);
title('Renyi Quadratic Entropy');