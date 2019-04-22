function crisp = crispness(p1, sigma)
    crisp = 0;
    m = length(p1);
%     Sig = 1/sigma * eye(3);

    
%     for i = 1:m
%         for j =1:m
%             delta = p1(i,:) - p1(j,:);
%             crisp = crisp + (2*pi*sigma)^(-3/2) * exp(-1/2*(delta * Sig * delta'));
%         
%         end
%     end
    
    for i = 1:m
        delta = p1(:,:) - p1(i,:);
        val = sum(delta,2);
        crisp = crisp + sum((2*pi*sigma)^(-3/2) * exp(-1/2 * val.^2 / sigma));
    end
    
    crisp = crisp / m^2;
end