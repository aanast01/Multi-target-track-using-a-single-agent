function [F,Sigma,x_coor,y_coor] = caclucatePd(n, agentX, FoVh, FoVv, s1, s2, mu, Pd_min, Sigma)

X=([-agentX(3)*tand(FoVh/2), agentX(3)*tand(FoVh/2), agentX(3)*tand(FoVh/2), -agentX(3)*tand(FoVh/2),-agentX(3)*tand(FoVh/2)]);
Y=([-agentX(3)*tand(FoVv/2), -agentX(3)*tand(FoVv/2), agentX(3)*tand(FoVv/2), agentX(3)*tand(FoVv/2),-agentX(3)*tand(FoVv/2)]);
x_coor= agentX(1) + X;
y_coor= agentX(2) + Y;

Sigma(n,:,:) = [agentX(3)*s1/50 0; 0 agentX(3)*s2/50];
x1 = -ceil(agentX(3)*tand(FoVh/2)):.5:ceil(agentX(3)*tand(FoVh/2)); x2 = -ceil(agentX(3)*tand(FoVv/2)):.5:ceil(agentX(3)*tand(FoVv/2));
[X1, X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)], mu, squeeze(Sigma(n,:,:)));
F = reshape(F, length(x2), length(x1));
Pd = min(1,(1-Pd_min)/(-85)*agentX(3)+1.3);
F = F./max(F(:));
Pd_adjustment = max(F(:)) - Pd;
F = F - Pd_adjustment;
F(F<Pd_min)=Pd_min;

end
