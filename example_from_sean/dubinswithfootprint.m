%frs for dubins car dynamics. no uncertainty
%footprint: rectangle of length L and width W centered at the reference
%point. The rectangular footprint does not rotate with heading

clear
close

degree = 6;

Z0range = zeros(3,2);
Krange = [0,5;0,0.25];

T = 2;

[zscale,zoffset] = get_state_scaling_factors(@dubins,Z0range,'K_range',Krange,'T',2);

L = 5;
W = 2;

maxz = zscale-zoffset;
minz = -zscale-zoffset;

maxx = maxz(1:2)+[L/2;W/2]*sqrt(2);
minx = minz(1:2)-[L/2;W/2]*sqrt(2);

xscale = (maxx-minx)/2;
xoffset = -minx-(maxx-minx)/2;

kscale = (Krange(:,2)-Krange(:,1))/2;
koffset = -Krange(:,1)-(Krange(:,2)-Krange(:,1))/2;


%setup problem
t = msspoly('t');
z = msspoly('z',3);
x = msspoly('x',2);
k = msspoly('k',2);

tunscaled = T*t;
zunscaled = zscale.*z-zoffset;
xunscaled = xscale.*x-xoffset;
kunscaled = kscale.*k-koffset;

dubinsp = dubins(t,z,k);

fscaled = T./zscale.*subs(dubinsp,[t;z;k],[tunscaled;zunscaled;kunscaled]);

hZ0 = (zunscaled-Z0range(:,1)).*(Z0range(:,2)-zunscaled);

hZ = 1-z.^2;
hX = 1-x.^2;
hK = 1-k.^2;

intXK = boxMoments([x;k],-ones(4,1),ones(4,1));

hFtprint = (xunscaled-(zunscaled(1:2)-[L;W]/2)).*(zunscaled(1:2)+[L;W]/2-xunscaled);

FRSstates = [x;k];
hFRSstates = [hX;hFtprint];

%% computefrs

prob.t = t;
prob.z = z;
prob.x = x;
prob.k = k;
prob.hZ = hZ;
prob.hZ0 = hZ0;
prob.hK = hK;
prob.cost = intXK;
prob.f = fscaled;
prob.degree = degree;
prob.FRS_states = FRSstates;
prob.hFRS_states = hFRSstates;

out = compute_FRS(prob);

%% test
krand = randRange(Krange(:,1),Krange(:,2));
krandscaled = (krand+koffset)./kscale;
z0rand = randRange(Z0range(:,1),Z0range(:,2));

[~,ztmp] = ode45(@(t,z)dubins(t,z,krand),[0 T],z0rand);

plot(ztmp(:,1),ztmp(:,2),'k','LineWidth',1)
hold on
ftps = [];
for i = 1:length(ztmp)
    ftps = +[ftps,ztmp(i,1:2)'+[-L/2 L/2 L/2 -L/2 -L/2;-W/2 -W/2 W/2 W/2 -W/2],NaN(2,1)];
end

plot(ftps(1,:),ftps(2,:),'k')

%plot contour
wk = subs(out.indicator_function,k,krandscaled);
plot_2D_msspoly_contour(wk,x,1,'Offset',-xoffset,'Scale',xscale,'Color',[0 0.75 0.25],'LineWidth',1)




%%
function dzdt = dubins(t,z,k)
cos_psi = 1-z(3)^2/2;
sin_psi = z(3)-z(3)^3/6;

dzdt = [k(1)*cos_psi;k(1)*sin_psi;k(2)];
end