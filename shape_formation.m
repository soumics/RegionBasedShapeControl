clc;
rad=1.7;
th=[0:360];
numofrobo=20;
plot(rad*cos((pi/360)*th),rad*sin((pi/360)*th),'-g')
%hold on
r=1.7*rand(numofrobo,1).^(0.1);
theta=360*rand(numofrobo,1);
x=cos((pi/360)*th)*r;
y=sin((pi/360)*th)*r;
plot(x,y,'o')

theta=[0.5 0]';
thetai=repmat(theta,1,numofrobo);
t0 = 0; tf = 15;
x0=[x';y';zeros(2,numofrobo);thetai];

[t,x] = ode23(@swrmctrl,[t0,tf],x0);
% [t,x] = ode23(@swrmctrlring,[t0,tf],x0);

xo11=t;
xo12=2*sin(t);

for cnt=1:length(t)
    p1=[];
    x1=x(cnt,:);
    for ii=1:6:(numofrobo*6)
        p1=[p1 [x1(ii) x1(ii+1)]'];
    end
    plot(p1(1,:),p1(2,:),'o')
    axis([-10 18 -10 15])
    pause(.25)
end