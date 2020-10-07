function xnew=swrmctrlring(t,xx)
numofrobo=5;
x=[];
for ii=1:6:(numofrobo*6)
    x=[x [xx(ii) xx(ii+1) xx(ii+2) xx(ii+3) xx(ii+4) xx(ii+5)]'];
end

% constants initialized
M=1; beta=0.5;
Ks=diag([30 30]);
Kp=1*eye(2);
kij=1;
kl=1;
gamma=150;
alphai=70;
Li=diag([0.05 0.05]);
r=0.3;
r1=1.75;
r2=1.1;
cresnt=0.8;

% trajectory of the center of the circle
xo11=t;
xo12=2*sin(t);
xo21=xo11-cresnt;
xo22=xo12-cresnt;

xo11dot=1;
xo12dot=2*cos(t);
xodot=[xo11dot xo12dot]';

xo11dotdot=0;
xo12dotdot=-2*sin(t);
xodotdot=[xo11dotdot xo12dotdot]';


xnew=[];
zii=[0;0];
ziidot=[0;0];
gradxio1=[0;0];
gradxio1dot=[0;0];
gradxij=[0;0];
gradxijdot=[0;0];
g=0;
f=0;

% for each robot [for i=1:100]
for i=1:numofrobo
    xi1=x(1,i);
    xi2=x(2,i);
    xi1dot=x(3,i);
    xi2dot=x(4,i);
    
    x2=[xi1dot xi2dot]';
    xidot=x2;
    
    f1=(xi1-xo11)^2-(xi2-xo12)^2-r1^2;
    f2=r2^2-(xi1-xo21)^2-(xi2-xo22)^2;
    f=[f1 f2];
    for l=1:2
        gradxio1=[-2*(xi1-xo11);-2*(xi2-xo12)];
        gradxio1dot=[-2*(xi1dot-xo11dot);-2*(xi2dot-xo12dot)];
        
        zii=zii+kl*max(0,f(l))*gradxio1;
        ziidot=ziidot+kl*max(0,f(l))*gradxio1dot;
        
    end
    
    sigmaij=[0;0];
    sigmaijdot=[0;0];
   
    % for each neighbouring robot [for j=1:j~=i:100]
    for j=1:numofrobo
        if j~=i
            
            xj1=x(1,j);
            xj2=x(2,j);
            xj1dot=x(3,j);
            xj2dot=x(4,j);
            
            g=r^2-(xi1-xj1)^2-(xi2-xj2)^2;
            
            gradxij=[-2*(xi1-xj1);-2*(xi2-xj2)];
            gradxijdot=[-2*(xi1dot-xj1dot);-2*(xi2dot-xj2dot)];
            
            sigmaij=sigmaij+(kij*max(0,g)*gradxij);
            sigmaijdot=sigmaijdot+(kij*max(0,g)*gradxijdot);   
        end
        
    end
      
    epsiloni=alphai*zii+gamma*sigmaij;

    xridot=xodot-epsiloni;

    epsilonidot=alphai*ziidot+gamma*sigmaijdot;

    xridotdot=xodotdot-epsilonidot;

    Y=[xridotdot xridot];
    
    s=xidot-xridot;
    
    theta_hat=[x(5,i) x(6,i)]';

    u=-Ks*s-Kp*epsiloni+Y*theta_hat;
    
    x1dot=x2;
    x2dot=inv(M)*(u-beta*x2);

    theta_hat=-Li*Y'*s;
    
    xnew=[xnew;[x1dot' x2dot' theta_hat']'];
end
