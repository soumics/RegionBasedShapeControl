r=1.7;
f=r^2-(xi1-xo11)^2-(xi2-xo12)^2;

gradxio1=[-2*(xi1-xo11);-2*(xi2-xo12)]
zii=kl*max(0,f)*gradxio1';

% for each neighbouring robot
g=r^2-(xi1-xj1)^2-(xi2-xj2)^2;
gradxij=[-2*(xi1-xj1);-2*(xi2-xj2)]
sigmaij=kij*max(0,g)*gradxij';

epsiloni=alphai*zii+gamma*sigmaij;
xodot=[1;2];
xridot=xodot-epsiloni;

si=xidot-xridot;

