function Xau = rk4_m(fun, t0, tf, dt, X00)
global h


%N5 = ceil((tf - t0)/dt);
Xau(:,1) = X00;
t(:,1) = t0;
% dt=0.02;
h;
for k = 1:1:1
    
    K1 = feval(fun, t(:,k), Xau(:,k));
    K2 = feval(fun, t(:,k) + (h/2), Xau(:,k) + (h/2)*K1);
    K3 = feval(fun, t(:,k) + (h/2), Xau(:,k) + (h/2)*K2);
    K4 = feval(fun, t(:,k) + h, Xau(:,k) + h*K3);
   
    Xau(:,k) = Xau(:,k) + dt*(K1/6 + K2/3 + K3/3 + K4/6);
end            