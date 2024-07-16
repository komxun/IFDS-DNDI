function xdot = Dyn_imple(t,xx)
        global U 
        Vv = xx(4:6);
U;
        x1dot = Vv;
        x2dot = U;        
        xdot=[x1dot' x2dot']';
end