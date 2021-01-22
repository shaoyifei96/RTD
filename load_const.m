m = 1558;
lf = 1.462884;
lr = 1.405516;
%             l = lf + lr;     
Cf = 1.432394487827058e+05;
Cr = Cf;
%             Cr = 2.214094963126969e+05;
%             g = 9.80655;
Izz = 3000; % from ram
Ca = Cf; % don't know if this makes sense

% feedback linearization parameters
Kv = 100;
Kr = 100;
Ku = 1;
amax = 3;
tpk = 3.25;
tbrk = 4;

Kvy = m*lf/(Cr*(lf+lr));
