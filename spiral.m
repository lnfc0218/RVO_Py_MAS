%%
clear; close all
range = 2*pi*5;
norm_v = 1;
X=[];
Y=[];
for theta=0:0.01:range
    rad = 0.02 + theta / range * norm_v;
    x = rad * cos(theta);
    y = rad * sin(theta);
    X = [X,x];
    Y = [Y,y];
end
plot(X,Y)
%%
clear; close all
range = 2*pi*2;
norm_v = 1;
X=[];
Y=[];
Z=[];
for azim=0:0.1:range
    rad = 0.02 + azim / range * norm_v;
    for zen=0:.1:pi
        x = rad * cos(azim) * sin(zen);
        y = rad * sin(azim) * sin(zen);
        z = rad * cos(zen);
        X = [X,x];
        Y = [Y,y];
        Z = [Z,z];
    end
end
plot3(X,Y,Z)