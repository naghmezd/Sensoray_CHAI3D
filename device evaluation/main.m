clc;clear all;
data = readmatrix('data.csv');
figure
pos1_modified = [-0.0014,0.00075,0.0001,0.0008,0.0002,-0.0002,-0.0004,-0.0005, -0.0006,-0.0005,-0.0002,-0.0003,-0.0003,-0.00016,-0.00026,-0.00021,-0.0001,-0.0001,-0.00017,-0.00016,0,-0.0001,-0.0001,-0.0001,-0.0001,-0.0001,-0.0001,-0.0001, -0.0001,-0.0001,-0.0001,0];
pos2_modified = [-0.017,-0.0075,-0.0066,-0.0054,-0.0038,-0.0033,-0.0031,-0.0029, -0.0026,-0.0024,-0.00195,-0.00187,-0.0018,-0.0016,-0.0017,-0.0015,-0.00136,-0.00131,-0.00129,-0.00125,-0.0011,-0.0011,-0.00107,-0.00107,-0.00104,-0.00104,-0.001,-0.00099,-0.00099,-0.00099,-0.00095,-0.00095];
position_modified = pos1_modified-pos2_modified;
force = ones(1,32)*1.96;%convert to Newton
stiff_modified(:)=(force(:)./position_modified(:));
for i=1:32
    k(1,i)=i*100;
end


pos1_unmodified=[0.0063, 0.0016 , 0.004 ,0.0010, 0.0009, 0.0006 ,0, 0.0001, -0.0001,0.0001, -0.0001,0.0001,0.0001, -0.0001, -0.0002, -0.0002, -0.0002, -0.0002, -0.0001, -0.0001, -0.0001, -0.0001, -0.0002, -0.0001, -0.0002, -0.0001,0,0, -0.0001, -0.0001, -0.0001,  -0.0001];
%p2 is the mean for lower values in position z data
pos2_unmodified=[-0.0323,-0.0145,-0.0092,-0.007,-0.0058,-0.0052,-0.0036,-0.0032,-0.0028,-0.0027,-0.0024,-0.0023,-0.0020,-0.0019,-0.0018,-0.0016,-0.0016,-0.0017,-0.0015, -0.0014, -0.0013, -0.0013, -0.0013, -0.0012, -0.0012, -0.0011, -0.0012, -0.0011, -0.001, -0.001, -0.0011, -0.001];
position_unmodified = pos1_unmodified-pos2_unmodified;
%force = ones(1,32)*1.96;%convert to Newton
stiff_unmodified(:)=(force(:)./position_unmodified(:));

hold on

plot(k(1:32),k(1:32),'r','LineWidth',2)
g_modified = fit(k(1:32)',stiff_modified(1:32)','poly3','Normalize','on','Robust','Bisquare');
a=plot(g_modified,k(1:32)',stiff_modified(1:32)','.')
set(a,'color','k','LineWidth',2)
g_unmodified = fit(k(1:32)',stiff_unmodified(1:32)','poly3','Normalize','on','Robust','Bisquare');
b=plot(g_unmodified,k(1:32)',stiff_unmodified(1:32)','.')
set(b,'color','b','LineWidth',2)
legend('Input','Modified Output','Modified Fit','Unmodified Output','Unmodified Fit')
xlabel('Stiffness')
ylabel('Stiffness')