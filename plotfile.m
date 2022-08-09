function plotfile(t,avp,att_mark)
dpos = deltapos(avp(1:end,7:9));
phi = rad2deg(avp(:,2)); theta = -rad2deg(avp(:,1)); psai = rad2deg(avp(:,3));
vx = avp(:,5); vy = -avp(:,4); vz = avp(:,6);
dx = dpos(:,2); dy = -dpos(:,1); dz = dpos(:,3);
msplot(221, t, [phi , theta], 'Att / ( \circ )');
legend('\it\phi','\it\theta')
msplot(222, t, psai, '\psi / ( \circ )');
msplot(223, t, [vx,vy,vz], 'Vel / ( m.s^{-1} )');
legend('\itv\rm_X', '\itv\rm_Y', '\itv\rm_Z')
msplot(224, t, [dx,dy,dz], '\DeltaPos / m');
legend('\Delta\itX', '\Delta\itY', '\Delta\itZ')
msplot(111, dx,dy, '\Delta\itX / m','\Delta\itY / m');


figure
subplot(311)
plot(rad2deg(att_mark(1,:)),'b','LineWidth',1.5)
ylabel('\it\theta /°')
grid on
set(gca,'FontSize',25)
set(gca,'gridlinestyle','--','GridColor','k','GridAlpha',1)
subplot(312)
plot(rad2deg(att_mark(2,:)),'b','LineWidth',1.5)
ylabel('\it\phi /°')
grid on
set(gca,'FontSize',25)
set(gca,'gridlinestyle','--','GridColor','k','GridAlpha',1)
subplot(313)
plot(rad2deg(att_mark(3,:)),'b','LineWidth',1.5)
ylabel('\it\psi /°')
xlabel('t/s')
grid on
set(gca,'FontSize',25)
set(gca,'gridlinestyle','--','GridColor','k','GridAlpha',1)
set(gcf,'unit','centimeters','position',[10,5,20,10])