%%SimulatorPlots
MarkerV=[];

Y_offset=0.5;

hold on;
CLR1 = [ 0 0.494 0.769; 1 0.5 0; 0 0.5 0.5; 1 0.5 0.25; 0.8 0.8 0; 0.3 0.6 0.9];
if(length(plt)>2)
    delete(plt(2:end));
end

Xmxp=MPCrng; Xminp=00;
Garea1=[0,2000];
Garea2=[5.5,5.5]+1;

y1=Y_offset+[L1(1:end).Y]';
x1=[L1(1:end).X]';
Clr1=[L1(1:end).OgLnID]';
Clr2=CLR1(Clr1(1:end),:);

ClrLR = [LR(1:end).OgLnID]';  % Use OgLnID for LR cars
assert(all(ClrLR <= size(CLR1, 1)), 'OgLnID exceeds CLR1 size'); 
ClrLRMap = CLR1(ClrLR, :);   % Map OgLnID to RGB colors

%% Sketch Road Networks.
xr=[LR(2:end).X]';
yr=xr*0+5; 
yr=Y_offset+0.2+(5.4-3./(1.+exp(-0.02*(xr-800.0))));
xr1=2.:50:1305; 
yr1=Y_offset+0.1+(5.4-3./(1.+exp(-0.02*(xr1-800.0))));
if(isempty(PltRoads))
    area(Garea1,Garea2, 'FaceColor',[0.65 0.85 0.25]);
    PltRoads=plot(xr1,yr1,'LineWidth',7,'Color',[0.7 0.7 0.7]);
    plot([5;3000], Y_offset+[2;2],'LineWidth',10,'Color',[0.7 0.7 0.7]);
    xline=[0:180:2000]'; yline=0*xline; %%GRIDs
    for I=1:length(xline)
        plot([xline(I);xline(I)], [yline(I);yline(I)+6],'k' );
    end
    plt=plot([5;3000], Y_offset+[1.5;1.5],'-','LineWidth',1.5,'Color','w'); %% Repeating
end
for J=1:length (xr)
    plt(end+1)= plot(xr(J), yr(J), 'sk','LineWidth',.151, 'MarkerSize', 4.5,'MarkerFaceColor', ClrLRMap(J,:)); %, 'MarkerFacColor',
    plt(end+1)=  plot(xr(J)-2.5, yr(J), 'sk','LineWidth',.151, 'MarkerSize', 5.5,'MarkerFaceColor', ClrLRMap(J,:)); %, 'MarkerFacColor',
end

for J=1:length (x1)
    plt(end+1)= plot(x1(J), y1(J), 'sk', 'LineWidth',0.01,  'MarkerSize', 4.5,'MarkerFaceColor', Clr2(J,:)); %, 'MarkerFacColor', Clr2(J,:)
    plt(end+1)=  plot(x1(J)-3, y1(J), 'sk', 'LineWidth',0.01, 'MarkerSize', 6,'MarkerFaceColor', Clr2(J,:)); %, 'MarkerFacColor', Clr2(J,:)
end


Msg=sprintf('Simulation time 00:%d:%02d',floor(dt*(i-1)/60),mod(floor(dt*(i-1)),60));
title(Msg);
axis([Xminp, Xmxp, 0.0, 6+Y_offset]);
grid on;
ylabel('Lane');
xlabel('X');
set(gca, 'YTick', 3);          % Set only tick at 3
set(gca, 'YTickLabel', {'1'}); % Display label '1' only

ax = gca;
Boundary=[0.025 0.25 0.96 0.6];
ax.Position = Boundary;

M(fm)=getframe(hFig);
fm=fm+1;
