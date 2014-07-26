function plotwedge(gca, Rbin_m, scan, detectList)
%PLOTWEDGE Summary of this function goes here
%   Not plotting detectList yet.
persistent h X Y cc

if isempty(h) % First time setup
  
  % Create the basis arcs
  R = 1:length(Rbin_m);

  A = [-pi*3/4:pi/16:-pi/4]';
  R_array = repmat(R,length(A),1);
  A_array = repmat(A,1,length(R));

  X = R_array.*cos(A_array);
  Y = R_array.*sin(A_array);

  scan_posint = round(abs(scan)*100)+1;
  cc = jet(max(scan_posint));
  h.wedge = plot(X,Y,'LineWidth',3);
  set(gca,'visible','off');
%  set(gca, 'ColorOrder', cc(scan,:), 'NextPlot', 'replacechildren','Color',[.5 .5 .5]);
  set(gca, 'ColorOrder', cc(scan_posint,:), 'NextPlot', 'replacechildren','Color',[.5 .5 .5]);

  axis equal; grid on
  cc = jet(101);

else

  scan_posint = round(abs(scan)*100+1);
  scan_posint(scan_posint > 101) = 101;
  fprintf('MaxWedge=%g\n',max(scan_posint));
  
  set(gca, 'ColorOrder', cc(scan_posint,:), 'NextPlot', 'replacechildren');
  delete(h.wedge);
  h.wedge = plot(X,Y,'LineWidth',3);
%  set(h.wedge,'Xdata',X,'Ydata',Y);
  %drawnow;
end
  
