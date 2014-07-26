function plotpulseresponse(haxis, Rbin_m, rawDisplay, mofDisplay, ...
  mafDisplay, envDisplay, detDisplay, ylimVec)
%PLOTPULSERESPONSE Summary of this function goes here
%   Detailed explanation goes here
persistent h

if isempty(h)  % On first call set up the plot structure
  h.raw = plot(Rbin_m,rawDisplay,'-','Color',[.7 .7 .7]);
  hold on;  grid on
  h.motion = plot(Rbin_m,mofDisplay,'Color',rgb('DarkOrange'));
  h.matched = plot(Rbin_m,mafDisplay,'Color',rgb('DeepSkyBlue'));
  h.env = plot(Rbin_m,envDisplay,'-','Color',rgb('LightGreen'));
  h.det = plot(detDisplay(1,:),detDisplay(2,:),'*','LineWidth',3,'Color',rgb('Red'));

  set(haxis,'Color',[.4 .4 .4],'FontSize',17,'FontWeight','bold')
  xlabel('Range (m)','FontSize',18,'FontWeight','bold')
  ylabel('Normalized Amplitude','FontSize',18,'FontWeight','bold')
  h.legend = legend('raw','motion filter','matched filter','envelope','detections');
  set(h.legend,'TextColor','w','FontSize',16)
  xlim([0 Rbin_m(end)]);
  ylim(ylimVec);
%  set(gca,'XColor',[0.5 0.5 0.5]);

else
  % I assume this is the fastest way to do this (?)
  set(h.raw,'Ydata',rawDisplay);
  set(h.motion,'Ydata',mofDisplay);
  set(h.matched,'Ydata',mafDisplay);
  set(h.env,'Ydata',envDisplay);
  set(h.det,'Xdata',detDisplay(1,:),'Ydata',detDisplay(2,:));
end
  
