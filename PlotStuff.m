function PlotStuff(Rbin_m,line1,line2,line3,line4,line5,line6,ylimVec)
persistent h

if isempty(h)  % On first call set up the plot structure
  h.subplot1 = subplot(2,1,1);
  h.line1 = plot(Rbin_m,line1,'-','Color',rgb('Gray'));
  hold on;  grid on
  h.line2 = plot(Rbin_m,line2,'Color',rgb('DeepSkyBlue'));
  h.line3 = plot(Rbin_m,line3,':','LineWidth',2,'Color',rgb('DeepSkyBlue'));

  set(gca,'Color',[.4 .4 .4],'FontSize',17,'FontWeight','bold')
  xlabel('Range (m)','FontSize',18,'FontWeight','bold')
  ylabel('Normalized Amplitude','FontSize',18,'FontWeight','bold')
%  h.legend = legend('raw','motion filter','matched filter','envelope','detections','threshold');
%  set(h.legend,'TextColor','w','FontSize',16)
  xlim([0 Rbin_m(end)]);
  ylim(ylimVec);
  
  h.subplot2 = subplot(2,1,2);
  h.line4 = plot(Rbin_m,line4,'-','LineWidth',1,'Color',rgb('Lime'));
  hold on;  grid on
  h.line5 = plot(Rbin_m,line5,':','LineWidth',2,'Color',rgb('Lime'));
  h.line6 = plot(Rbin_m,line6,'-','LineWidth',1,'Color',rgb('Red'));

  set(gca,'Color',[.4 .4 .4],'FontSize',17,'FontWeight','bold')
  xlabel('Range (m)','FontSize',18,'FontWeight','bold')
  ylabel('Normalized Amplitude','FontSize',18,'FontWeight','bold')
%  h.legend = legend('raw','motion filter','matched filter','envelope','detections','threshold');
%  set(h.legend,'TextColor','w','FontSize',16)
  xlim([0 Rbin_m(end)]);
  ylim(ylimVec);

%  set(gca,'XColor',[0.5 0.5 0.5]);

else
  
  axes(h.subplot1);
  set(h.line1,'Ydata',line1);
  set(h.line2,'Ydata',line2);
  set(h.line3,'Ydata',line3);

  axes(h.subplot2);
  set(h.line4,'Ydata',line4);
  set(h.line5,'Ydata',line5);
  set(h.line6,'Ydata',line6);

end
  
