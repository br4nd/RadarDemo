function plot_pulse(rangebin_m,line1,line2,line3,line4,line5)
%plot_pulse(gca,rangebin_m,raw_plot,bpf_plot,motion_plot,persist_plot,env_plot)

%PLOTPULSERESPONSE Summary of this function goes here
%   Detailed explanation goes here
persistent HH

ylimVec = [-1.1 1.1];

if isempty(HH) || ~isvalid(HH.line1) % First time setup
%  axes(h_axis);
  cla reset;
  hold off;
  HH.line1 = plot(rangebin_m,line1,'-','Color',rgb('Silver')); %raw_plot
  hold on;  grid on
  HH.line2 = plot(rangebin_m,line2,'Color',rgb('LightBlue'));%bpf_plot
  HH.line3 = plot(rangebin_m,line3,'Color',rgb('Lime'));%motion_plot
  HH.line4 = plot(rangebin_m,line4,'Color',rgb('Gold'));%persist_plot
  HH.line5 = plot(rangebin_m,line5,'Color',rgb('OrangeRed'),'linewidth',3);%env_plot

  set(gca,'Color',[.4 .4 .4],'FontSize',17,'FontWeight','bold')
%  xlabel('Range (m)','FontSize',18,'FontWeight','bold')
%  ylabel('Normalized Amplitude','FontSize',18,'FontWeight','bold')
%  HH.legend = legend('raw','motion filter','envelope');
%  set(HH.legend,'TextColor','w','FontSize',16)
  xlim([0 rangebin_m(end)]);
  ylim(ylimVec);
%  set(gca,'XColor',[0.5 0.5 0.5]);

else
%  axes(h_axis)
  set(HH.line1,'Ydata',line1);
  set(HH.line2,'Ydata',line2);
  set(HH.line3,'Ydata',line3);
  set(HH.line4,'Ydata',line4);
  set(HH.line5,'Ydata',line5);
end
  
