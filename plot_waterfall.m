function plot_waterfall(range_vec_m, scan, wf_depth, caxis_lim)
%PLOTWATERFALL Summary of this function goes here
%   Not plotting detectList yet.
persistent imageH SA

if isempty(imageH) || ~isvalid(imageH) % First time setup
%  axes(h_axis)
  cla reset;
  hold off;
  SA = nan(wf_depth,length(scan));
  SA(1,:) = scan;
  imageH = imagesc(SA);
  xticklabels = [0:range_vec_m(end)];
  xticks = linspace(1,length(scan),numel(xticklabels));
  set(gca, 'XTick', xticks, 'XTickLabel', xticklabels,'FontSize',16,'FontWeight','bold')
  ylabel('Scan number (slow time)','FontSize',20,'FontWeight','bold')
  xlabel('Distance (m)','FontSize',20,'FontWeight','bold')
%  colormap hot
  colormap jet
  
  if ~isempty(caxis_lim)
    caxis(caxis_lim)
  end
  h_cb = colorbar;
  set(h_cb,'FontSize',16,'FontWeight','bold')
	
else
%  axes(h_axis)
  SA(2:end,:) = SA(1:end-1,:);
%  scanArray(1,:) = 20*log10(scan);
  SA(1,:) = scan;
%  xlim([0 480]);
  set(imageH,'CData',SA);
end
  
