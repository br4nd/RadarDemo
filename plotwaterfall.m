function plotwaterfall(haxis, Rbin_m, scan, detectList, slowTimeDepth,caxisLim)
%PLOTWATERFALL Summary of this function goes here
%   Not plotting detectList yet.
persistent h scanArray

if isempty(h) % First time setup
  %slowTimeDepth = 50;
  scanArray = nan(slowTimeDepth,length(scan));
  scanArray(1,:) = scan;
  h.image = imagesc(scanArray);
  xticklabels = [0:1:Rbin_m(end)];
  xticks = linspace(1,length(scan),numel(xticklabels));
  set(gca, 'XTick', xticks, 'XTickLabel', xticklabels,'FontSize',16,'FontWeight','bold')
  ylabel('Scan number (slow time)','FontSize',20,'FontWeight','bold')
  xlabel('Distance (m)','FontSize',20,'FontWeight','bold')
  colormap jet
  h.cb = colorbar;
  set(h.cb,'FontSize',18,'FontWeight','bold')
  if ~isempty(caxisLim)
    caxis(caxisLim)
  end
else
  scanArray(2:end,:) = scanArray(1:end-1,:);
%  scanArray(1,:) = 20*log10(scan);
  scanArray(1,:) = scan;
%  xlim([0 480]);
  set(h.image,'CData',scanArray);
end
  
