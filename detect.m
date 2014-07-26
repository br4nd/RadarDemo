function detectList = detect(Rbin_m,scan_env,detectMode,minThreshold)
% DETECT - dirt simple detection
% A total hack.  Needs improvement.

detectList = nan(2,length(Rbin_m));

switch detectMode
  case 0 % flat threshold
    overI = find(scan_env > minThreshold);
    if ~isempty(overI)
      detectList = [Rbin_m(overI); scan_env(overI)];
    end
  case 1 % 1/r threshold
    K = .5;
    threshVec = K./(Rbin_m.^2);
%    threshVec(threshVec<minThreshold) = minThreshold;
    overI = find(scan_env > threshVec);
    if ~isempty(overI)
      detectList = [Rbin_m(overI); scan_env(overI)];
    end
  case 2 % CFAR (work in progress - do not use ...)
    % Based on Cell Averaging Continuous False Alarm Rate (CFAR)
    % http://www.mathworks.com/help/phased/examples/constant-false-alarm-rate-cfar-detection.html
    trainingPreN = 1;
    guardPreN = 10;
    guardPostN = 1;
    trainingPostN = 10;

    N = sum([trainingPreN,guardPreN,trainingPostN,guardPostN]);
    alpha = N*(0.0001^(-1/N)-1);

    % 'cut': Cell Under Test
    cutStartI = trainingPreN + guardPreN + 1;
    cutEndI = length(scan_env) - trainingPostN - guardPostN;

    detectI = [];
    for cutI = cutStartI:cutEndI
      trainingBeforeV = scan_env(cutI-guardPreN-trainingPreN:cutI-guardPreN);
      trainingAfterV = scan_env(cutI+guardPostN:cutI+guardPostN+trainingPostN);
      trainingAvg = mean([trainingBeforeV,trainingAfterV]);
      minThreshold = alpha*trainingAvg + minThreshold;
      if scan_env(cutI) > minThreshold
        detectI = [detectI cutI];
      end
      fprintf('cutI:%d,scan_env(cutI)=%g,trainingAvg=%g,threshold=%g\n',...
        cutI,scan_env(cutI),trainingAvg,minThreshold);
    end

    if isempty(detectI)
      detectList = [nan;nan];
    else
      detectList = [Rbin_m(detectI);scan_env(detectI)];
    end
end


