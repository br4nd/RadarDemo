function Y = motionfilter(X,memDepth)
% Implement the Binomial MTI (Moving Target Indicator) filter
persistent M

if isempty(M)
  xN = length(X);
  M = min(abs(X))*randn(memDepth,xN);
  Y = zeros(1,xN);
  return
end

switch memDepth
  case 1
    H = [1 -1];  % Subtract previous
  case 2
    H = [1 -2 1];
  case 3
    H = [1,-3,3,-1];
  case 4
    H = [1,-4,6,-4,1];
  otherwise
    error('Max motion memory depth is 4');
end
Xp = [X;M];
H = H/max(abs(H));
Y = H*Xp;

% Update memory
M(2:memDepth,:) = M(1:memDepth-1,:);
M(1,:) = X;

