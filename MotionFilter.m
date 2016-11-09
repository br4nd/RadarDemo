function Y = MotionFilter(X,memDepth)
% Implement the Binomial MTI (Moving Target Indicator) filter
persistent M HH

if isempty(M)
  xN = length(X);
  M = min(abs(X))*randn(memDepth,xN);
  Y = zeros(1,xN);
  %% Experimental way of producing binomial coefficients (under development)
%   pt = pascal_triangle(memDepth);
%   HH = pt(end,:);
%   HH(2:2:end) = -HH(2:2:end)
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
  case 5
    H = [1,-5,10,-10,5,-1];
  case 6
    H = [1,-6,15,-20,15,-6,1];
  case 7
    H = [1,-7,21,-35,35,-21,7,-1];
%   case 8
%     H = [1,-8,28,-56,70,-56,28,-8,-1];    
  otherwise
    error('Max motion memory depth is 7');
end
Xp = [X;M];
H = H/max(abs(H));
Y = H*Xp;

% Update memory
M(2:memDepth,:) = M(1:memDepth-1,:);
M(1,:) = X;

