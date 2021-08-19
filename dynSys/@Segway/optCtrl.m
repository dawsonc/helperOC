function uOpt = optCtrl(obj, ~, ~, deriv, uMode)
% uOpt = optCtrl(obj, t, y, deriv, uMode)

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

%% Optimal control
theta = x{dims==2};
G = deriv{dims==3} * (-1.8 * cos(theta) - 10.9) / (cos(theta) - 24.7);
G = G + deriv{dims==4} * (9.3 * cos(theta) + 38.6) / (cos(theta) ^ 2 - 24.7);
if strcmp(uMode, 'max')
  uOpt = (G>=0)*obj.wRange(2) + (G<0)*(obj.wRange(1));
elseif strcmp(uMode, 'min')
  uOpt = (G>=0)*(obj.wRange(1)) + (G<0)*obj.wRange(2);
else
  error('Unknown uMode!')
end

end