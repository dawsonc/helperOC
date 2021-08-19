function dx = dynamics(obj, ~, x, u, d)
% Charles Dawson, 2021-08-18

if nargin < 5
  d = [0; 0];
end

if nargin < 6
  dims = obj.dims;
end

convert2num = false;
if ~iscell(x)
  x = num2cell(x);
  convert2num = true;
end

if ~iscell(u)
  u = num2cell(u);
end

if ~iscell(d)
  d = num2cell(d);
end

dx = cell(length(dims), 1);

for i = 1:length(dims)
  dx{i} = dynamics_cell_helper(obj, x, u, d, dims, dims(i));
end

if convert2num
  dx = cell2mat(dx);
end

end

function dx = dynamics_cell_helper(obj, x, u, d, dims, dim)

switch dim
  case 1
    dx = x{dims==3};
  case 2
    dx = x{dims==4};
  case 3
    theta = x{dims==2};
    theta_dot = x{dims==4};
    dx = (cos(theta) * (9.8 * sin(theta) + 11.5 * v) + 68.4 * v - 1.2 * (theta_dot ^ 2) * sin(theta)) / (cos(theta) - 24.7);
    dx = dx + u * (-1.8 * cos(theta) - 10.9) / (cos(theta) - 24.7) + d{1};
  case 4
    theta = x{dims==2};
    theta_dot = x{dims==4};
    dx = (-58.8 * v * cos(theta) - 243.5 * v - sin(theta) * (208.3 + (theta_dot ^ 2) * cos(theta))) / (torch.cos(theta) ^ 2 - 24.7);
    dx = dx + u * (9.3 * cos(theta) + 38.6) / (cos(theta) ^ 2 - 24.7) + d{2};
  otherwise
    error('Only dimension 1-4 are defined for dynamics of Segway!')
end

% disp("===========")
% disp(dim)
% disp("u")
% disp(u)
% disp("d")
% disp(d)
% disp("x")
% disp(x)
% disp("dx")
% disp(dx)
% disp("===========")
end