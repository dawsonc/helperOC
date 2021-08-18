function dx = dynamics(obj, ~, x, u, d)
% Dynamics:
%    \dot{x}_1 = x_3
%    \dot{x}_2 = x_4
%    \dot{x}_3 = 2*n*x_4 + 3*n*x_1 + u_1 / mass + d1
%    \dot{x}_4 = -2*n*x_3 + u_2 / mass + d2
%         u \in [-uMax, uMax]^2
%         d \in [-dMax, dMax]
%
% Charles Dawson, 2021-08-17

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
    dx = 2 * obj.n * x{dims==4} + 3 * obj.n ^ 2 * x{dims==1} + u{1} / obj.mass + d{1};
  case 4
    dx = -2 * obj.n * x{dims==3} + u{2} / obj.mass + d{2};
  otherwise
    error('Only dimension 1-4 are defined for dynamics of Satellite2D!')
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