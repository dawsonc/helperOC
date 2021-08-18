classdef Satellite2D < DynSys
  properties
    % Input bounds
    uMin = [-20, -20]
    uMax = [20, 20]
    
    n = 1.25 % Mean-motion
    mass = 1.25 % mass
    
    % Disturbance
    dRange
    
    % Dimensions that are active
    dims
  end
  
  methods
    function obj = Satellite2D(x, dRange, dims)
      % obj = Satellite2D(x, dMax, dims)
      %     2D CHW Satellite class
      %
      % Dynamics:
      %    \dot{x}_1 = x_3
      %    \dot{x}_2 = x_4
      %    \dot{x}_3 = 2*n*x_4 + 3*n*x_1 + u_1 / mass
      %    \dot{x}_4 = -2*n*x_3 + u_2 / mass
      %         u \in [-uMax, uMax]^2
      %         d \in [-dMax, dMax]
      %
      % Inputs:
      %   x      - state: [xpos; ypos]
      %   dMax   - disturbance bounds
      %
      % Output:
      %   obj       - a Satellite2D object
      
      if numel(x) ~= obj.nx
        error('Initial state does not have right dimension!');
      end
      
      if ~iscolumn(x)
        x = x';
      end
      
      if nargin < 2
        dRange = {[0; 0];[0; 0]};
      end
      
      if nargin < 3
        dims = 1:4;
      end
      
      if ~iscell(dRange)
          dRange = {-dRange,dRange};
      end
      
      % Basic vehicle properties
      obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
      %obj.hdim = find(dims == 3);   % Heading dimensions
      obj.nx = length(dims);
      obj.nu = 2;
      obj.nd = 2;
      
      obj.x = x;
      obj.xhist = obj.x;

      obj.dRange = dRange;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef
