classdef Segway < DynSys
  properties
    % Input bounds
    uRange = [-10, 10]
    
    % Disturbance
    dRange
    
    % Dimensions that are active
    dims
  end
  
  methods
    function obj = Segway(x, dRange, dims)
      % obj = Segway(x, dMax, dims)
      %     Segway class
      %
      % Inputs:
      %   x      - state: [p, theta, v, omega]
      %   dMax   - disturbance bounds
      %
      % Output:
      %   obj       - a Segway object
      
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
      obj.nu = 1;
      obj.nd = 2;
      
      obj.x = x;
      obj.xhist = obj.x;

      obj.dRange = dRange;
      obj.dims = dims;
    end
    
  end % end methods
end % end classdef
