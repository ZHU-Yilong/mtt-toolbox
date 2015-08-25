function y = n3blkdiag(varargin)
%BLKDIAG  Block diagonal concatenation of multidimensional matrix input arguments.
%
%                                    |A 0 .. 0|
%   Y = NBLKDIAG(A,B,...)  produces  |0 B .. 0|
%                                    |0 0 ..  |
%
%   Class support for inputs: 
%      float: double, single
%
%   See also BLKDIAG, DIAG, HORZCAT, VERTCAT


y = [];

% if any(~cellfun('isclass',varargin,'double'))
    Dim = ndims(varargin{1});
%     if any(~cellfun('isequal',cellfun('ndims',varargin),Dim))        
        y = zeros(size(varargin{1}));
        for k=1:nargin
            x = varargin{k};
            p1 = size(y); p2 = size(x);
            p = p1 + p2;
            y = zeros(p);
            y(p1) = y; 
            y(p-p2+ones(Dim,1)) = x;
%             for jj = 1:Dim
%                 
%                 [p1,m1] = size(y); [p2,m2] = size(x);
%                 y = [y zeros(p1,m2); zeros(p2,m1) x]; %#ok
%             end
        end
%         return
%     end
% end


