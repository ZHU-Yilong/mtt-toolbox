function y = n3blkdiag(varargin)
%N3BLKDIAG  Block diagonal concatenation of three-dimensional matrix input arguments.
%
%                                    |A 0 .. 0|
%   Y = N3BLKDIAG(A,B,...)  produces  |0 B .. 0|
%                                    |0 0 ..  |
%
%   Class support for inputs: 
%      float: double, single
%
%   See also BLKDIAG, DIAG, HORZCAT, VERTCAT

if nargin==0
    y = zeros(0,0,0);
    return;
elseif all(cellfun('isclass',varargin,'double')) && all(cellfun('ndims',varargin)==3)
    y = zeros(0,0,0);
    for k=1:nargin
        x = varargin{k};
        y1 = y;
        [p1,m1,n1] = size(y1); [p2,m2,n2] = size(x);
        p = p1 + p2; m = m1+m2; n = n1+n2;
        y = zeros(p,m,n);
        y(1:p1,1:m1,1:n1) = y1;
        y(p1+1:p,m1+1:m,n1+1:n) = x;
    end
else
    y = zeros(0,0,0);
end


