% MTX matrix set for the linearized MPC problem for the WMR.
%   Inside the Kuhne's MPC Algorithms,
%   MTX returns a time-variant, discrete-time matrix, according
%   to the number of input arguments.
%   
%   MTX(theta(k)) returns the coordinate error transformation for the
%   angle theta at instant k.
%   MTX(theta(k),T) returns the input-state matrix - B(k) - with
%   sampling period of T seconds.
%   MTX(theta(k),v(k),T) returns the state transition matrix - A(k) - with
%   sampling period of T seconds.
%
%   © Felipe Kuhne - GCAR - DELET - UFRGS
%   kuhne@eletro.ufrgs.br
%

function M = mtx(varargin)

if ((nargin==0) | (nargin>3))
    error('MTX function requires at least one and no more than three input arguments')
end

if (nargin==1) % coordinate error transformation matrix
    M = [cos(varargin{1}) sin(varargin{1}) 0 ;
        -sin(varargin{1}) cos(varargin{1}) 0 ;
        0 0 1];
end

if (nargin==2) % input-state matrix B(k)
    M = [cos(varargin{1})*varargin{2} 0 ;
         sin(varargin{1})*varargin{2} 0 ;
         0 varargin{2}];
end

if (nargin==3) % state transition matrix A(k)
    M = [1 0 -varargin{2}*sin(varargin{1})*varargin{3} ;
         0 1 varargin{2}*cos(varargin{1})*varargin{3} ;
         0 0 1];
end
