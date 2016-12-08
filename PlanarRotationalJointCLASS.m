% classdef PlanarRotationalJointCLASS < PlanarJointCLASS
%
% Defines a rotational joint in a kinematic tree that is implemented as a
% set of linked objects.  
% 
% Methods:
%  joint = PlanarRotationalJointCLASS(name, predBody, sucBody) 
%            Creates a rotational joint object with name 'name' that links
%            the body specified in 'predBody' with the body specified in
%            'sucBody'. Both are objects of the type 'PlanarBodyCLASS',
%            and function as predecessor and successor in a kinematic tree.
%
% Properties:
%     - none -
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef PlanarRotationalJointCLASS < PlanarJointCLASS
    
    % Public methods
    methods (Access = public)
        % The constructor only calls the constructor of the superclass.  We
        % have to do this by hand, since otherwise an empty constructor is
        % invoked.
        function obj = PlanarRotationalJointCLASS(name, predBody, sucBody)
            obj = obj@PlanarJointCLASS(name, predBody, sucBody);
        end
    end
    
    % Protected methods
    methods (Access = protected)
        % This method overwrites the 'displayType' method of the class
        % JointCLASS.  By doing so, we can change the behavior of this
        % specific joint 
        function s = getType(obj)
            % Use the code below to invoke displayType of the superclass
            % "JointCLASS"
            % s = displayType@JointCLASS(obj);
            s = 'Planar Rotational Joint';
        end
        
        function [Dp_r_DpDs, A_DpDs] = JointFunction(obj)
            alpha = obj.q;
            Dp_r_DpDs = [0;0];
            A_DpDs    = [+cos(alpha), -sin(alpha);
                         +sin(alpha), +cos(alpha)];
        end
    end
end

