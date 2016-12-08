% classdef PlanarBodyCLASS < TreeBodyCLASS
%
% Extends a body in a kinematic tree that is implemented as a set of linked
% objects to contain position and orientation data and compute this in a
% recursive fashion. 
% 
% Methods:
%  body = PlanarBodyCLASS(name)
%            Creates a planar body-object with the given name
%  recursiveForwardKinematics(B_r_IB, A_IB) 
%            Set's the position and orientation variables of this body to
%            the provided values and recursively calls all child joints to 
%            pass on this information.
%  recursiveGraphics() 
%            Draws the symbol of a COG at the position of the body.
%            Recursively calls all child joints to have them drawn, as
%            well. 
%
% Properties:
%
%   B_r_IB;  % Position of the body
%   A_IB;    % Orientation of the body
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef PlanarBodyCLASS < TreeBodyCLASS
     
    % Public properties
    properties (SetAccess = public, GetAccess = public)
        B_r_IB;  % Position of the body
        A_IB;    % Orientation of the body
    end
    
    % Public methods
    methods (Access = public)
        % Constructor
        function obj = PlanarBodyCLASS(name)
            % Call Constructor of the SuperCLASS
            obj = obj@TreeBodyCLASS(name);
        end
        
        function recursiveForwardKinematics(obj, B_r_IB, A_IB)
            % Position and orientation are given by the parent joint and
            % passed in its call of 'recursiveForwardKinematics'
            obj.A_IB   = A_IB; 
            obj.B_r_IB = B_r_IB;
            
            if obj.isLeaf== false
                % If this is no leaf, recursively call the child joints:
                for i = 1:obj.nChildren
                    obj.childJoints{i}.recursiveForwardKinematics(B_r_IB, A_IB);
                end
            end
        end
        
        function recursiveGraphics(obj)
            % Add a patch  to the current axis that is representing the COG
            % as a circle (with radius r) devided in four black and white
            % colored segments:
            r = 0.1;
            n_COG = linspace(0,2*pi,21);
            B_r_ICircle =[obj.B_r_IB(1,1) + cos(n_COG)*r;
                          obj.B_r_IB(2,1) + sin(n_COG)*r];
            % Transform this circle from B to I coordinates:
            I_r_ICircle = obj.A_IB * B_r_ICircle;          
            I_r_IB = obj.A_IB * obj.B_r_IB;
            % Extract geometric data:
            xPos = I_r_ICircle(1,:);
            yPos = I_r_ICircle(2,:);
            x = I_r_IB(1);
            y = I_r_IB(2);
            xData = [x,x,x,x;xPos(1:6)',xPos(6:11)',xPos(11:16)', xPos(16:21)'];
            yData = [y,y,y,y;yPos(1:6)',yPos(6:11)',yPos(11:16)', yPos(16:21)'];
            cData(1,:,:) = [0 0 0;1 1 1;0 0 0;1 1 1]; 
            patch(xData, yData, cData); 
            if obj.isLeaf == false
                % If this is no leaf, recursively call the child joints:
                for i = 1:obj.nChildren
                    obj.childJoints{i}.recursiveGraphics();
                end
            end
        end
    end
    
    % Protected methods
    methods (Access = protected)
        % none 
    end
    
    % Private methods
    methods (Access = private)
        % none 
    end
end

