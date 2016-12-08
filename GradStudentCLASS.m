% classdef GradStudentCLASS < StudentCLASS
%
% Defines a subclass that additionally contains the previous degree
% 
% Additional Methods:
%   PrintDegree()   % Prints out the name of the previous degree
% Overwritten Methods:
%   PrintName()     % Prints out the name and the previous degree
% Additonal Properties:
%   degree          % The previous degree of the grad student
%
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef GradStudentCLASS < StudentCLASS
    % Public properties
    properties (SetAccess = public, GetAccess = public)
         degree; 
    end
    % Public methods
    methods (Access = public)
        % Constructor
        function obj = GradStudentCLASS(name, ID, degree)
            % The constructor must ACTIVELY call the constructor of the
            % superclass
            obj = obj@StudentCLASS(name, ID);
            % Then we can do the initializeation for the subclass
            obj.degree = degree;
        end
        % Print out the name of the degree.
        function PrintDegree(obj)
            disp([obj.name,' holds a ', obj.degree]);
        end
    end
    % Protected methods
    methods (Access = protected)
        % Print out the name of the student with the previous degree (this
        % overrides the same function in the superclass)
        function PrintName(obj)
            disp(['Name: ',obj.name,', ',obj.degree]);
        end
    end
end