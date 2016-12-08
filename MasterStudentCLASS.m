% classdef MasterStudentCLASS < GradStudentCLASS
%
% Defines a subclass that changes the way, the name is displayed
% 
% Overwritten Methods:
%   PrintName()     % Prints out the name and the previous degree, adds
%   'Master Student'
% Additonal Properties:
%   -
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef MasterStudentCLASS < GradStudentCLASS
    % Public properties
    properties (SetAccess = public, GetAccess = public)
         % none
    end
    % Public methods
    methods (Access = public)
        % Constructor
        function obj = MasterStudentCLASS(name, ID, degree)
            % The constructor must ACTIVELY call the constructor of the
            % superclass
            obj = obj@GradStudentCLASS(name, ID, degree);
        end
    end
    % Protected methods
    methods (Access = protected)
        % Print out the name of the student adding 'Master Student' (this
        % overrides the same function in the superclass)
        function PrintName(obj)
            disp(['Name: ',obj.name,', ',obj.degree,', Master Student']);
        end
    end
end