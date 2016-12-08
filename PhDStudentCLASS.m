% classdef PhDStudentCLASS < GradStudentCLASS
%
% Defines a subclass that changes the way, the name is displayed
% 
% Overwritten Methods:
%   PrintName()     % Prints out the name and the previous degree, adds
%   'PhD Student'
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef PhDStudentCLASS < GradStudentCLASS
    % Public methods
    methods (Access = public)
        % Constructor
        function obj = PhDStudentCLASS(name, ID, degree)
            % The constructor must ACTIVELY call the constructor of the
            % superclass
            obj = obj@GradStudentCLASS(name, ID, degree);
        end
    end
    % Protected methods
    methods (Access = protected)
        % Print out the name of the student adding 'PhD Student' (this
        % overrides the same function in the superclass)
        function PrintName(obj)
            disp(['Name: ',obj.name,', ',obj.degree,', PhD Student']);
        end
    end
end