% classdef StudentCLASS < handle
%
% Defines a superclass that contains a name and a UMich-ID and has
% functionality to print these values out 
% 
% Methods:
%  PrintRecord()    % Prints out the full record of the student
%  PrintName()      % Prints out the name of the student
% Properties:
%   name            % The name of the student.
%   ID (private)    % The umich ID of the student
%
%
%   C. David Remy cdremy@umich.edu
%   Matlab R2012b
%   9/12/2013
%   v11
%
classdef StudentCLASS < handle
    % Public properties
    properties (SetAccess = public, GetAccess = public)
         name; 
    end
    % Private properties
    properties (SetAccess = private, GetAccess = private)
         ID; 
    end
    
    % Public methods
    methods (Access = public)
        % Constructor
        function obj = StudentCLASS(name, ID)
            obj.name = name;
            obj.ID   = ID;
        end
        % Print out the ID and call the function that prints the name
        function PrintRecord(obj)
            disp(['UMich-ID: ',num2str(obj.ID)]);
            obj.PrintName();
            disp('');
        end
    end
    % Protected methods
    methods (Access = protected)
        % Print out the name
        function PrintName(obj)
            disp(['Name:     ',obj.name]);
        end
    end
end