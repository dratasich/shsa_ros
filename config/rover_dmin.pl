%
% SHSA Knowledge Base for Daisy's dmin Calculation
%
% Denise Ratasich
% 2018-11-08
%

:- use_module(library(shsa)).


% SHSA knowledge base from
% Ratasich et al.: Self-healing by property-guided structural adaptation. 2018.
%

% dmin
function(dmin, rdl, [laser]).
function(dmin, rds, [sonar]).
function(dmin, rda, [current, velocity]).
function(dmin, rdm, [map, position]).
function(dmin, rdb, [bumper]).
function(velocity, rpv, [position, position_last]).

% to create executable substitutions: define implementations of the relations
implementation(rdl, "dmin.v = min(laser.v.ranges)").
implementation(rds, "
dist = min(sonar.v)
e = 2  # max allowed error of dist
dmin.v = interval.interval([dist - e, dist + e])
").
implementation(rda, "").
implementation(rdm, "").
implementation(rdb, "
dmin.v = None
if bumper.v == 0:
   dmin.v = 0
").
implementation(rpv, "").

% hard-code provided itoms
% (though the itoms can be extracted by calling `rostopic list`,
% a mapping to the variables is needed, so we just statically define it here)
itomsOf(dmin, ["/emergency_stop/dmin/data"]).
itomsOf(sonar, ["/p2os/sonar/ranges"]).
