-- Diagnostic only: compare local mapping without online pose-graph optimization.
-- Keep cartographer_2d.lua unchanged so the regular mapping launch still works.
include "cartographer_2d.lua"

POSE_GRAPH.optimize_every_n_nodes = 0

return options
