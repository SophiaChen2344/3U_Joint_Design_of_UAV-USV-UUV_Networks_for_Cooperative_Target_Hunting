# Experiment Log

## Status

- [ ] Stage 1: MATLAB fixed topology + USDE + fixed-time controller
- [x] Stage 2: digitize exact fixed topology from Fig. 10
- [x] Stage 3: align initial states from Table II
- [ ] Stage 4: comparison against ASCFC and FICFC

## Notes

- Fill Table II initial states from the paper.
- DDPG is intentionally deferred for now.
- Reconstruct nominal fixed topology from Fig. 10.
- Keep all current implementation work in MATLAB.
- Fig. 10 adjacency matrix entered as:
  [0 1 0 1 0;
   1 0 1 0 0;
   0 1 0 0 0;
   1 0 0 0 1;
   0 0 0 1 0]
- Table II initial states entered as:
  UAV1 [0,10]^T, pi/16
  UAV2 [5,6]^T, pi/10
  UAV3 [-5,6]^T, pi/6
  UAV4 [-2,3]^T, pi/4
  UAV5 [2,3]^T, pi/15
  UAV0 [0,0]^T, pi/16
- Added MATLAB alignment metrics and a sweep script over `vmax`, `wmax`, `theta_id_dot_limit`, and global heading offset.
- Current heuristic defaults in config:
  vmax = 6
  wmax = 8
  theta_id_dot_limit = 5
- Default heading offset is now set to `+0.05 rad` as the current best-guess from coarse alignment.
- Alignment metrics now emphasize the first `5s` and `10s` curve shape, plus zero-crossing penalty, to better match Fig. 15-16.
