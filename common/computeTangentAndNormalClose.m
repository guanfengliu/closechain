function [tl, tr, nl, nr] = computeTangentAndNormalClose(node_l, node_c, node_r, obst_id, mpData)
  pl = mpData.obstacle.coord(:, node_l);
  pc = mpData.obstacle.coord(:, node_c);
  pr = mpData.obstacle.coord(:, node_r);
  tl = (pc - pl) / norm(pc - pl);
  tr = (pr - pc) / norm(pr - pc);
  dd = pc - mpData.obstacle.center{obst_id};
  nl = dd - (dd' * tl) * tl;
  nl = nl / norm(nl);
               
  nr = dd - (dd' * tr) * tr;
  nr = nr / norm(nr);
end
