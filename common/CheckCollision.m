function collision = CheckCollision(angle, mpData, is_close)
 
  if is_close
      linkLengthVec = mpData.conf.linkLengthVec(1:end-1);
      linkWidthVec = mpData.conf.linkWidthVec(1:end-1);
  else
      linkLengthVec = mpData.conf.linkLengthVec;
      linkWidthVec = mpData.conf.linkWidthVec;
  end
 % first: link-obstacle collision check 
  [fv, fvList] = triangleChain(mpData.base', linkLengthVec, linkWidthVec, angle);
  collision = CollisionCheck(fv, mpData.obstacle.fv);
  if collision
      return;
  end
  
  num_comb = size(mpData.test_list, 1);
  % second: link-link collision check (self collision)          
  for ind3 = 1:num_comb
       ind_links = mpData.test_list(ind3,:);
       collision = CollisionCheck(fvList{ind_links(1)}, fvList{ind_links(2)});
       if collision
          return;
       end
   end
end