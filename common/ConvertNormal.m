function out = ConvertNormal(X)
    X = mod(X,2*pi);
    t = X > pi;
    X(t) = X(t)-2*pi;
    t = X < -pi;
    X(t) = X(t) + 2*pi;
    out = X;
   return
   