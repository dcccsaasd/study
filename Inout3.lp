\ENCODING=ISO-8859-1
\Problem name: ilog.cplex

Minimize
 obj: x1
Subject To
 c1: x1 - 0.6 x2 - 0.8 x3 - 0.3 x4 - 0.8 x5 - 0.9 x6 - 0.4 x7  = 0
 c2: x2 + x5  = 100
 c3: x3 + x6  = 200
 c4: x4 + x7  = 300
 c5: 0.5 x2 + 0.4 x3 + 0.3 x4 <= 20
 c6: 0.2 x2 + 0.4 x3 + 0.6 x4 <= 40
Bounds
      x2 >= 10
      x3 >= 10
      x4 >= 10
End
