\ENCODING=ISO-8859-1
\Problem name: ilog.cplex

Minimize
 obj: 0.6 x1 + 0.8 x2 + 0.8 x3 + 0.9 x4 + 0.3 x5 + 0.4 x6
Subject To
 c1: x1 + x2  = 100
 c2: x3 + x4  = 200
 c3: x5 + x6  = 300
 c4: 0.5 x1 + 0.4 x3 + 0.3 x5 <= 20
 c5: 0.2 x1 + 0.4 x3 + 0.6 x5 <= 40
End
