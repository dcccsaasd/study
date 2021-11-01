package examples;/* --------------------------------------------------------------------------
 * File: MIPex2.java
 * Version 12.9.0  
 * --------------------------------------------------------------------------
 * Licensed Materials - Property of IBM
 * 5725-A06 5725-A29 5724-Y48 5724-Y49 5724-Y54 5724-Y55 5655-Y21
 * Copyright IBM Corporation 2001, 2019. All Rights Reserved.
 *
 * US Government Users Restricted Rights - Use, duplication or
 * disclosure restricted by GSA ADP Schedule Contract with
 * IBM Corp.
 * --------------------------------------------------------------------------
 * (1)ROWS: 每一行，包括目标函数与约束条件
 * (1 N 表示自由行， obj是对目标函数的命名，可以任意取
 * (2 L 表示该行小于等于， c1是对改行的命名，可以任意取名
 * (3 G 表示该行大于等于
 * (4 E 表示该行等于
 * (2)其中 MARK0000 ‘MARKER’ ‘INTORG’ MARK0001 ‘MARKER’ ‘INTEND’ 分别表示整数变量的起止
 * (3)Bounds: 表示各变量的上界或下界
 * (1 LO 表示下界
 * (2 UP 表示上界
 * (3 FX 表示该变量固定值
 * (4 FR 表示改变量的范围为(?∞,∞)
 * (5 MI 表示下界为负无穷
 * (6 PL 表示上界为正无穷
 * (7 MPS 变量默认的范围为[0,∞)
 * MIPex2.java - Reading in and optimizing a MIP problem.  In fact, this
 *               program also works for LP or QP problems, but is different
 *               from LPex2 in that no dual solution information is queried.
 *
 * To run this example, command line arguments are required.
 * i.e.,   java MIPex2  filename
 * where 
 *     filename is the name of the file, with .mps, .lp, or .sav extension
 * Example:
 *     java MIPex2  mexample.mps
 */

import ilog.concert.*;
import ilog.cplex.*;


public class MIPex2 {
   static void usage() {
      System.out.println("usage:  MIPex2 <filename>");
   }

   public static void main(String[] args) {
      if ( args.length != 1 ) {
         usage();
         return;
      }
      try {
         IloCplex cplex = new IloCplex();
       
         cplex.importModel(args[0]);
       
         if ( cplex.solve() ) {
            System.out.println("Solution status = " + cplex.getStatus());
            System.out.println("Solution value  = " + cplex.getObjValue());
          
            // Access the IloLPMatrix object that has been read from a file in
            // order to access variables which are the columns of the LP.  The
            // method importModel() guarantees that exactly one IloLPMatrix
            // object will exist, which is why no tests or iterators are
            // needed in the following line of code.

            IloLPMatrix lp = (IloLPMatrix)cplex.LPMatrixIterator().next();
          
            double[] x = cplex.getValues(lp);
            for (int j = 0; j < x.length; ++j) {
               System.out.println("Variable " + j + ": Value = " + x[j]);
            }

         }
         cplex.exportModel("mipex2.lp");
         cplex.end();
      }
      catch (IloException e) {
         System.err.println("Concert exception caught: " + e);
      }
   }
}
