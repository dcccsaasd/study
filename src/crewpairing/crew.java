package crewpairing;

import java.nio.file.Path;
import java.sql.Time;

public class crew {
    //�����ڵ���
     class AP_Node{
        private String City_Name;
        private boolean Is_AP_Base;
        private int Children_Num;
        private Flight_Node Children;

        public AP_Node(){//���в�����String,bollean,int,Flight_Node

        }
    }
    //����ڵ���
    class Flight_Node{
        private int Flight_No;
        private boolean Is_Leaf;
        //private int Leg_Num;
        //private Leg Leg_Node;
        private Flight_Node Children;
        private Flight_Node Father;
        private int Children_Num;
        private int Current_Children_Num;
        private Flight_Node Left_Flight_Node,Right_Flight_Node;
        private AP_Node Dep_Ctiy,Arr_City;
        //private time Dep_Time,Arr_Time;
        //private time Flight_Time,Turnover_Time;
        private Flight_Node Dep_Time,Arr_Time;
        private Flight_Node Flight_Time,Turnover_Time;

        public Flight_Node() { //���в���int,int,int
            Current_Children_Num=Children_Num;
        }

        public void Cal_Flight_Time(){

        }

        public void Cal_Turnover_Time(){

        }

        public Flight_Node Copy(){
            return null;
        }

    }
    //�����࣬�����ǵ����ڣ����Բ�����
    class Leg{

    }
    //����������������
    public class AP_Flight_Tree extends AP_Node{

         public AP_Flight_Tree(){

         }

         public void Children_Connect(){

         }

    }
    //��������
    class Flight_Tree{
         private int AP_Num;
         private int Flight_Node_Num;
         private int Level_Num;
         private int Leaf_Node_Num;
         private AP_Node Root_AP;
         private Flight_Node Current_Flight;
         private Flight_Node Leaf_Flight_Node;
         private AP_Flight_Tree AP_Flight_Tree_Node;

         //Flight_Tree(){}//;ȱʡ���캯��
         public Flight_Tree(){//���в���int,int,int,int,AP_Node

         }

         public void Connect_AP_Tree(){

         }

         public void Cut_Branch(){

         }

         public void Construct(){

         }

         public void Show_Tree(){

         }

         public void Save_Tree(){

         }
    }
    //��������������
    class Flight_Network extends Flight_Tree{
         private Path Flight_Path;

         public Flight_Network(){

         }

         public void Construct_Flight_Network(){

         }

         public void Save_Flight_Network(){

         }

         public Path Search_Path(){//���в���int
             return null;
         }

         public void Save_Path(){

         }
    }
    //���໷·(·����)
    class Path{
         private int min_Flight_Num,max_Flight_Num;
         //private time max_FLight_Time,max_Elapse_Time;
         private Flight_Network thisNetwork;
         //private Path Flight_String;
         //private Stack Current_path;
         private Flight_Node Current_Flight;
         //private time Flight_Time,Elapse_Time;

         /*
         public Path(){//���в���Flight_Network

         }
         */
         public void Deep_Search(){

         }

         public boolean Judge_Regulation(){
             return true;
         }

         public void Cal_Flight_Time(){

         }

         public void Cal_Elapse_Time(){

         }

         public void Output_path(){

         }

         public void Save_Path(){

         }

    }
}