package UWR;

public class Location {

    //空间已知4点坐标
    float p[][] = new float[4][3];
    //空间已知4点距离
    float d[] = new float[4] ;

    //初始化空间4点坐标
    //p:坐标,数组
    //num:1-4
    void set_point(float point[],int num)  throws Exception
    {
        int j = 0;

        for (j = 0;j < 3;j++)
        {
            p[num - 1][j] = point[j];
        }
    }

    //初始化空间4点距离
    //distance:距离
    //num:1-4
    void set_distance(float distance,int num)  throws Exception
    {
        d[num - 1] = distance;
    }

    //计算未知点坐标
    //p:计算后的返回值
    //fail:back -1
    float[] calc()  throws Exception
    {
        float point[]=new float[3];
        //矩阵A
        float A[][] = new float[3][3];
        //矩阵B
        float B[]= new float[3];
        int i = 0;
        int j = 0;

        //初始化B矩阵
        for (i = 0;i < 3;i++)
        {
            B[i] = (LocationMath.d_p_square(p[i + 1]) - LocationMath.d_p_square(p[i]) - (d[i + 1] * d[i + 1] - d[i] * d[i])) / 2;
        }

        //初始化A矩阵
        for (i = 0;i < 3;i++)
        {
            for (j = 0;j < 3;j++)
            {
                A[i][j] = p[i + 1][j] - p[i][j];
            }
        }

        //计算未知点坐标
        point = LocationMath.solve(A,B);

        return point;
    }

}
