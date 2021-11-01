package UWR;

public class Location {

    //�ռ���֪4������
    float p[][] = new float[4][3];
    //�ռ���֪4�����
    float d[] = new float[4] ;

    //��ʼ���ռ�4������
    //p:����,����
    //num:1-4
    void set_point(float point[],int num)  throws Exception
    {
        int j = 0;

        for (j = 0;j < 3;j++)
        {
            p[num - 1][j] = point[j];
        }
    }

    //��ʼ���ռ�4�����
    //distance:����
    //num:1-4
    void set_distance(float distance,int num)  throws Exception
    {
        d[num - 1] = distance;
    }

    //����δ֪������
    //p:�����ķ���ֵ
    //fail:back -1
    float[] calc()  throws Exception
    {
        float point[]=new float[3];
        //����A
        float A[][] = new float[3][3];
        //����B
        float B[]= new float[3];
        int i = 0;
        int j = 0;

        //��ʼ��B����
        for (i = 0;i < 3;i++)
        {
            B[i] = (LocationMath.d_p_square(p[i + 1]) - LocationMath.d_p_square(p[i]) - (d[i + 1] * d[i + 1] - d[i] * d[i])) / 2;
        }

        //��ʼ��A����
        for (i = 0;i < 3;i++)
        {
            for (j = 0;j < 3;j++)
            {
                A[i][j] = p[i + 1][j] - p[i][j];
            }
        }

        //����δ֪������
        point = LocationMath.solve(A,B);

        return point;
    }

}
