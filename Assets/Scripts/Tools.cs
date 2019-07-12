using UnityEngine;

public static class Tools {

    public static float[,] M_Transpose(float[,] matrix)
    {
        int w = matrix.GetLength(0);
        int h = matrix.GetLength(1);
        float[,] result = new float[h, w];
        for (int i = 0; i < w; i++)
            for (int j = 0; j < h; j++)
                result[j, i] = matrix[i, j];

        return result;

    }

    public static float[,] M_Populate(float[,] matrix, Vector3[] pos) {

        for (int i = 0; i < pos.Length; i++)
        {
            matrix[0, i] = pos[i].x;
            matrix[1, i] = pos[i].y;
            matrix[2, i] = pos[i].z;
        }

        return matrix;
    }

    public static float[,] M_Multiply(float[,] m_A, float[,] m_B)
    {
        int rA = m_A.GetLength(0);
        int cA = m_A.GetLength(1);
        int rB = m_B.GetLength(0);
        int cB = m_B.GetLength(1);
        float[,] m_C = new float[rA, cB];

        if (cA == rB)
        {
            for (int i = 0; i < rA; i++)
            {
                for (int j = 0; j < cB; j++)
                {
                    m_C[i, j] = 0;
                    for (int k = 0; k < cA; k++) // OR k<b.GetLength(0)
                        m_C[i, j] += m_A[i, k] * m_B[k, j];
                }
            }
        }
        else
            Tools.displayDebug("Can't multiply!!");

        return m_C;
    }

    

    public static void displayDebug(string txt)
    {
        //Debug.Log(txt);
    }
}
