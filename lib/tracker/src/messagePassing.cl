__kernel void message_passing(
    __global const float* prev,
    __global const float* pairwise,
    __global float* curr,
    __global int* ids,
    int offset, int rows, int cols)
{
    int i = get_global_id(0);
    int j = get_global_id(1);

    // boundary check
    if (i >= rows || j >= cols)
        return;

    float curMax = -INFINITY;
    int curMaxI = i;
    int curMaxJ = j;

    for (int ii = -offset; ii <= offset; ii++)
    {
        int curI = i + ii;
        if ((curI < 0) || (curI >= rows))
            continue;
        for (int jj = -offset; jj <= offset; jj++)
        {
            int curJ = j + jj;
            if ((curJ < 0) || (curJ >= cols))
                continue;

            float curPrev = prev[curI * cols + curJ];
            float curPP = pairwise[(ii + offset) * cols + (jj + offset)];
            float curVal = curPrev + curPP;

            if (curVal > curMax)
            {
                curMax = curVal;
                curMaxI = curI;
                curMaxJ = curJ;
            }
        }
    }
    curr[i * cols + j] = curMax;
    vstore2((int2)(curMaxI, curMaxJ), i * cols + j, ids);
}
