__kernel void sgemm3(__global float *A, 
                     __global float *B, 
                     __global float *C, 
                     const unsigned int N, 
                     __local float *localA, 
                     __local float *localB)
{
    int globalRow = get_global_id(1);
    int globalCol = get_global_id(0);
    int localRow  = get_local_id(1);
    int localCol  = get_local_id(0);
    int localSize = get_local_size(0);  // assuming square local size

    float sum = 0.0f;

    // Loop over all blocks of both matrices
    for (int k = 0; k < N; k += localSize) {
        // Load block of matrix A to local memory
        localA[localRow * localSize + localCol] = A[globalRow * N + k + localCol];

        // Load block of matrix B to local memory, adjusting for column-major access
        localB[localRow * localSize + localCol] = B[(k + localRow) * N + globalCol];

        // Synchronize to make sure the tiles are loaded
        barrier(CLK_LOCAL_MEM_FENCE);

        // Multiply the two matrix blocks and accumulate result
        for (int j = 0; j < localSize; j++) {
            sum += localA[localRow * localSize + j] * localB[j * localSize + localCol];
        }

        // Ensure computation is done before loading next block
        barrier(CLK_LOCAL_MEM_FENCE);
    }

    C[globalRow * N + globalCol] = sum;
}