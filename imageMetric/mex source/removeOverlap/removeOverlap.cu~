/* function for projecting lidar points
 *
 */

#include "../common.h"

__global__ void buildIndexKernel(const size_t imWidth,
									  const size_t imHeight,
									  const float* const xIn,
									  const float* const yIn,
									  const float* const d,
									  const size_t numPoints,
									  int* const idx){

	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;

	if(i >= numPoints){
		return;
	}

	int x = round(xIn[i]);
	int y = round(yIn[i]);

	if((x < 0) || (y < 0) || (x >= imWidth) || (y >= imHeight)){
		return;
	}
	else{
		int cI, check = 0;
		do {
			cI = idx[y + x*imHeight];
			if((cI < 0) || (d[cI] > d[i])){
				int check = atomicCAS(&idx[y + x*imHeight],cI,i);
			}
			else
			{
				break;
			}
		} while(cI != check);	
	}
}

__global__ void setValidKernel(bool* const valid,
									  const size_t idxSize,
									  const int* const idx){

	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;

	if(i >= idxSize){
		return;
	}
	if(idx[i] < 0){
		return;
	}
	valid[idx[i]] = true;
}

__global__ void setupIdxKernel(const size_t idxSize,
									  int* const idx){

	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;

	if(i >= idxSize){
		return;
	}
	idx[i] = -1;
}

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, mxArray const *prhs[])
{
    //initialize the MathWorks GPU API.
    mxInitGPU();

    //read data
    mxGPUArray const * pointsMat = mxGPUCreateFromMxArray(prhs[0]);
    mxGPUArray const * distMat = mxGPUCreateFromMxArray(prhs[1]);
    size_t imWidth = ((uint32_T *) mxGetData(prhs[2]))[1];
    size_t imHeight = ((uint32_T *) mxGetData(prhs[2]))[0];
    size_t numPoints = mxGPUGetDimensions(pointsMat)[0];

	
    //get input pointers
    float* distPtr = (float*)(mxGPUGetDataReadOnly(distMat));

    float* xInPtr = (float*)(mxGPUGetDataReadOnly(pointsMat));
	float* yInPtr = &(xInPtr[numPoints]);
	
    //create output
	mwSize outSize[] = {numPoints,1};
    mxGPUArray* validMat = mxGPUCreateGPUArray(2, outSize, mxLOGICAL_CLASS, mxREAL, MX_GPU_INITIALIZE_VALUES);
	plhs[0] = mxGPUCreateMxArrayOnGPU(validMat);

	bool* validPtr = (bool*)(mxGPUGetData(validMat));

	//create idx matrix
	int *idxStore;
	CudaSafeCall(cudaMalloc((void **)&idxStore, imWidth*imHeight*sizeof(int)));
	setupIdxKernel<<<gridSize(imWidth*imHeight), BLOCK_SIZE>>>(imWidth*imHeight, idxStore);
	CudaCheckError();
	
    //run and get ouputs
	buildIndexKernel<<<gridSize(numPoints), BLOCK_SIZE>>>(imWidth, imHeight, xInPtr, yInPtr, distPtr, numPoints, idxStore);
	CudaCheckError();
	setValidKernel<<<gridSize(imWidth*imHeight), BLOCK_SIZE>>>(validPtr, imWidth*imHeight, idxStore);
	CudaCheckError();
	
	//free idx matrix
	CudaSafeCall(cudaFree(idxStore));
	
    //destroy reference structures
    mxGPUDestroyGPUArray(pointsMat);
    mxGPUDestroyGPUArray(distMat);
	mxGPUDestroyGPUArray(validMat);
}
