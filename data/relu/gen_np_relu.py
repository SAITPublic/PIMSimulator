import numpy as np

dim_test = 4
dim_1ch = 16 * 8 * 8
dim_4ch = 16 * 16 * 8 * 4
dim_16ch = 16 * 8 * 8 * 16 * 48

DIM_IN = 64 * 1024 * 2
DIM_IN = 1024 * 1024

np.set_printoptions(precision=20)
np.random.seed(1113)
data_in = np.random.randn(DIM_IN).astype('float16')
data_out = np.zeros(DIM_IN).astype('float16')


def ReLU(x):
    return abs(x) * (x > 0)


data_out = ReLU(data_in)

np.save("relu_input_" + str(DIM_IN), data_in)
np.save("relu_output_" + str(DIM_IN), data_out)
# np.save("bn_params_resnet50", params)

# np.set_printoptions(threshold=np.nan)
print("data in : ", data_in)
print("data out : ", data_out)
