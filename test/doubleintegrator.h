constexpr int kDimension = 2;
constexpr int kNumStates = kDimension * 2; 
constexpr int kNumErrStates = kNumStates;
constexpr int kNumInputs = kDimension;

const float h = 0.01;
const float b = h * h / 2.0;

const float Qfdata[4] = {
  10.0, 10.0, 10.0, 10.0
};
const float qfdata[4] = {
  0,0,0,0
};

const float Qdata[4] = {
  1.0, 1.0, 1.0, 1.0
};
const float qdata[4] = {
  0,0,0,0
};

const float Rdata[2] = {
  1e-2, 1e-2
};
const float rdata[2] = {
  0,0
};

const float Adata[16] = {  // NOLINT
    1, 0, 0, 0, 
    0, 1, 0, 0,
    h, 0, 1, 0,
    0, h, 0, 1
};

const float Bdata[8] = {  // NOLINT
    b, 0, h, 0,
    0, b, 0, h
};

const float fdata[4] = {
  0, 0, 0, 0
};