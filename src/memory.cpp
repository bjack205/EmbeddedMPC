#include "memory.hpp"

c_int* AllocateInt(char** p_mem, int len) {
  c_int* int_array = (c_int*)(*p_mem);
  *p_mem += len * sizeof(c_int);
  return int_array;
}