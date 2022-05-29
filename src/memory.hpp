#pragma once

#include "EmbeddedMPC.h"

template<class T>
T* AllocateBuffer(char** ptr, int len) {
  T* array = (T*)(*ptr);
  *ptr += len * sizeof(T);
  return array;
}