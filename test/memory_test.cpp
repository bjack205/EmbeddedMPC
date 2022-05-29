#include <fmt/core.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "EmbeddedMPC.h"
#include "doubleintegrator.h"
#include "memory.cpp"

TEST(MemoryTests, AllocateInt) {
  char buf[100];
  char* ptr = (char*)buf;
  c_int* buf_int = AllocateBuffer<c_int>(&ptr, 10);
  EXPECT_EQ(ptr, buf + 10 * sizeof(c_int));
  EXPECT_EQ((void*)buf_int, (void*)buf);

  c_int* buf_int2 = AllocateBuffer<c_int>(&ptr, 5);
  EXPECT_EQ(ptr, buf + 15 * sizeof(c_int));
  EXPECT_EQ((void*)(buf_int2), (void*)(buf_int + 10));

  EXPECT_EQ((void*)buf_int, (void*)buf);
}

TEST(MemoryTests, MixedAllocation) {
  char buf[100];
  char* ptr = (char*)buf;

  // Grab 10 ints
  int size_int = 10;
  c_int* buf_int = AllocateBuffer<c_int>(&ptr, size_int);
  EXPECT_EQ(ptr, buf + 10 * sizeof(c_int));
  EXPECT_EQ((void*)buf_int, (void*)buf);

  // Grab 5 floats 
  int size_float = 5;
  c_float* buf_float = AllocateBuffer<c_float>(&ptr, size_float);
  EXPECT_EQ(ptr, buf + 10 * sizeof(c_int) + 5 * sizeof(c_float));
  EXPECT_EQ((char*)buf_float, (char*)(buf_int + size_int));
}