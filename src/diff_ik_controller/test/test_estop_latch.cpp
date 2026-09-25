#include <gtest/gtest.h>

#include "diff_ik_controller/estop_latch.hpp"

using diff_ik::EstopLatch;

TEST(EstopLatch, LatchesAndClears) {
  EstopLatch latch;
  EXPECT_FALSE(latch.stopped());
  EXPECT_FALSE(latch.clear());  // nothing to clear
  EXPECT_TRUE(latch.trigger());
  EXPECT_TRUE(latch.stopped());
  EXPECT_FALSE(latch.trigger());  // stays latched, no second edge
  EXPECT_TRUE(latch.clear());     // edge -> caller resyncs
  EXPECT_FALSE(latch.stopped());
}
