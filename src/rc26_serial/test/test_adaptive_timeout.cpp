#include <gtest/gtest.h>

#include "rc26_serial/adaptive_timeout.hpp"

TEST(AdaptiveTimeout, InitializesFromFirstSample) {
    rc26_serial::AdaptiveTimeout timeout;
    timeout.update(80.0F);

    EXPECT_NEAR(timeout.ewmaMs(), 80.0F, 1e-3F);
    EXPECT_EQ(timeout.get().count(), 240);
}

TEST(AdaptiveTimeout, ConvergesOnStableSamples) {
    rc26_serial::AdaptiveTimeout timeout;
    timeout.update(200.0F);
    for (int i = 0; i < 32; ++i) {
        timeout.update(100.0F);
    }

    EXPECT_NEAR(timeout.ewmaMs(), 100.0F, 2.0F);
    EXPECT_LE(timeout.get().count(), 140);
    EXPECT_GE(timeout.get().count(), 50);
}

TEST(AdaptiveTimeout, TimeoutBackoffRespectsUpperBound) {
    rc26_serial::AdaptiveTimeout timeout;
    timeout.update(60.0F);

    int prev = timeout.get().count();
    EXPECT_GE(prev, 50);
    for (int i = 0; i < 8; ++i) {
        timeout.onTimeout();
        const int current = timeout.get().count();
        EXPECT_GE(current, prev);
        prev = current;
    }
    EXPECT_EQ(timeout.get().count(), 500);
}

TEST(AdaptiveTimeout, ClampRespectsLowerBound) {
    rc26_serial::AdaptiveTimeout timeout;
    timeout.update(1.0F);
    EXPECT_EQ(timeout.get().count(), 50);

    for (int i = 0; i < 10; ++i) {
        timeout.update(1.0F);
    }
    EXPECT_EQ(timeout.get().count(), 50);
}
