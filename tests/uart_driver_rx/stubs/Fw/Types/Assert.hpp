// Test stub: FW_ASSERT as a ztest assertion.
#ifndef TEST_STUB_FW_ASSERT_HPP
#define TEST_STUB_FW_ASSERT_HPP
#include <zephyr/ztest.h>
#define FW_ASSERT(cond, ...) zassert_true((cond), "FW_ASSERT(" #cond ")")
#endif
