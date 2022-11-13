

#include <ztest.h>


static void test_good(void)
{
	unsigned int key1, key2;

	key1 = arch_irq_lock();
	zassert_true(arch_irq_unlocked(key1),
		     "IRQs should have been unlocked, but key is 0x%x\n",
		     key1);
	key2 = arch_irq_lock();
	zassert_false(arch_irq_unlocked(key2),
		      "IRQs should have been locked, but key is 0x%x\n",
		      key2);
	arch_irq_unlock(key2);
	arch_irq_unlock(key1);
}


static void test_fail(void)
{
	zassert_equal(17, 17, "bad index");
}

void test_main(void)
{
	ztest_test_suite(test,
		ztest_unit_test(test_good),
		ztest_unit_test(test_fail)
	);

	ztest_run_test_suite(test);
}
