#include "unity.h"
#include <Arduino.h>
#include "Drivers/Imu.h"

void setUp(void)
{
    Serial.begin(115200);
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_imus_init(void)
{
    // test stuff
    Serial.println("INIT");
    TEST_ASSERT_TRUE(true);
}

void test_imu_communication(void)
{
    // more test stuff
    Serial.println("COMM");
    TEST_ASSERT_TRUE(true);
}

int runUnityTests(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_imus_init);
    RUN_TEST(test_imu_communication);
    return UNITY_END();
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    runUnityTests();
}

void loop() {}
