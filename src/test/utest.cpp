#include <gtest/gtest.h>
#include <climits>
#include "../Foo.cpp"

TEST(Foo, testCase1) {
    Foo foo;
    int a = 4;
    int b = 5;
    int sum = foo.add(4, 5);
    EXPECT_EQ(sum, 9);

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
