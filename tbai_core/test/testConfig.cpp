#include <fstream>
#include <string>

#include "tbai_core/config/YamlConfig.hpp"
#include <gtest/gtest.h>
#include <tbai_core/Types.hpp>

#define DUMMY_CONFIG_PATH "./test_config_abc123.yaml"

class YamlConfigTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() {
        const std::string configPath = DUMMY_CONFIG_PATH;
        std::ofstream file(configPath);
        std::string content = "a:\n  b: hello\n  c: 1\n  d: 3.14\n  e:\n    f: 28\nvec: [1,2,3] \nmat: [[1,2],[3,4]]\n";
        content = content + "joint_names: [joint1, joint2, joint3]\n";
        file << content;
        file.close();
    }

    static void TearDownTestSuite() {
        const std::string configPath = DUMMY_CONFIG_PATH;
        std::remove(configPath.c_str());
    }
};

TEST_F(YamlConfigTest, delimDot) {
    const std::string configPath = DUMMY_CONFIG_PATH;
    const char delim = '.';
    tbai::core::YamlConfig config(configPath, delim);

    ASSERT_EQ(config.get<std::string>("a.b"), "hello");
    ASSERT_EQ(config.get<int>("a.c"), 1);
    ASSERT_EQ(config.get<double>("a.d"), 3.14);
    ASSERT_EQ(config.get<int>("a.e.f"), 28);
}

TEST_F(YamlConfigTest, delimForwardslash) {
    const std::string configPath = DUMMY_CONFIG_PATH;
    const char delim = '/';
    tbai::core::YamlConfig config(configPath, delim);

    ASSERT_EQ(config.get<std::string>("a/b"), "hello");
    ASSERT_EQ(config.get<int>("a/c"), 1);
    ASSERT_EQ(config.get<double>("a/d"), 3.14);
    ASSERT_EQ(config.get<int>("a/e/f"), 28);
}

TEST_F(YamlConfigTest, listOfStrings) {
    const std::string configPath = DUMMY_CONFIG_PATH;
    const char delim = '/';
    tbai::core::YamlConfig config(configPath, delim);

    std::vector<std::string> jointNames = config.get<std::vector<std::string>>("joint_names");
    ASSERT_EQ(jointNames.size(), 3);
    ASSERT_EQ(jointNames[0], "joint1");
    ASSERT_EQ(jointNames[1], "joint2");
    ASSERT_EQ(jointNames[2], "joint3");
}

TEST_F(YamlConfigTest, vector) {
    const std::string configPath = DUMMY_CONFIG_PATH;
    const char delim = '/';
    tbai::core::YamlConfig config(configPath, delim);

    tbai::vector_t vec = config.get<tbai::vector_t>("vec");
    ASSERT_EQ(vec.size(), 3);
    ASSERT_EQ(vec(0), 1.0);
    ASSERT_EQ(vec(1), 2.0);
    ASSERT_EQ(vec(2), 3.0);
}

TEST_F(YamlConfigTest, matrix) {
    const std::string configPath = DUMMY_CONFIG_PATH;
    const char delim = '/';
    tbai::core::YamlConfig config(configPath, delim);

    tbai::matrix_t mat = config.get<tbai::matrix_t>("mat");
    ASSERT_EQ(mat.rows(), 2);
    ASSERT_EQ(mat.cols(), 2);
    ASSERT_EQ(mat(0, 0), 1.0);
    ASSERT_EQ(mat(0, 1), 2.0);
    ASSERT_EQ(mat(1, 0), 3.0);
    ASSERT_EQ(mat(1, 1), 4.0);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
