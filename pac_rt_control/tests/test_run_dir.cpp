// 실행 기록 폴더(prepare_run_dir) 단위 테스트: 임시 폴더 아래에서 확인
#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include "rt_control/run_dir.hpp"

namespace {

namespace fs = std::filesystem;
using rt_control::prepare_run_dir;

class RunDirTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string tmpl = (fs::temp_directory_path() / "rt_run_dir_XXXXXX").string();
    ASSERT_NE(mkdtemp(tmpl.data()), nullptr);
    root_ = tmpl;
  }
  void TearDown() override { fs::remove_all(root_); }

  static void touch(const fs::path& p) { std::ofstream(p) << "x\n"; }

  fs::path root_;
};

TEST_F(RunDirTest, EmptyRunIdUsesTimeAndTag) {
  const auto d = prepare_run_dir(root_.string(), "", "stop", "20261007_120000");
  ASSERT_TRUE(d.ok) << d.error;
  EXPECT_EQ(d.run_id, "20261007_120000_stop");
  EXPECT_EQ(d.path, root_.string() + "/20261007_120000_stop");
  EXPECT_TRUE(fs::is_directory(d.path));
}

TEST_F(RunDirTest, EmptyRunIdNoTag) {
  const auto d = prepare_run_dir(root_.string(), "", "", "20261007_120000");
  ASSERT_TRUE(d.ok) << d.error;
  EXPECT_EQ(d.run_id, "20261007_120000");
}

TEST_F(RunDirTest, RunIdCreatesFolderAndIgnoresTag) {
  const auto d = prepare_run_dir(root_.string(), "exp01", "stop", "20261007_120000");
  ASSERT_TRUE(d.ok) << d.error;
  EXPECT_EQ(d.run_id, "exp01");
  EXPECT_EQ(d.path, root_.string() + "/exp01");
  EXPECT_TRUE(fs::is_directory(d.path));
}

TEST_F(RunDirTest, RunIdExistingFolderWithoutRtCsvIsOk) {
  // 다른 계층이 먼저 만든 폴더 (자기 파일만 있음)
  fs::create_directories(root_ / "exp02");
  touch(root_ / "exp02" / "a_infer.csv");
  const auto d = prepare_run_dir(root_.string(), "exp02", "", "20261007_120000");
  ASSERT_TRUE(d.ok) << d.error;
  EXPECT_TRUE(fs::exists(root_ / "exp02" / "a_infer.csv"));
}

TEST_F(RunDirTest, RunIdExistingRtCsvIsError) {
  fs::create_directories(root_ / "exp03");
  touch(root_ / "exp03" / "rt.csv");
  const auto d = prepare_run_dir(root_.string(), "exp03", "", "20261007_120000");
  EXPECT_FALSE(d.ok);
  EXPECT_NE(d.error.find("rt.csv"), std::string::npos);
  // 기존 파일은 그대로
  std::ifstream in(root_ / "exp03" / "rt.csv");
  std::string line;
  std::getline(in, line);
  EXPECT_EQ(line, "x");
}

TEST_F(RunDirTest, RunIdWithSlashOrDotsIsError) {
  EXPECT_FALSE(prepare_run_dir(root_.string(), "a/b", "", "t").ok);
  EXPECT_FALSE(prepare_run_dir(root_.string(), "..", "", "t").ok);
  EXPECT_FALSE(prepare_run_dir(root_.string(), ".", "", "t").ok);
  EXPECT_FALSE(fs::exists(root_ / "a"));
}

TEST_F(RunDirTest, RunIdPathIsFileIsError) {
  touch(root_ / "exp04");
  EXPECT_FALSE(prepare_run_dir(root_.string(), "exp04", "", "t").ok);
}

}  // namespace
