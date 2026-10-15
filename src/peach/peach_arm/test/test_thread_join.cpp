#include "peach_arm/thread_join.hpp"
#include <gtest/gtest.h>

TEST(ThreadJoin, CompletedTaskMakesStdThreadNonJoinable)
{
  std::packaged_task<void()> task([] {});
  auto done = task.get_future();
  std::thread worker(std::move(task));
  done.wait();
  const bool joined = peach_arm::joinFinishedThread(worker, done, std::chrono::milliseconds(0));
  EXPECT_TRUE(joined);
  EXPECT_FALSE(worker.joinable());
  if (worker.joinable()) {worker.join();}
}

TEST(ThreadJoin, TimeoutDoesNotJoinOrConsumeAnActiveTask)
{
  std::promise<void> release;
  auto ready = release.get_future();
  std::packaged_task<void()> task([&ready] {ready.wait();});
  auto done = task.get_future();
  std::thread worker(std::move(task));
  EXPECT_FALSE(peach_arm::joinFinishedThread(worker, done, std::chrono::milliseconds(0)));
  EXPECT_TRUE(worker.joinable());
  release.set_value();
  done.wait();
  worker.join();
}
