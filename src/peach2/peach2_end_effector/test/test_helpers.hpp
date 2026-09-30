#pragma once

#include <memory>
#include <string>

#include "peach2_end_effector/end_effector.hpp"
#include "peach2_end_effector/io_backend.hpp"
#include "peach2_end_effector/plugins.hpp"
#include "peach2_end_effector/tool_profile.hpp"

namespace peach2_test
{

struct FakeClock
{
  double t{100.0};
  double now() const {return t;}
  void sleep(double dt) {t += dt;}
};

inline std::unique_ptr<peach2_end_effector::EndEffector> make_plugin(const std::string & id)
{
  if (id == "shear_v1") {
    return std::make_unique<peach2_end_effector::ShearV1>();
  }
  if (id == "bite_shear_v1") {
    return std::make_unique<peach2_end_effector::BiteShearV1>();
  }
  return std::make_unique<peach2_end_effector::AdaptiveShearV1>();
}

struct Rig
{
  std::shared_ptr<FakeClock> clock{std::make_shared<FakeClock>()};
  std::shared_ptr<peach2_end_effector::MockIoBackend> io;
  std::unique_ptr<peach2_end_effector::EndEffector> ee;

  explicit Rig(
    const std::string & id = "adaptive_shear_v1",
    peach2_end_effector::MockIoBackend::Config mock = {},
    peach2_end_effector::CurrentSignatureConfig current = {})
  {
    auto c = clock;
    io = std::make_shared<peach2_end_effector::MockIoBackend>(mock, [c]() {return c->now();});
    peach2_end_effector::EndEffectorContext ctx;
    ctx.profile = peach2_end_effector::load_tool_profile(id, PEACH2_TOOL_CONFIG_DIR);
    ctx.io = io;
    ctx.pins.cmd_pin = mock.cmd_pin;
    ctx.pins.feedback_pin = mock.feedback_pin;
    ctx.pins.feedback_active_high = mock.feedback_active_high;
    ctx.current = current;
    ctx.now_s = [c]() {return c->now();};
    ctx.sleep_s = [c](double dt) {c->sleep(dt);};
    ee = make_plugin(id);
    ee->initialize(ctx);
  }
};

}  // namespace peach2_test
