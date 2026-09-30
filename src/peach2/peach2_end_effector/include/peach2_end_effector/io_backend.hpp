#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>

namespace peach2_end_effector
{

/// Tool digital IO. `set_output` returning true only means the controller acknowledged the
/// write (aubo_msgs/SetIO.success), never that the blade moved.
class IoBackend
{
public:
  virtual ~IoBackend() = default;
  virtual bool set_output(int pin, bool level) = 0;
  /// Latest level of an input pin; nullopt when not (yet) observed or stale.
  virtual std::optional<bool> feedback(int pin) = 0;
  /// Actuator current [A]; nullopt when not measured.
  virtual std::optional<double> current() = 0;
};

/// Decorator that routes every write through a permission callback (the manipulation
/// command gate). A refused write returns false without touching the inner backend.
class GatedIoBackend : public IoBackend
{
public:
  using Permit = std::function<bool (int pin, bool level)>;

  GatedIoBackend(std::shared_ptr<IoBackend> inner, Permit permit)
  : inner_(std::move(inner)), permit_(std::move(permit)) {}

  bool set_output(int pin, bool level) override
  {
    if (!permit_ || !permit_(pin, level)) {
      return false;
    }
    return inner_->set_output(pin, level);
  }
  std::optional<bool> feedback(int pin) override {return inner_->feedback(pin);}
  std::optional<double> current() override {return inner_->current();}

private:
  std::shared_ptr<IoBackend> inner_;
  Permit permit_;
};

/// Simulated tool for tests and hardware_mode:=mock. Time comes from an injected clock so
/// tests are deterministic.
class MockIoBackend : public IoBackend
{
public:
  enum class Fault
  {
    NONE,
    WRITE_FAILS,       ///< set_output returns false
    STUCK_OPEN,        ///< feedback never reports closed (blade blocked / sensor dead low)
    STUCK_CLOSED,      ///< feedback always closed
    LOOPBACK,          ///< feedback mirrors the command output with zero delay (P0-3)
    NO_FEEDBACK,       ///< feedback() returns nullopt
    OPEN_STUCK,        ///< after closing, feedback never returns to open
  };

  struct Config
  {
    int cmd_pin{0};
    int feedback_pin{1};
    bool close_level{true};          ///< output level that closes the blade
    bool feedback_active_high{true}; ///< DI high = closed
    double close_delay_s{0.15};
    double open_delay_s{0.15};
    /// Simulated current: rises to peak while closing, drops after the cut.
    bool simulate_current{false};
    bool current_cut_signature{true};
    double peak_current_a{2.0};
    double idle_current_a{0.1};
    Fault fault{Fault::NONE};
  };

  MockIoBackend(Config config, std::function<double()> now_s);

  bool set_output(int pin, bool level) override;
  std::optional<bool> feedback(int pin) override;
  std::optional<double> current() override;

  void set_fault(Fault fault);
  void set_current_cut_signature(bool present);
  int write_count() const;
  std::optional<bool> last_output(int pin) const;

private:
  bool blade_closed_locked(double now) const;

  mutable std::mutex mutex_;
  Config config_;
  std::function<double()> now_s_;
  bool output_level_{false};
  bool output_written_{false};
  bool closed_at_change_{false};
  double output_change_s_{-1e9};
  int writes_{0};
};

}  // namespace peach2_end_effector
