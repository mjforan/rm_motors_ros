#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

constexpr static const uint8_t ID_MIN = 1;

constexpr static const uint8_t ID_MAX = 7;

constexpr static const double N_PER_A = 741.0;

constexpr static const double V_MAX = 24.0;

constexpr static const double I_MAX = 1.62;

enum class CmdMode {
  Voltage,
  Current,
  Torque,
  Velocity,
};

enum class FbField {
  Position,
  Velocity,
  Current,
  Temperature,
};

struct Gm6020Can;

extern "C" {

Gm6020Can *gm6020_can_init(const char *interface);

int8_t gm6020_can_run(Gm6020Can *gm6020_can, uint64_t period_ms);

int8_t gm6020_can_run_once(Gm6020Can *gm6020_can);

int8_t gm6020_can_cmd_single(Gm6020Can *gm6020_can, CmdMode mode, uint8_t id, double cmd);

int8_t gm6020_can_cmd_multiple(Gm6020Can *gm6020_can,
                               CmdMode mode,
                               const double *const *cmds,
                               uint8_t len);

double gm6020_can_get(Gm6020Can *gm6020_can, uint8_t id, FbField field);

} // extern "C"
