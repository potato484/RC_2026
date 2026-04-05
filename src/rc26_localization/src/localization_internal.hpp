#pragma once

// Internal TU-local constants shared by rc26_localization implementation files.
// Keep them in an unnamed namespace so existing callsites can use unqualified names
// (e.g. kNearZero) without adding a new namespace prefix.

namespace rc26_localization {
namespace {
constexpr double kNearZero = 1e-6;

// Global relocalization candidate cost weights
constexpr double kCostWf = 0.5;
constexpr double kCostWxy = 0.3;
constexpr double kCostWyaw = 0.2;

// T8: slope constraint
constexpr double kMaxSlopeRollPitchCorrectionDeg = 5.0;

// Observability/covariance diagnostics
constexpr double kDiagObsNoiseNominal = 1e-2;
constexpr double kDiagObsNoiseDegenerate = 1e6;
}  // namespace
}  // namespace rc26_localization

