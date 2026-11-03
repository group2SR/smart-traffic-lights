#ifndef CONFIGURATION_STORAGE_H
#define CONFIGURATION_STORAGE_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "TrafficConfig.h"
#include "TrafficPhaseManager.h"

#if defined(ARDUINO) && defined(ESP32)
#include <Preferences.h>
#endif

// ============================================================================
// CONFIGURATION STORAGE MODULE (ESP32 Preferences / NVS + Checksum Validation)
// ============================================================================
// Stores and validates:
// - Operating ControlMode (AUTO, FIXED, ACTUATED, ADAPTIVE, MANUAL)
// - Phase durations (min/max green, clear-gap, yellow, all-red, startup safe)
// - Pedestrian WALK and clearance durations
// - Sensor debounce, stability filtering, stuck timeout, and sensor-to-movement map
// - Demand thresholds, wait-time weights, and starvation limits
// - Right-Hand Traffic (Keep-Right) conflict matrix & protected-turn commissioning flag
// - Electrical output driver settings
//
// Corrupted or out-of-range settings are automatically rejected and fall back
// to documented safe defaults.

struct PersistentControllerConfig {
    uint32_t                            magic;
    uint16_t                            schemaVersion;
    uint8_t                             controlMode; // ControlMode enum value
    uint8_t                             vehicleDriverType;
    uint8_t                             pedDriverType;
    bool                                vehicleDriverVerified;
    bool                                pedDriverVerified;
    TrafficConfig::TimingConfig         timings;
    TrafficConfig::SensorConfig         sensors;
    TrafficConfig::DemandConfig         demand;
    TrafficConfig::ConflictMatrixConfig conflictMatrix;
    uint32_t                            checksum;
};

class ConfigurationStorage {
public:
    static constexpr uint32_t CONFIG_MAGIC          = 0x53544C32UL; // "STL2"
    static constexpr uint16_t CONFIG_SCHEMA_VERSION = 2;
    static constexpr const char* NVS_NAMESPACE      = "stl_cfg_v2";
    static constexpr const char* NVS_BLOB_KEY       = "ctrl_blob";

    ConfigurationStorage()
        : hasShadowBlob_(false),
          useHardwareNvs_(true) {
        memset(&shadowBlob_, 0, sizeof(shadowBlob_));
    }

    void setUseHardwareNvs(bool enable) { useHardwareNvs_ = enable; }
    bool isUsingHardwareNvs() const { return useHardwareNvs_; }

    static PersistentControllerConfig makeDefaultConfig() {
        PersistentControllerConfig cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.magic                 = CONFIG_MAGIC;
        cfg.schemaVersion         = CONFIG_SCHEMA_VERSION;
        cfg.controlMode           = static_cast<uint8_t>(ControlMode::AUTO);
        cfg.vehicleDriverType     = static_cast<uint8_t>(TrafficConfig::OutputDriverType::UNVERIFIED_SAFE_DISABLED);
        cfg.pedDriverType         = static_cast<uint8_t>(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH);
        cfg.vehicleDriverVerified = TrafficConfig::VEHICLE_HIGH_SIDE_DRIVER_VERIFIED;
        cfg.pedDriverVerified     = TrafficConfig::PED_ULN2003A_DRIVER_VERIFIED;
        cfg.timings               = TrafficConfig::DEFAULT_TIMING_CONFIG;
        cfg.sensors               = TrafficConfig::DEFAULT_SENSOR_CONFIG;
        cfg.demand                = TrafficConfig::DEFAULT_DEMAND_CONFIG;
        cfg.conflictMatrix        = TrafficConfig::DEFAULT_CONFLICT_MATRIX;
        cfg.checksum              = computeChecksum(cfg);
        return cfg;
    }

    static uint32_t computeChecksum(const PersistentControllerConfig& cfg) {
        // FNV-1a 32-bit hash over all fields prior to checksum
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&cfg);
        const size_t len     = offsetof(PersistentControllerConfig, checksum);
        uint32_t hash        = 2166136261UL;
        for (size_t i = 0; i < len; ++i) {
            hash ^= bytes[i];
            hash *= 16777619UL;
        }
        return hash;
    }

    static bool validateConfig(const PersistentControllerConfig& cfg) {
        if (cfg.magic != CONFIG_MAGIC || cfg.schemaVersion != CONFIG_SCHEMA_VERSION) {
            return false;
        }
        if (cfg.checksum != computeChecksum(cfg)) {
            return false;
        }
        if (cfg.controlMode > static_cast<uint8_t>(ControlMode::MANUAL)) {
            return false;
        }
        if (cfg.vehicleDriverType > static_cast<uint8_t>(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH) ||
            cfg.pedDriverType > static_cast<uint8_t>(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH)) {
            return false;
        }
        // Vehicle modules are 5V common-ground and MUST NOT use low-side ULN2003A
        if (cfg.vehicleDriverType == static_cast<uint8_t>(TrafficConfig::OutputDriverType::LOW_SIDE_ULN2003A_ACTIVE_HIGH)) {
            return false;
        }
        if (!isTimingRangeValid(cfg.timings)) {
            return false;
        }
        if (cfg.sensors.irStabilityMs < 10 || cfg.sensors.irStabilityMs > 5000 ||
            cfg.sensors.pedDebounceMs < 10 || cfg.sensors.pedDebounceMs > 2000 ||
            cfg.sensors.maxInputStuckMs < 1000 || cfg.sensors.maxInputStuckMs > 600000) {
            return false;
        }
        for (uint8_t i = 0; i < TrafficConfig::IR_SENSOR_COUNT; ++i) {
            if (cfg.sensors.sensorToMovement[i] >= TrafficConfig::VEHICLE_GROUP_COUNT) {
                return false;
            }
        }
        if (cfg.demand.waitTimeWeightMs < 100 || cfg.demand.waitTimeWeightMs > 60000 ||
            cfg.demand.activeSensorWeight == 0 || cfg.demand.activeSensorWeight > 50 ||
            cfg.demand.starvationSkipLimit == 0 || cfg.demand.starvationSkipLimit > 20) {
            return false;
        }
        if (!cfg.conflictMatrix.rightHandTrafficRule) {
            return false;
        }
        // Core conflicts must remain enabled
        if (!cfg.conflictMatrix.conflicts[0][1] || !cfg.conflictMatrix.conflicts[1][0] ||
            !cfg.conflictMatrix.conflicts[0][2] || !cfg.conflictMatrix.conflicts[1][2] ||
            !cfg.conflictMatrix.conflicts[0][3] || !cfg.conflictMatrix.conflicts[1][3] ||
            !cfg.conflictMatrix.conflicts[2][3]) {
            return false;
        }
        return true;
    }

    bool save(const PersistentControllerConfig& inCfg) {
        PersistentControllerConfig toSave = inCfg;
        toSave.magic         = CONFIG_MAGIC;
        toSave.schemaVersion = CONFIG_SCHEMA_VERSION;
        toSave.checksum      = computeChecksum(toSave);

        if (!validateConfig(toSave)) {
            return false;
        }

        shadowBlob_    = toSave;
        hasShadowBlob_ = true;

#if defined(ARDUINO) && defined(ESP32)
        if (useHardwareNvs_) {
            Preferences prefs;
            if (!prefs.begin(NVS_NAMESPACE, false)) {
                return false;
            }
            const size_t written = prefs.putBytes(NVS_BLOB_KEY, &toSave, sizeof(toSave));
            prefs.end();
            return (written == sizeof(toSave));
        }
#endif
        return true;
    }

    bool load(PersistentControllerConfig& outCfg) {
        PersistentControllerConfig candidate;
        memset(&candidate, 0, sizeof(candidate));
        bool readOk = false;

#if defined(ARDUINO) && defined(ESP32)
        if (useHardwareNvs_) {
            Preferences prefs;
            if (prefs.begin(NVS_NAMESPACE, true)) {
                const size_t len = prefs.getBytesLength(NVS_BLOB_KEY);
                if (len == sizeof(candidate)) {
                    const size_t got = prefs.getBytes(NVS_BLOB_KEY, &candidate, sizeof(candidate));
                    readOk = (got == sizeof(candidate));
                }
                prefs.end();
            }
        }
#endif
        if (!readOk && hasShadowBlob_) {
            candidate = shadowBlob_;
            readOk    = true;
        }

        if (!readOk || !validateConfig(candidate)) {
            outCfg = makeDefaultConfig();
            return false;
        }

        outCfg = candidate;
        return true;
    }

    bool resetToDefaults(PersistentControllerConfig& outCfg) {
        outCfg = makeDefaultConfig();
        return save(outCfg);
    }

    // Helper for unit testing corrupted NVS blobs
    void injectRawShadowBlobForTest(const PersistentControllerConfig& rawBlob) {
        shadowBlob_    = rawBlob;
        hasShadowBlob_ = true;
    }

private:
    static bool inBounds(uint32_t val) {
        return (val >= TrafficConfig::MIN_ALLOWED_INTERVAL_MS &&
                val <= TrafficConfig::MAX_ALLOWED_INTERVAL_MS);
    }

    static bool isTimingRangeValid(const TrafficConfig::TimingConfig& t) {
        if (!inBounds(t.startupSafeMs) || !inBounds(t.allRedMs) ||
            !inBounds(t.mainMinGreenMs) || !inBounds(t.mainYellowMs) ||
            !inBounds(t.sideMinGreenMs) || !inBounds(t.sideMaxGreenMs) ||
            !inBounds(t.sideClearGapMs) || !inBounds(t.sideYellowMs) ||
            !inBounds(t.pedWalkMs)      || !inBounds(t.mainMaxGreenMs) ||
            !inBounds(t.mainClearGapMs) || !inBounds(t.turnMinGreenMs) ||
            !inBounds(t.turnMaxGreenMs) || !inBounds(t.turnClearGapMs) ||
            !inBounds(t.turnYellowMs)   || !inBounds(t.pedClearanceMs)) {
            return false;
        }
        if (t.sideMaxGreenMs < t.sideMinGreenMs ||
            t.mainMaxGreenMs < t.mainMinGreenMs ||
            t.turnMaxGreenMs < t.turnMinGreenMs) {
            return false;
        }
        return true;
    }

    PersistentControllerConfig shadowBlob_;
    bool                       hasShadowBlob_;
    bool                       useHardwareNvs_;
};

#endif // CONFIGURATION_STORAGE_H

