#pragma once

#include <array>
#include <cstdint>

#include "chad/util/geometry.hpp"
#include "chad/util/util.hpp"

namespace chad {

/// PCG-XSH-RR 32/64 (O'Neill 2014). 16 bytes of state against mt19937's 2.5 KB,
/// and no multi-kilobyte seeding pass, which is what makes it the usual choice
/// on a microcontroller.
class Pcg32 {
public:
    static constexpr std::uint64_t kDefaultStream = 0xda3e39cb94b95bdbULL;

    explicit Pcg32(std::uint64_t seed = 0x853c49e6748fea9bULL,
                   std::uint64_t stream = kDefaultStream) {
        reseed(seed, stream);
    }

    void reseed(std::uint64_t seed, std::uint64_t stream = kDefaultStream) {
        mState = 0u;
        mInc = (stream << 1u) | 1u;
        nextU32();
        mState += seed;
        nextU32();
        mHaveSpare = false;
    }

    std::uint32_t nextU32() {
        const std::uint64_t old = mState;
        mState = old * 6364136223846793005ULL + mInc;
        const std::uint32_t xorshifted =
            static_cast<std::uint32_t>(((old >> 18u) ^ old) >> 27u);
        const std::uint32_t rot = static_cast<std::uint32_t>(old >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((32u - rot) & 31u));
    }

    /// Uniform on [0, 1). 24 bits, so every value is exactly representable and
    /// 1.0f is never returned -- a resampling walk that saw 1.0 would step off
    /// the end of the cumulative array.
    float uniform() {
        return static_cast<float>(nextU32() >> 8) * (1.0f / 16777216.0f);
    }

    float uniform(float lo, float hi) { return lo + (hi - lo) * uniform(); }

    /// Standard normal by the Marsaglia polar method: one log and one sqrt per
    /// *pair*, with the second variate kept. Box-Muller costs a log plus a
    /// sin/cos pair per single variate.
    float normal();

    void discardSpare() { mHaveSpare = false; }

private:
    std::uint64_t mState{0};
    std::uint64_t mInc{0};
    float mSpare{0.0f};
    bool mHaveSpare{false};
};

/// A wall or a flat field structure, as a segment in field coordinates.
struct MclLine {
    Pose p1{}, p2{};
};

/// A cylindrical field structure.
struct MclCircle {
    float x{0.0f}, y{0.0f}, radius{0.0f};
};

/// Everything a distance sensor can range off, held as spans rather than fixed
/// arrays: a test can substitute its own rectangle, and an empty season list
/// needs no zero-length array.
struct MclFieldMap {
    const MclLine* lines{nullptr};
    int lineCount{0};
    const MclCircle* circles{nullptr};
    int circleCount{0};
    /// Half the interior span. Particles outside it are penalised.
    float halfWall{Field::FIELD_HALF_WALL};
};

/// The particle filter with no clock, no hardware and no hidden randomness, so
/// all of it can be exercised off-robot. `MclTracking` is this class driven by
/// PROS distance sensors on a PROS task.
class MclCore {
public:
    static constexpr int PARTICLE_MAX = 1024;
    static constexpr int SENSOR_COUNT = MAX_DISTANCE_SENSORS;

    using LineObstacle = MclLine;
    using CircleObstacle = MclCircle;
    using FieldMap = MclFieldMap;

    /// Deliberately fixed. A brain with no wall clock and no entropy source has
    /// nothing better to seed from, and a filter that behaves the same way on
    /// every run is one whose field failures can be reproduced. setSeed() moves
    /// off it.
    static constexpr std::uint32_t kDefaultSeed = 0x9e3779b9u;

    // Field order is load-bearing: gamespecific/config.cpp initialises this with
    // designated initialisers, which C++ requires to be in declaration order.
    // Append new fields at the end.
    struct Config {
        int particleCount = 1024;   // [yours]
        // Only gates the no-measurement case: step() resamples unconditionally
        // whenever a beam scored. See the note there.
        int resampleThreshold = 512;      // N_eff floor, no-measurement only [model]
        float maxSensorRange = 100.0f;     // inches [yours]
        // Steady-state roughening only; the recovery search width is
        // distSearchVariance, applied by fit residual.
        //
        // NOT what the robot runs -- config.cpp ships 1.5, and that is where the
        // trade was settled on 648 paired kidnaps. 0.15 tracks 2.4x better
        // (0.65 in mean, 1.7 in worst, against 1.56 and 7.2) and recovers worse
        // on the pose actually driven on: 528/648 fused against 550, McNemar
        // p = 0.0013. 1.0 and 0.6 lose the same way at p = 0.003 and 0.024.
        float distResampleVariance = 0.15f; // inches, jitter added during resample [sim]
        float thetaResampleVariance = 0.001f;  // radians [sim]
        float maxThetaDeviation = 0.08f;   // radians, clamp particle theta vs IMU [model]

        // Multipliers on the fractional error of each measured axis, so they are
        // dimensionless and scale with how far the robot actually moved.
        float trackingWheelVariance = 0.08f;   // dedicated tracking (dead) wheels [model]
        float imeVariance = 0.12f;             // integrated motor encoders (more slip) [robot]
        float imuVariance = 0.005f;   // radians per update, absolute [model]

        // Lateral slip when nothing measures it: a term proportional to forward
        // travel plus a constant per-update floor.
        float horizDependentVarianceProp = 0.03f;  // multiplied by |dVert| [sim]
        float horizConstantNoise = 0.03f;          // inches/update baseline [robot]

        // Trades kidnap latency for smoothness, and only applies when the EKF is
        // off -- with it on, it owns the writeback and this is inert. 0.06 is
        // 1.75x smoother than 0.10 for 0.3 s more recovery; 0.03 is 3.1x for
        // 1.1 s. Raise it back toward 0.10 if recovery speed matters more.
        float distSyncProp = 0.06f;        // lerp factor for syncing to chassis [sim]
        float thetaSyncProp = 0.001f;   // [sim]
        float updatePeriodMs = 25.0f;      // background task interval [yours]
        int confidenceThreshold = 10;      // min sensor confidence [yours]
        float minSensorRange = 1.0f;       // mm, readings below this are invalid [yours]

        // A beam is dropped when the obstacle is what it actually RANGED: inside
        // the cone AND at the reported distance. Cone alone blanks a wide region
        // and discards good wall readings; a pencil ray misses most real hits.
        // Judged per particle -- deciding once from the estimate would use the
        // pose to choose which readings may correct it.
        float obstacleConeFrac = 0.22f;    // tan(12.5 deg), the V5 half-cone [model]
        float obstacleRangeTol = 8.0f;     // inches; reading vs obstacle surface [model]

        // Score for a beam blocked for this particle, as a probability DENSITY
        // per inch of reading -- the beam terms below are normalised to the same
        // units, so the two are comparable. 1/(2 * obstacleRangeTol) is the
        // uniform over the window the deadzone test accepts, which is where 0.06
        // comes from. Not 1.0, or particles would win by hiding behind obstacles.
        float deadzoneLikelihood = 0.06f;   // per inch [model]

        // Beam sigma. Close readings carry roughly fixed noise, far ones scale
        // with the reading. Measure both with Characterizer::measureRangeNoise()
        // rather than trusting these: they came off one set of sensors.
        // Each beam keeps its own physical sigma. There is deliberately no
        // sqrt(beams/4) factor holding the total Fisher information at the
        // four-beam value: it halved a lone beam's sigma, so one grazing reading
        // was trusted twice as hard as it would have been among four. Particle
        // depletion under many beams is the roughening's job, not the sensor
        // model's.
        float rangeSigmaCloseIn = 0.787f;   // used at or below 200 mm [robot]
        float rangeSigmaProp = 0.05f;       // fraction of the reading beyond that [robot]

        // Beam mixture, Probabilistic Robotics 6.3. A pure Gaussian says a
        // reading 6 in short is impossible, so one robot in the way annihilates
        // the whole cloud; the extra terms make such a reading uninformative
        // instead of fatal, and give the tails a slope so a badly placed particle
        // still knows which way to improve. Residuals are in sigmas, which folds
        // the short term's decay length into the range-dependent sigma.
        // The floor has to stay small. At 0.02 a beam reading 95 in away from
        // what a particle predicted costs the same as one reading 8 sigma away,
        // so a particle on the wrong side of the field is competitive with the
        // right one and the cloud migrates to a false mode.
        //
        // The short term departs from Table 6.1 knowingly: the book's decays
        // outward from the SENSOR, so it is small at z*, where this one is at
        // its maximum -- hence the 0.405 nat step at a zero residual and a
        // 0.229 sigma bias away from the wall on a flat prior. Kept because the
        // book's form is exponential in INCHES, so in sigma space its height
        // depends on z*/sigma and cannot be tabulated once: a logf per particle
        // per beam against a table that costs none. The bias is bounded by the
        // converged cloud's width, not by the tail, so it measures in
        // hundredths of an inch on real geometry (zzCritic D5, D16) against a
        // 0.13 in noise-driven error.
        float hitWeight = 1.0f;          // weight on the Gaussian [sim]
        float shortWeight = 0.50f;       // weight on the unexpected-obstacle tail [sim]
        float shortDecaySigmas = 1.2f;   // decay of that tail, in sigmas [sim]
        float randWeight = 0.0008f;      // floor; no single beam can zero a particle [sim]

        // Augmented MCL recovery (Probabilistic Robotics 8.3.2, in logs). A
        // kidnapped robot makes every particle equally wrong, so N_eff stays
        // high and plain resampling never fires. Comparing a fast and a slow
        // average of the measurement likelihood detects that, and the gap sets
        // how much of the cloud is redrawn across the field.
        // The gap is measured per beam, so a beam dropping in or out of range
        // does not read as a displacement.
        float alphaSlow = 0.005f;   // [model]
        float alphaFast = 0.10f;   // [model]
        //
        // Field-wide injection, still OFF by default. Recovery from a mid-run
        // displacement is now the mismatch-driven search roughening below, which
        // needs no global search. Injection only earns its keep for a true
        // kidnapping, and validating it needs the season's real post geometry on
        // a robot: an injected particle that points a beam at an unmapped post
        // gets that beam deadzoned instead of scored, and whether that beats the
        // truth depends on where the posts are.
        float maxRandomFrac = 0.0f;      // cap on the share redrawn per resample [model]
        float injectResampleFloor = 0.03f;  // mismatch deadband [model]

        bool autoSync = true;              // automatically sync pose to odometry [model]

        // Appended after the fields gamespecific/config.cpp names by designator.

        /// Feed "nothing in range" readings to the filter as evidence that no
        /// surface is within maxSensorRange, instead of discarding them. This is
        /// the p_max term of the textbook beam model and it adds real
        /// information, but it also gives a mis-set maxSensorRange a way to
        /// punish the truth. Off preserves the tuned behaviour.
        bool useNoReturnBeams = false;   // [model]

        /// Per-update forward position noise that does not scale with travel,
        /// inches. The measured-axis noise above is purely proportional, so at
        /// zero a parked cloud has no forward diffusion at all and resample
        /// jitter is the only thing spreading it. Non-zero is the textbook
        /// model; zero is what the current tuning was measured with.
        float vertConstantNoise = 0.0f;   // [model]

        /// Cloud width when the filter is searching rather than tracking: the
        /// roughening used once the measurements say the cloud is in the wrong
        /// place. Acquisition is acquireVariance, further down.
        ///
        /// 12 in. Wider than distResampleVariance by design, and safe only
        /// because of the ramp driver below: on the old fast/slow driver even 6
        /// leaked into healthy tracking and produced 60 in excursions. Over 648
        /// paired kidnaps, 6 recovers 628 raw and 515 fused where 12 recovers
        /// 637 and 550 (McNemar p = 0.039 and 0.0000).
        float distSearchVariance = 12.0f;   // inches [sim]
        float thetaSearchVariance = 0.01f;  // radians [sim]

        /// The search ramp, in searchMismatch() units. Below the floor the
        /// roughening is distResampleVariance and nothing widens, so ordinary
        /// measurement noise cannot re-inflate a converged cloud; at full it is
        /// distSearchVariance.
        ///
        /// Both are read against searchMismatch(), NOT likelihoodMismatch().
        /// injectResampleFloor used to serve as this floor as well, which is
        /// wrong now that the two signals are different quantities on different
        /// scales -- the mean-based one reaches 0.36 during healthy driving, so
        /// any band that separated anything would have to sit above that and
        /// leave nothing between healthy and lost.
        float searchMismatchFloor = 0.5f;   // [sim]
        float searchMismatchFull = 0.9f;    // [sim]

        /// The estimate's MEDIAN range residual, in sigmas, at which the filter
        /// calls itself lost -- searchMismatch() is fitSigmas() divided by this
        /// and clamped, so the band above is 4 to 7.2 sigmas. A perfectly placed
        /// estimate reads 1 by the definition of a sigma, and the host tests
        /// measure 0.25 while tracking, 0.26 at 2.5 in out, 0.70 at 10 in and
        /// 6.47 after a 24 in kidnap: an order of magnitude of separation rather
        /// than a tuned threshold. Zero disables the search ramp entirely.
        ///
        /// 8 rather than 12: over 648 paired kidnaps the fused pose recovers
        /// 550/648 against 534 (McNemar p = 0.0035), raw MCL does not move
        /// (p = 0.31), and tracking is unchanged. 4 adds 3 more, which is noise,
        /// for half the margin between the floor and a real calibration
        /// residual -- and the sim cannot see calibration error at all.
        float lostFitSigmas = 8.0f;   // [sim]

        /// Scales the per-beam residual cap in updateFit, which is
        /// lostFitCapProp * lostFitSigmas * searchMismatchFloor / alphaFast --
        /// expressed against the band so the two cannot drift apart. Measured
        /// safe over 0.4 to 1.4; 0 and 3.0 both let one outlier tick fire the
        /// ramp. 0 disables the cap.
        float lostFitCapProp = 0.8f;   // [sim]

        /// The spread setPose() seeds, which used to be distSearchVariance.
        /// Splitting them because raising the search width to 12 for kidnap
        /// recovery also widened the cloud at the start of every run, and that
        /// showed up as trajectory-following error in the first second: the
        /// trajectory suite's landing error went from 0.79 in to 1.76 in on one
        /// path. Acquisition wants the tightest envelope that still covers where
        /// the robot might actually be; recovery wants one that can walk 24 in.
        float acquireVariance = 6.0f;        // inches [sim]
        float acquireThetaVariance = 0.01f;  // radians [sim]
    };

    /// Pose estimate plus the cloud's own account of how sure it is.
    struct Estimate {
        Pose pose{};
        float nEff{0.0f};
        float covXX{0.0f}, covXY{0.0f}, covYY{0.0f};  // inches^2
        float covTheta{0.0f};                         // radians^2
    };

    struct Particle {
        Pose pose{};
        /// Always Rotation2d::fromRadians(pose.theta). Cached because the motion
        /// model, the ray cast and the circular mean all want it, so one exact
        /// sin/cos pair per tick replaces three LUT lookups.
        Rotation2d rot{};
        float weight{1.0f};
        /// Weights accumulate in logs and are exponentiated relative to the
        /// cloud maximum. Multiplying eight beam likelihoods directly underflows
        /// to zero for every particle at once, and the only recovery from that
        /// is a uniform reset -- throwing the filter away exactly when it was
        /// merely slightly wrong.
        float logWeight{0.0f};
    };

    /// One validated reading, reduced to what the filter needs. A no-return is
    /// carried as a reading at maxSensorRange, so weighting has no special case.
    struct Beam {
        int sensor{0};
        float rangeIn{0.0f};
        float invSigma{1.0f};
    };

    /// Raw per-sensor state, as read off the hardware.
    struct RawSensor {
        int mm{0};
        int confidence{0};
        bool present{false};
        /// Slot switched off, or a mapped blocker is known to be in the way.
        /// Either way the reading is not a field measurement.
        bool suppressed{false};
    };

    struct MotionInput {
        OdomDelta delta{};
        /// Only read when hasImu. A faulted V5 IMU HOLDS its last good heading,
        /// which is finite and looks exactly like a robot that is not turning,
        /// so a sentinel cannot carry this and the flag defaults to false.
        float imuHeadingRad{0.0f};
        bool hasImu{false};
        bool hasVertWheel{false};
        bool hasHorizWheel{false};
        float dtSeconds{0.025f};
    };

    MclCore();
    explicit MclCore(const Config& config, std::uint32_t seed = kDefaultSeed);

    const Config& config() const { return mConfig; }
    /// particleCount is held: resizing the cloud would invalidate the buffers a
    /// background task may be walking.
    void setConfig(const Config& config);
    void setSeed(std::uint32_t seed);
    Pcg32& rng() { return mRng; }

    void setFieldMap(const FieldMap& map) { mMap = map; }
    const FieldMap& fieldMap() const { return mMap; }

    /// Segments that, if a beam reaches one, mean the beam is not measuring the
    /// field. Judged once from the reference pose in buildBeams. Storage stays
    /// the caller's.
    void setBlockers(const LineObstacle* lines, int count);

    /// Round structures a beam may have ranged instead of a wall. Judged per
    /// particle in updateWeights. Storage stays the caller's.
    void setDeadzoneCircles(const CircleObstacle* circles, int count);

    /// Robot-frame sensor poses: x forward, y left, theta the pointing angle.
    void setSensorMounts(const std::array<Pose, SENSOR_COUNT>& mounts);
    const std::array<Pose, SENSOR_COUNT>& sensorMounts() const { return mMounts; }

    void setDrift(float vertPerUpdate, float horizPerUpdate) {
        mVertDrift = vertPerUpdate;
        mHorizDrift = horizPerUpdate;
    }

    void setPose(const Pose& pose);
    /// Scatter the cloud over the whole field with uniform heading.
    void uniformReset();

    void setParticles(const Particle* particles, int count);
    const Particle* particles() const { return mParticles; }
    int particleCount() const { return mConfig.particleCount; }

    Pose rawEstimate() const { return mRawEstimate; }
    Estimate estimate() const;
    /// The estimate step() computed, weighted by the measurements it had just
    /// applied. Prefer this to estimate() after a step: resample() flattens
    /// every weight to 1, so a recomputed estimate reports nEff == N however
    /// badly the cloud is doing, and reports the roughening kernel's spread
    /// rather than the evidence's.
    Estimate stepEstimate() const { return mStepEstimate; }
    float injectFraction() const { return mInjectFrac; }
    /// How badly the measurements fit the cloud's AVERAGE particle, 0 to 1: the
    /// augmented-MCL fast/slow log-likelihood gap, per beam. This is what
    /// injectFraction() is capped from, because injection replaces cloud mass
    /// and the mean is the right question for that.
    ///
    /// It is the wrong question for the search width. The mean falls when the
    /// cloud merely gets wider, so widening the cloud raises this, which widens
    /// it further -- and in the simulator it reaches 0.36 during perfectly
    /// healthy driving, against a searchMismatchFull of 0.35. Use
    /// searchMismatch() for anything that reacts by changing the spread.
    float likelihoodMismatch() const { return mMismatch; }
    /// How badly the published estimate itself fails to explain the readings:
    /// its MEDIAN range residual in sigmas, smoothed at alphaFast -- median,
    /// not mean, so one beam on something unmapped cannot declare a healthy
    /// filter lost. This is what
    /// drives the search width. It is ABSOLUTE, where likelihoodMismatch() is a
    /// ratio against the filter's own recent history -- which is the whole
    /// point, because a ratio against your own history rises when you widen the
    /// cloud, so widening in response to it widens further.
    float fitSigmas() const { return mFitSigmas; }
    /// fitSigmas() mapped onto 0..1 through lostFitSigmas, which is the form
    /// injectResampleFloor and searchMismatchFull are expressed in.
    float searchMismatch() const { return mSearchMismatch; }
    float lastSpeed() const { return mLastSpeed; }
    bool resampledLast() const { return mResampledLast; }
    /// Resamples since construction or the last setPose(). resampledLast() only
    /// answers for the tick just gone, which cannot show a filter that has
    /// stopped resampling intermittently.
    std::uint32_t resampleCount() const { return mResampleCount; }
    const std::array<float, SENSOR_COUNT>& deadzoneFraction() const { return mDeadzoneFrac; }

    int snapshotParticles(float* out, int maxParticles) const;

    void predict(const MotionInput& in);
    void updateWeights(const Beam* beams, int count);
    /// `hasImu` false means no absolute heading exists, so injected particles
    /// inherit the cloud's own heading instead of the argument.
    void resample(float imuHeadingRad, bool hasImu = true);
    /// predict, weight, then resample if the cloud has degenerated. Returns the
    /// new estimate, which is also published as rawEstimate().
    Pose step(const MotionInput& in, const Beam* beams, int beamCount);

    /// Turn raw readings into weighted beams. `valid` receives one flag per
    /// slot. `reference` supplies the pose the incidence-angle sigma scaling and
    /// the blocker test are judged from.
    int buildBeams(const RawSensor* raw, int count, const Pose& reference,
                   Beam* out, bool* valid) const;

    /// Distance from (x, y) along the unit direction (dirCos, dirSin) to the
    /// nearest mapped surface, clamped to maxSensorRange.
    float castRay(float x, float y, float dirCos, float dirSin) const;

    /// The same cast, also reporting the angle between the ray and the normal of
    /// the surface it actually reached, in [0, pi/2]. A miss reports 0. Which
    /// wall a ray hits depends on where the robot is, so the incidence cannot be
    /// inferred from the heading alone.
    float castRayIncidence(float x, float y, float dirCos, float dirSin,
                           float* incidenceRad) const;

    /// What sensor `sensor` would read from `pose` if the map were the whole
    /// truth: mount offset rotated into the field, mount yaw composed with the
    /// heading. This is precisely the prediction weighting compares against.
    float expectedRange(const Pose& pose, int sensor) const;

    /// Nearest hit of a ray against one segment, or `maxRange` for a miss.
    static float intersectLine(float ox, float oy, float dirCos, float dirSin,
                               const LineObstacle& line, float maxRange);
    /// Nearest hit against one circle, or `maxRange`.
    static float intersectCircle(float ox, float oy, float dirCos, float dirSin,
                                 const CircleObstacle& circle, float maxRange);

    /// Would `rangeIn` be explained by a deadzone circle rather than the map?
    bool beamDeadzoned(float ox, float oy, float dirCos, float dirSin,
                       float rangeIn) const;

    /// Log of the beam mixture at a signed residual in sigmas; negative means
    /// the reading came back shorter than predicted. LUT plus interpolation.
    /// The unexpected-obstacle term switches on at a zero residual, so the two
    /// halves are tabulated separately and never interpolated across each other.
    float logBeamLikelihood(float sigmas) const;
    /// The same quantity evaluated directly, for bounding the LUT error.
    static float logBeamLikelihoodExact(const Config& config, float sigmas);
    /// log of the constant that turns the sigma-space mixture above into a
    /// probability density per inch: subtract it and add log(1/sigma). Without
    /// it the beam terms are unnormalised and deadzoneLikelihood, which is a
    /// real density, is not comparable with them.
    static float logMixtureNorm(const Config& config);

    /// Low-variance (systematic) resampling, Probabilistic Robotics table 4.4:
    /// one uniform draw and a single sweep, giving variance O(N) below
    /// multinomial. Writes `outCount` source indices.
    static void systematicResample(const float* weights, int count, float u01,
                                   int* out, int outCount);

    /// ESS = (sum w)^2 / sum w^2. Equals `count` for a flat cloud and 1 when a
    /// single particle holds all the weight.
    static float effectiveSampleSize(const float* weights, int count);

    /// Circular mean: atan2 of the weighted unit vectors. Averaging radians
    /// arithmetically is wrong across the +/-pi branch cut.
    static float circularMean(const float* angles, const float* weights, int count);

    static constexpr float LUT_SIGMA_SPAN = 8.0f;
    /// Floor on any particle's log weight. A cloud that has all reached it must
    /// still produce a finite pose.
    static constexpr float LOG_FLOOR = -80.0f;

private:
    void buildLut();
    float beamSigma(const RawSensor& raw, float rangeIn, bool noReturn,
                    float incidenceRad) const;
    /// Roughening for this resample: distResampleVariance while the cloud is
    /// tracking, ramping to distSearchVariance once it stops fitting.
    void roughening(float* distSigma, float* thetaSigma) const;
    /// Score the published estimate against the same beams the cloud was
    /// weighted on, and update fitSigmas()/searchMismatch() from it.
    void updateFit(const Pose& pose, const Beam* beams, int count);

    /// Two half tables, each with a node exactly at a zero residual, so the
    /// step in the mixture there is never smoothed across.
    static constexpr int LUT_HALF = 512;
    static constexpr int LUT_SIZE = 2 * LUT_HALF;

    Config mConfig;
    FieldMap mMap{};
    Pcg32 mRng;

    const LineObstacle* mBlockers{nullptr};
    int mBlockerCount{0};
    const CircleObstacle* mDeadzoneCircles{nullptr};
    int mDeadzoneCircleCount{0};

    std::array<Pose, SENSOR_COUNT> mMounts{};
    std::array<Rotation2d, SENSOR_COUNT> mMountRots{};

    Particle mParticlesA[PARTICLE_MAX]{};
    Particle mParticlesB[PARTICLE_MAX]{};
    Particle* mParticles{mParticlesA};
    Particle* mScratch{mParticlesB};
    float mWeightScratch[PARTICLE_MAX]{};
    int mIndexScratch[PARTICLE_MAX]{};

    float mLut[LUT_SIZE]{};
    float mLogMixtureNorm{0.0f};

    std::array<float, SENSOR_COUNT> mDeadzoneFrac{};

    float mVertDrift{0.0f};
    float mHorizDrift{0.0f};

    Pose mRawEstimate{};
    Estimate mStepEstimate{};
    float mLastSpeed{0.0f};
    int mScoredBeams{0};
    bool mResampledLast{false};

    float mLogWSlow{0.0f};
    float mLogWFast{0.0f};
    bool mHaveLogAvg{false};
    float mFitSigmas{0.0f};
    bool mHaveFit{false};
    float mMismatch{0.0f};
    float mSearchMismatch{0.0f};
    float mInjectFrac{0.0f};
    std::uint32_t mResampleCount{0};
};

/// Field geometry. MODIFY PER SEASON.
namespace mclfield {

/// Everything a distance sensor can range off. The perimeter is the VRC
/// 140.4 in interior; append this season's fixed structures. The cast takes the
/// nearest hit over the whole list, so order does not matter.
inline constexpr std::array<MclLine, 4> kSurfaces = {{
    {{-70.2f, -70.2f}, { 70.2f, -70.2f}},
    {{ 70.2f, -70.2f}, { 70.2f,  70.2f}},
    {{ 70.2f,  70.2f}, {-70.2f,  70.2f}},
    {{-70.2f,  70.2f}, {-70.2f, -70.2f}},
}};

/// Round structures a sensor can range off: match loader posts and the like.
inline constexpr std::array<MclCircle, 0> kCircularSurfaces{};

/// A beam that reaches one of these is not measuring the field, so the reading
/// is dropped outright rather than fitted.
inline constexpr std::array<MclLine, 0> kBlockers{};

inline MclFieldMap map() {
    return {kSurfaces.data(), static_cast<int>(kSurfaces.size()),
            kCircularSurfaces.data(), static_cast<int>(kCircularSurfaces.size()),
            Field::FIELD_HALF_WALL};
}

}  // namespace mclfield

}  // namespace chad
