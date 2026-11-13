using HySlam.Core.Data;
using OpenCvSharp;
using Serilog;

using GLVector3 = OpenTK.Mathematics.Vector3;

namespace HySlam.Core.Slam
{
    /// <summary>
    /// Monocular visual odometry constrained to a flat road surface.
    ///
    /// The camera is assumed to move on a horizontal plane at a known height above the
    /// road. Matched road features are projected onto that plane in both frames and a
    /// robust 2-D rigid transform (yaw + translation) is fitted. This removes the scale
    /// ambiguity, vertical drift and roll/pitch drift of a general 6-DOF estimate and
    /// produces a metric (meter) trajectory.
    ///
    /// The map combines exact road-surface points, obtained from the ground-plane
    /// projection, with triangulated above-road structure that is observed from the
    /// current keyframe. All points keep the colour sampled from the source video.
    /// </summary>
    internal sealed class SlamEngine : IDisposable
    {
        // === Tracking ===
        private const int MinTrackedPoints = 40;
        private const int MinInliers = 20;
        private const double MinInlierRatio = 0.40;
        private const double DefaultMaxLkError = 20.0;
        private const double ForwardBackwardErrorPx = 1.0;

        // === Ground-plane odometry ===
        // Fraction of the image height where the road horizon sits. The measurement
        // uses the lane-line vanishing point and the flow focus of expansion, both of
        // which consistently land near 0.665 of the frame height on dashcam footage.
        private const double DefaultGroundHorizonFraction = 0.665;

        private const double RansacTolerance = 0.25;    // meters
        private const int RansacIterations = 400;
        private const double MaxGroundResidual = 0.35;  // meters
        private const double MaxYawStepDegrees = 8.0;
        private const double DefaultMaxTranslationStep = 3.0;  // meters per processed frame
        private const double YawSmoothing = 0.20;
        private const double TranslationSmoothing = 0.30;
        private const int MaxPredictedFrames = 2;

        // === Map ===
        private const double SkyMargin = 0.18;          // ignored top fraction of the image
        private const double MinStructureDepth = 0.5;   // meters
        private const double MaxReprojErrPx = 2.0;
        private const double MinParallaxDeg = 0.3;
        private const double MinStructureHeight = 0.4;  // meters above the road
        private const int DefaultMaxFramesPerKeyframe = 30;
        private const int MaxRoadPointsPerFrame = 160;
        private const int DefaultMaxStructurePointsPerFrame = 220;
        private const int MaxMapPoints = 150000;

        // Structure triangulation is the most expensive step; run it every N frames
        // instead of every frame to keep the UI responsive on long clips.
        private const int StructureUpdateInterval = 3;

        private readonly MapData map;
        private readonly KeyframeManager keyframes;
        private readonly MotionPredictor motionPredictor = new();
        private readonly VoxelGrid spatialIndex = new();
        private readonly CameraPose currentPose = new();
        private readonly Mat gray = new();

        private Mat cameraMatrix = null!;
        private Mat? distortion;
        private Mat? undistorted;
        private bool hasDistortion;

        private double fx, fy, cx, cy;

        // Ground plane in camera coordinates: n . X = cameraHeight.
        private double groundHeight = 1.3;
        private double normalX, normalY = 1.0, normalZ;

        // Scene-dependent depth window and detection band. The road defaults keep the
        // previous behaviour; an indoor scene overrides them through the CLI preset.
        private double minGroundDepth = 6.0;
        private double maxGroundDepth = 26.0;
        private double maxStructureDepth = 45.0;
        private double groundBandTop = 0.70;
        private double groundBandBottom = 0.99;
        private int maxFramesPerKeyframe = DefaultMaxFramesPerKeyframe;
        private double maxTranslationStep = DefaultMaxTranslationStep;
        private int maxStructureKeyframes = MaxStructureKeyframes;
        private int maxStructurePointsPerFrame = DefaultMaxStructurePointsPerFrame;

        // Multi-keyframe structure: triangulate structure points from the last N
        // keyframes instead of a single one. This makes the map denser and more
        // consistent on complex city roads and high-speed tracks.
        private const int MaxStructureKeyframes = 3;

        // Lucas-Kanade parameters. A high-speed clip needs a wider window and one
        // more pyramid level, otherwise the per-frame displacement exceeds the
        // search range and every match is rejected.
        private int lkWindowSize = 21;
        private int lkLevels = 3;
        private double maxLkError = DefaultMaxLkError;

        // === Frame-to-frame tracking state ===
        private Mat? previousGray;
        private List<Point2f> previousPoints = new();
        private bool hasPreviousFrame;

        // === Bake support ===
        // A previous bake pass can supply the expected motion step for each processed
        // frame. The estimate is still fitted from the image; the prior only seeds the
        // predictor and takes over when tracking fails.
        private IReadOnlyList<PosePriorStep>? posePrior;

        // Diagnostics: trace the first few calls in detail so a native crash inside
        // OpenCV can be located from the last logged stage.
        private const int TracedCalls = 4;
        private int processCalls;

        // === Metric pose state ===
        // World frame: x right, y down, z forward, origin on the road below the
        // first camera position. The camera therefore sits at y = -CameraHeight.
        private double heading;
        private double centerX, centerZ;
        private double smoothedYawStep;
        private double smoothedStepX, smoothedStepZ;
        private bool hasSmoothedMotion;

        private bool hasLastMotion;
        private int framesSinceMotion;

        private int featureCount, matchCount, inlierCount, triangulatedCount;
        private int structureCalls;
        private int processedFrames;
        private int lastKeyframeFrame;

        // Deterministic RNG for the custom RANSAC (xorshift64).
        private ulong rngState = 0x9E3779B97F4A7C15UL;

        public CameraPose Pose => currentPose;
        public int FeatureCount => featureCount;
        public int MatchCount => matchCount;
        public int InlierCount => inlierCount;
        public int TriangulatedCount => triangulatedCount;

        /// <summary>True when the last processed frame produced an accepted motion fit.</summary>
        public bool LastMotionValid { get; private set; } = true;

        /// <summary>Motion step applied by the last processed frame (for bake priors).</summary>
        public PosePriorStep LastStep { get; private set; }

        /// <summary>Number of processed frames since Initialize (bake progress).</summary>
        public int ProcessedFrames => processedFrames;

        /// <summary>
        /// The undistorted frame used for tracking, or null when no distortion
        /// correction is active. The preview should use the same frame so overlays
        /// line up with the tracked points.
        /// </summary>
        public Mat? UndistortedFrame => hasDistortion ? undistorted : null;

        public SlamEngine(MapData map, KeyframeManager keyframes)
        {
            this.map = map;
            this.keyframes = keyframes;
        }

        public void Initialize(
            double width,
            double height,
            CameraCalibration? calibration = null,
            double cameraHeight = 1.3,
            double? pitchDegrees = null,
            double horizontalFovDegrees = 0.0,
            double minGroundDepth = 6.0,
            double maxGroundDepth = 26.0,
            double maxStructureDepth = 45.0,
            double groundBandTop = 0.70,
            double groundBandBottom = 0.99,
            int maxFramesPerKeyframe = DefaultMaxFramesPerKeyframe,
            double maxTranslationStep = DefaultMaxTranslationStep,
            int lkWindowSize = 21,
            int lkLevels = 3,
            double maxLkError = DefaultMaxLkError,
            int maxStructureKeyframes = MaxStructureKeyframes,
            int maxStructurePointsPerFrame = DefaultMaxStructurePointsPerFrame)
        {
            var calib = (calibration ?? CameraCalibration.Estimate(width, height, horizontalFovDegrees))
                .ScaledTo(width, height);

            fx = calib.Fx;
            fy = calib.Fy;
            cx = calib.Cx;
            cy = calib.Cy;

            cameraMatrix?.Dispose();
            cameraMatrix = new Mat(3, 3, MatType.CV_64FC1, Scalar.All(0));
            cameraMatrix.Set(0, 0, fx);
            cameraMatrix.Set(1, 1, fy);
            cameraMatrix.Set(0, 2, cx);
            cameraMatrix.Set(1, 2, cy);
            cameraMatrix.Set(2, 2, 1.0);

            distortion?.Dispose();
            if (calib.HasDistortion)
            {
                hasDistortion = true;
                distortion = new Mat(1, 5, MatType.CV_64FC1, Scalar.All(0));
                for (int i = 0; i < 5; i++)
                    distortion.Set(0, i, calib.Distortion[i]);

                undistorted ??= new Mat();
            }
            else
            {
                hasDistortion = false;
                distortion = null;
                undistorted?.Dispose();
                undistorted = null;
            }

            groundHeight = cameraHeight > 0 ? cameraHeight : 1.3;

            this.minGroundDepth = minGroundDepth > 0 ? minGroundDepth : 6.0;
            this.maxGroundDepth = maxGroundDepth > this.minGroundDepth ? maxGroundDepth : this.minGroundDepth + 1.0;
            this.maxStructureDepth = maxStructureDepth > 0 ? maxStructureDepth : 45.0;
            this.groundBandTop = Math.Clamp(groundBandTop, 0.0, 0.94);
            this.groundBandBottom = Math.Clamp(groundBandBottom, this.groundBandTop + 0.05, 1.0);
            this.maxFramesPerKeyframe = maxFramesPerKeyframe > 0 ? maxFramesPerKeyframe : DefaultMaxFramesPerKeyframe;
            this.maxTranslationStep = maxTranslationStep > 0 ? maxTranslationStep : DefaultMaxTranslationStep;
            this.maxStructureKeyframes = maxStructureKeyframes > 0 ? maxStructureKeyframes : MaxStructureKeyframes;
            this.maxStructurePointsPerFrame = maxStructurePointsPerFrame > 0
                ? maxStructurePointsPerFrame
                : DefaultMaxStructurePointsPerFrame;
            this.lkWindowSize = lkWindowSize >= 5 ? lkWindowSize : 21;
            this.lkLevels = Math.Clamp(lkLevels, 0, 6);
            this.maxLkError = maxLkError > 0 ? maxLkError : DefaultMaxLkError;

            // Positive pitch means the camera looks down, which tilts the ground
            // normal towards +z in camera coordinates. When the operator does not
            // supply a pitch we place the horizon at DefaultGroundHorizonFraction of
            // the image height. That value was measured from the lane-line vanishing
            // point and the optical-flow focus of expansion of typical dashcam
            // footage: the road horizon sits clearly below the image centre, so a
            // naive centred principal point would mis-scale every depth.
            double pitchRadians;
            if (pitchDegrees.HasValue)
            {
                pitchRadians = pitchDegrees.Value * Math.PI / 180.0;
            }
            else
            {
                double horizonRow = height * DefaultGroundHorizonFraction;
                pitchRadians = -Math.Atan((horizonRow - cy) / fy);
            }

            normalX = 0.0;
            normalY = Math.Cos(pitchRadians);
            normalZ = Math.Sin(pitchRadians);

            // Reset the metric pose and tracking state.
            heading = 0.0;
            centerX = 0.0;
            centerZ = 0.0;
            smoothedYawStep = 0.0;
            smoothedStepX = 0.0;
            smoothedStepZ = 0.0;
            hasSmoothedMotion = false;
            hasLastMotion = false;
            framesSinceMotion = 0;
            processedFrames = 0;
            lastKeyframeFrame = -maxFramesPerKeyframe;
            rngState = 0x9E3779B97F4A7C15UL;

            motionPredictor.Reset();

            previousGray?.Dispose();
            previousGray = null;
            previousPoints.Clear();
            hasPreviousFrame = false;

            UpdatePoseFromState();

            // Keep RANSAC inside OpenCV reproducible as well.
            Cv2.SetTheRNG(0x5EEDUL);
        }

        /// <summary>
        /// Applies an optional motion prior from a previous bake pass. Pass null to clear.
        /// </summary>
        public void ApplyPosePrior(IReadOnlyList<PosePriorStep>? prior)
        {
            posePrior = prior;

            if (prior is null || prior.Count == 0)
                return;

            motionPredictor.Seed(prior[0].YawStep, prior[0].StepX, prior[0].StepZ);
        }

        public bool ProcessFrame(Mat frame, out List<Point2f> keyframePoints, out List<Point2f> currentPoints)
        {
            keyframePoints = new List<Point2f>();
            currentPoints = new List<Point2f>();

            int callIndex = processCalls++;
            bool trace = callIndex < TracedCalls;

            if (trace)
            {
                Log.Information(
                    "ProcessFrame #{Call}: begin (size {Width}x{Height}, channels {Channels}, hasPrevious {HasPrevious}, previousPoints {PreviousPoints}).",
                    callIndex,
                    frame.Width,
                    frame.Height,
                    frame.Channels(),
                    hasPreviousFrame,
                    previousPoints.Count);
            }

            Mat colorFrame = frame;
            if (hasDistortion && undistorted is not null && distortion is not null)
            {
                Cv2.Undistort(frame, undistorted, cameraMatrix, distortion);
                colorFrame = undistorted;
            }

            if (colorFrame.Channels() == 3)
                Cv2.CvtColor(colorFrame, gray, ColorConversionCodes.BGR2GRAY);
            else
                colorFrame.CopyTo(gray);

            if (trace)
                Log.Information("ProcessFrame #{Call}: grayscale ready.", callIndex);

            // Two masks: odometry only uses the near-field road band, while keyframes
            // and structure triangulation need the full drivable view.
            using var groundMask = BuildGroundMask();
            using var structureMask = BuildTrackingMask();

            if (!hasPreviousFrame || previousGray is null || previousPoints.Count < MinTrackedPoints)
            {
                if (trace)
                    Log.Information("ProcessFrame #{Call}: first frame path, detecting features.", callIndex);

                DetectFeatures(groundMask);
                ReplacePreviousFrame(previousPoints);
                UpdatePoseFromState();
                MaybeCreateKeyframe(structureMask);
                LastMotionValid = true;

                if (trace)
                    Log.Information("ProcessFrame #{Call}: first frame path done.", callIndex);

                return false;
            }

            if (trace)
                Log.Information("ProcessFrame #{Call}: tracking {Points} previous points.", callIndex, previousPoints.Count);

            var (prevTracked, currTracked) = TrackAndValidate(previousGray, gray, previousPoints, groundMask);

            if (trace)
                Log.Information("ProcessFrame #{Call}: tracked {Tracked} points.", callIndex, currTracked.Count);

            featureCount = previousPoints.Count;
            matchCount = currTracked.Count;
            inlierCount = 0;
            triangulatedCount = 0;

            bool motionValid = false;
            bool[] inlierFlags = Array.Empty<bool>();

            // Forecast the next motion from the recent history. On fast footage the
            // inter-frame displacement can be large; the forecast helps decide whether
            // a planar fit is a genuine motion estimate or an outlier.
            var (predYaw, predStepX, predStepZ) = motionPredictor.Forecast();
            bool hasForecast = motionPredictor.HasPrediction;

            // A previous bake pass knows how the camera moved at this point of the clip.
            // Use that as the forecast; the current fit still has to agree with it.
            bool hasPrior = posePrior is not null && processedFrames < posePrior.Count;
            if (hasPrior)
            {
                PosePriorStep priorStep = posePrior![processedFrames];
                predYaw = priorStep.YawStep;
                predStepX = priorStep.StepX;
                predStepZ = priorStep.StepZ;
                hasForecast = true;
            }

            if (prevTracked.Count >= MinTrackedPoints)
            {
                motionValid = EstimatePlanarMotion(
                    prevTracked, currTracked, out double yawStep, out double tx, out double tz,
                    out int inliers, out inlierFlags, hasForecast, predYaw, predStepX, predStepZ);

                if (motionValid)
                {
                    // Trust the estimate only if it agrees with the prediction (or there
                    // is no prediction yet). Otherwise reject it as a tracking outlier.
                    bool predictionAgrees = !hasForecast ||
                        motionPredictor.Observe(yawStep, tx, tz, maxTranslationStep * 1.5);

                    if (predictionAgrees)
                    {
                        ApplyMotion(yawStep, tx, tz);
                        inlierCount = inliers;
                        hasLastMotion = true;
                        framesSinceMotion = 0;
                    }
                    else
                    {
                        motionValid = false;
                    }
                }
            }

            if (!motionValid && hasLastMotion && framesSinceMotion < MaxPredictedFrames)
            {
                // No usable fit this frame. Fall back to the previous pass's step when one
                // exists, otherwise to the smoothed prediction, so the trajectory does not
                // freeze on high-speed or low-texture segments.
                if (hasPrior)
                {
                    PosePriorStep priorStep = posePrior![processedFrames];
                    ApplyMotion(priorStep.YawStep, priorStep.StepX, priorStep.StepZ);
                    motionPredictor.Observe(
                        priorStep.YawStep, priorStep.StepX, priorStep.StepZ, maxTranslationStep * 2.0);
                }
                else
                {
                    ApplySmoothedStep();
                    motionPredictor.Observe(smoothedYawStep, smoothedStepX, smoothedStepZ, maxTranslationStep * 2.0);
                }

                framesSinceMotion++;
            }

            LastMotionValid = motionValid;

            UpdatePoseFromState();

            if (trace)
                Log.Information("ProcessFrame #{Call}: pose updated (motionValid {MotionValid}).", callIndex, motionValid);

            if (motionValid)
                AddRoadPoints(currTracked, inlierFlags, colorFrame);

            if (trace)
                Log.Information("ProcessFrame #{Call}: road points added (total {Total}).", callIndex, map.Points.Count);

            // Run structure triangulation only every N frames; it is the most
            // expensive step and running it every frame causes UI stutter.
            triangulatedCount = (processedFrames % StructureUpdateInterval == 0)
                ? AddStructurePoints(colorFrame)
                : 0;

            if (trace)
                Log.Information("ProcessFrame #{Call}: structure points added ({Added}).", callIndex, triangulatedCount);

            TrimMapIfNeeded();

            var camPos = currentPose.GetCameraPositionGL();
            if (map.Trajectory.Count == 0 || (camPos - map.Trajectory[^1]).Length > 0.02f)
                map.Trajectory.Add(camPos);

            // Always start the next step from a fresh near-field detection. Reusing only
            // the surviving matches lets the pool drift towards the far edge of the band
            // (features move down and leave the bottom), until the depth gate rejects
            // almost everything and the frame count collapses.
            DetectFeatures(groundMask);
            ReplacePreviousFrame(previousPoints);

            if (trace)
                Log.Information("ProcessFrame #{Call}: next point pool ready ({Points}).", callIndex, previousPoints.Count);

            processedFrames++;
            MaybeCreateKeyframe(structureMask);

            if (trace)
                Log.Information("ProcessFrame #{Call}: keyframe step done (id {KeyframeId}).", callIndex, keyframes.CurrentId);

            keyframePoints = prevTracked;
            currentPoints = currTracked;
            return true;
        }

        // ---------------------------------------------------------------------
        // Feature detection and tracking
        // ---------------------------------------------------------------------

        private void DetectFeatures(Mat mask)
        {
            Point2f[] corners = Cv2.GoodFeaturesToTrack(
                gray, 1200, 0.005, 5.0, mask, 7, false, 0.04);

            previousPoints = corners is { Length: > 0 }
                ? new List<Point2f>(corners)
                : new List<Point2f>();
        }

        private void ReplacePreviousFrame(IReadOnlyList<Point2f> points)
        {
            previousGray?.Dispose();
            previousGray = gray.Clone();
            previousPoints = new List<Point2f>(points);
            hasPreviousFrame = previousPoints.Count >= MinTrackedPoints;
        }

        private (List<Point2f> From, List<Point2f> To) TrackAndValidate(
            Mat fromImage,
            Mat toImage,
            IReadOnlyList<Point2f> input,
            Mat mask)
        {
            var from = new List<Point2f>();
            var to = new List<Point2f>();

            if (input.Count < 8)
                return (from, to);

            using var fromMat = Mat.FromArray(input.ToArray());
            using var toMat = new Mat();
            using var status = new Mat();
            using var error = new Mat();

            Cv2.CalcOpticalFlowPyrLK(
                fromImage, toImage, fromMat, toMat, status, error,
                new Size(lkWindowSize, lkWindowSize), lkLevels,
                new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.01));

            for (int i = 0; i < input.Count; i++)
            {
                if (status.At<byte>(i, 0) == 0) continue;
                if (error.At<float>(i, 0) > maxLkError) continue;

                var pt = toMat.At<Point2f>(i, 0);
                if (pt.X < 3 || pt.Y < 3 || pt.X >= toImage.Cols - 3 || pt.Y >= toImage.Rows - 3) continue;
                if (mask.At<byte>((int)pt.Y, (int)pt.X) == 0) continue;

                from.Add(input[i]);
                to.Add(pt);
            }

            if (to.Count < 8)
                return (new List<Point2f>(), new List<Point2f>());

            // Deliberately no CornerSubPix here. On road texture the refinement
            // snaps the tracked pixel onto the nearest high-contrast marking (usually
            // the dashed lane divider), which introduces a systematic yaw bias in the
            // planar fit. Raw pyramidal LK positions plus the forward-backward check
            // are unbiased and gave the validated trajectory.
            using var currentMat = Mat.FromArray(to.ToArray());
            using var backMat = new Mat();
            using var backStatus = new Mat();
            using var backError = new Mat();

            Cv2.CalcOpticalFlowPyrLK(
                toImage, fromImage, currentMat, backMat, backStatus, backError,
                new Size(lkWindowSize, lkWindowSize), lkLevels,
                new TermCriteria(CriteriaTypes.Eps | CriteriaTypes.MaxIter, 30, 0.01));

            var fbFrom = new List<Point2f>(to.Count);
            var fbTo = new List<Point2f>(to.Count);
            double maxErrorSq = ForwardBackwardErrorPx * ForwardBackwardErrorPx;

            for (int i = 0; i < to.Count; i++)
            {
                if (backStatus.At<byte>(i, 0) == 0) continue;

                var back = backMat.At<Point2f>(i, 0);
                double dx = back.X - from[i].X;
                double dy = back.Y - from[i].Y;
                if (dx * dx + dy * dy > maxErrorSq) continue;

                fbFrom.Add(from[i]);
                fbTo.Add(to[i]);
            }

            return (fbFrom, fbTo);
        }

        // ---------------------------------------------------------------------
        // Ground-plane odometry
        // ---------------------------------------------------------------------

        private bool TryProjectToGround(Point2f pixel, out double gx, out double gy, out double gz)
        {
            gx = gy = gz = 0.0;

            double rx = (pixel.X - cx) / fx;
            double ry = (pixel.Y - cy) / fy;
            double rz = 1.0;

            double denominator = rx * normalX + ry * normalY + rz * normalZ;
            if (denominator <= 1e-6)
                return false;

            double scale = groundHeight / denominator;
            gx = rx * scale;
            gy = ry * scale;
            gz = rz * scale;
            return true;
        }

        /// <summary>
        /// Estimates the rigid transform Q = R(theta) * P + t that maps the previous
        /// frame's ground points P onto the current frame's ground points Q. Only the
        /// yaw and the in-plane translation are solved, which is exactly the motion of
        /// a camera travelling on a flat road.
        /// </summary>
        private bool EstimatePlanarMotion(
            IReadOnlyList<Point2f> previous,
            IReadOnlyList<Point2f> current,
            out double theta,
            out double tx,
            out double tz,
            out int inliers,
            out bool[] inlierFlags,
            bool hasForecast = false,
            double predYaw = 0.0,
            double predStepX = 0.0,
            double predStepZ = 0.0)
        {
            theta = tx = tz = 0.0;
            inliers = 0;

            int count = Math.Min(previous.Count, current.Count);
            inlierFlags = new bool[count];

            var px = new List<double>(count);
            var pz = new List<double>(count);
            var qx = new List<double>(count);
            var qz = new List<double>(count);
            var sourceIndex = new List<int>(count);

            for (int i = 0; i < count; i++)
            {
                if (!TryProjectToGround(previous[i], out double pxv, out _, out double pzv)) continue;
                if (!TryProjectToGround(current[i], out double qxv, out _, out double qzv)) continue;

                if (pzv < minGroundDepth || pzv > maxGroundDepth) continue;
                if (qzv < minGroundDepth || qzv > maxGroundDepth) continue;

                // Reject points that are far outside the visible road wedge.
                if (Math.Abs(pxv) > 0.9 * pzv + 0.5) continue;
                if (Math.Abs(qxv) > 0.9 * qzv + 0.5) continue;

                px.Add(pxv);
                pz.Add(pzv);
                qx.Add(qxv);
                qz.Add(qzv);
                sourceIndex.Add(i);
            }

            int n = px.Count;
            if (n < MinInliers)
                return false;

            var bestInlier = new bool[n];
            int bestCount = 0;

            for (int iteration = 0; iteration < RansacIterations; iteration++)
            {
                int a = NextRandom(n);
                int b = NextRandom(n);
                if (a == b) continue;

                double pdx = px[a] - px[b];
                double pdz = pz[a] - pz[b];
                double qdx = qx[a] - qx[b];
                double qdz = qz[a] - qz[b];
                if (pdx * pdx + pdz * pdz < 1.0) continue; // degenerate sample
                if (qdx * qdx + qdz * qdz < 1.0) continue;

                FitRigid(px, pz, qx, qz, a, b, out double candidateTheta, out double candidateTx, out double candidateTz);

                var candidate = new bool[n];
                int candidateCount = CountInliers(
                    px, pz, qx, qz, candidateTheta, candidateTx, candidateTz, candidate);

                if (candidateCount > bestCount)
                {
                    bestCount = candidateCount;
                    bestInlier = candidate;
                }
            }

            if (bestCount < MinInliers)
                return false;

            // Refine twice on the inlier set.
            for (int pass = 0; pass < 2; pass++)
            {
                FitRigidFromMask(px, pz, qx, qz, bestInlier, out theta, out tx, out tz);

                var refined = new bool[n];
                int refinedCount = CountInliers(px, pz, qx, qz, theta, tx, tz, refined);
                if (refinedCount < MinInliers)
                    return false;

                bestInlier = refined;
                bestCount = refinedCount;
            }

            double yawDegrees = Math.Abs(theta) * 180.0 / Math.PI;
            if (yawDegrees > MaxYawStepDegrees)
                return false;

            double step = Math.Sqrt(tx * tx + tz * tz);
            if (step > maxTranslationStep)
                return false;

            var residuals = new List<double>(bestCount);
            for (int i = 0; i < n; i++)
            {
                if (!bestInlier[i]) continue;
                double predictedX = Math.Cos(theta) * px[i] - Math.Sin(theta) * pz[i] + tx;
                double predictedZ = Math.Sin(theta) * px[i] + Math.Cos(theta) * pz[i] + tz;
                double dx = predictedX - qx[i];
                double dz = predictedZ - qz[i];
                residuals.Add(Math.Sqrt(dx * dx + dz * dz));
            }

            residuals.Sort();
            double medianResidual = residuals.Count > 0 ? residuals[residuals.Count / 2] : double.MaxValue;

            if (medianResidual > MaxGroundResidual)
                return false;

            double ratio = bestCount / (double)n;
            if (ratio < MinInlierRatio)
                return false;

            // When a forecast exists, reject estimates that deviate too far from the
            // predicted constant-velocity motion. This stops tracking outliers (for
            // example a wall mistaken for the road on a fast city stretch) from
            // corrupting the trajectory.
            if (hasForecast)
            {
                double estYawDegrees = Math.Abs(theta) * 180.0 / Math.PI;
                double predYawDegrees = Math.Abs(predYaw) * 180.0 / Math.PI;
                double yawDiff = Math.Abs(WrapAngleDeg(estYawDegrees - predYawDegrees));
                const double maxYawDiffDeg = 25.0;

                double estStep = Math.Sqrt(tx * tx + tz * tz);
                double predStep = Math.Sqrt(predStepX * predStepX + predStepZ * predStepZ);
                double stepDiff = Math.Abs(estStep - predStep);
                const double maxStepDiffMeters = 6.0;

                if (yawDiff > maxYawDiffDeg || stepDiff > maxStepDiffMeters)
                    return false;
            }

            inliers = bestCount;

            for (int i = 0; i < n; i++)
            {
                if (bestInlier[i])
                    inlierFlags[sourceIndex[i]] = true;
            }

            return true;
        }

        private static void FitRigid(
            List<double> px, List<double> pz, List<double> qx, List<double> qz,
            int a, int b,
            out double theta, out double tx, out double tz)
        {
            double pcx = (px[a] + px[b]) * 0.5;
            double pcz = (pz[a] + pz[b]) * 0.5;
            double qcx = (qx[a] + qx[b]) * 0.5;
            double qcz = (qz[a] + qz[b]) * 0.5;

            double apx = px[a] - pcx, apz = pz[a] - pcz;
            double aqx = qx[a] - qcx, aqz = qz[a] - qcz;
            double bpx = px[b] - pcx, bpz = pz[b] - pcz;
            double bqx = qx[b] - qcx, bqz = qz[b] - qcz;

            double num = apx * aqz - apz * aqx + bpx * bqz - bpz * bqx;
            double den = apx * aqx + apz * aqz + bpx * bqx + bpz * bqz;

            theta = Math.Atan2(num, den);
            double c = Math.Cos(theta);
            double s = Math.Sin(theta);

            tx = qcx - (c * pcx - s * pcz);
            tz = qcz - (s * pcx + c * pcz);
        }

        private static void FitRigidFromMask(
            List<double> px, List<double> pz, List<double> qx, List<double> qz,
            bool[] mask,
            out double theta, out double tx, out double tz)
        {
            double pcx = 0, pcz = 0, qcx = 0, qcz = 0;
            int count = 0;

            for (int i = 0; i < px.Count; i++)
            {
                if (!mask[i]) continue;
                pcx += px[i];
                pcz += pz[i];
                qcx += qx[i];
                qcz += qz[i];
                count++;
            }

            if (count == 0)
            {
                theta = tx = tz = 0.0;
                return;
            }

            pcx /= count;
            pcz /= count;
            qcx /= count;
            qcz /= count;

            double num = 0, den = 0;

            for (int i = 0; i < px.Count; i++)
            {
                if (!mask[i]) continue;

                double apx = px[i] - pcx, apz = pz[i] - pcz;
                double aqx = qx[i] - qcx, aqz = qz[i] - qcz;
                num += apx * aqz - apz * aqx;
                den += apx * aqx + apz * aqz;
            }

            theta = Math.Atan2(num, den);
            double c = Math.Cos(theta);
            double s = Math.Sin(theta);

            tx = qcx - (c * pcx - s * pcz);
            tz = qcz - (s * pcx + c * pcz);
        }

        private static int CountInliers(
            List<double> px, List<double> pz, List<double> qx, List<double> qz,
            double theta, double tx, double tz,
            bool[] mask)
        {
            double c = Math.Cos(theta);
            double s = Math.Sin(theta);
            double toleranceSq = RansacTolerance * RansacTolerance;
            int count = 0;

            for (int i = 0; i < px.Count; i++)
            {
                double predictedX = c * px[i] - s * pz[i] + tx;
                double predictedZ = s * px[i] + c * pz[i] + tz;
                double dx = predictedX - qx[i];
                double dz = predictedZ - qz[i];

                if (dx * dx + dz * dz <= toleranceSq)
                {
                    mask[i] = true;
                    count++;
                }
                else
                {
                    mask[i] = false;
                }
            }

            return count;
        }

        private void ApplyMotion(double yawStep, double tx, double tz)
        {
            LastStep = new PosePriorStep(yawStep, tx, tz);

            if (!hasSmoothedMotion)
            {
                smoothedYawStep = yawStep;
                hasSmoothedMotion = true;
            }
            else
            {
                smoothedYawStep += YawSmoothing * (yawStep - smoothedYawStep);
            }

            double nextHeading = heading + smoothedYawStep;
            double c = Math.Cos(nextHeading);
            double s = Math.Sin(nextHeading);

            // t is expressed in the current camera frame. The camera displacement in
            // world coordinates is -R(nextHeading)^T * t.
            double dx = -(c * tx + s * tz);
            double dz = -(-s * tx + c * tz);

            if (smoothedStepX == 0.0 && smoothedStepZ == 0.0)
            {
                smoothedStepX = dx;
                smoothedStepZ = dz;
            }
            else
            {
                smoothedStepX += TranslationSmoothing * (dx - smoothedStepX);
                smoothedStepZ += TranslationSmoothing * (dz - smoothedStepZ);
            }

            heading = nextHeading;
            centerX += smoothedStepX;
            centerZ += smoothedStepZ;
        }

        private void ApplySmoothedStep()
        {
            LastStep = new PosePriorStep(smoothedYawStep, smoothedStepX, smoothedStepZ);
            heading += smoothedYawStep;
            centerX += smoothedStepX;
            centerZ += smoothedStepZ;
        }

        private void UpdatePoseFromState()
        {
            double c = Math.Cos(heading);
            double s = Math.Sin(heading);

            // The pose maps actual camera coordinates to the world. The camera is
            // mounted with a small pitch relative to the road, so its orientation is
            // R_y(heading) * R_cameraToLevel, where R_cameraToLevel uses the ground
            // normal (normalY, normalZ) established in Initialize. Keeping the pitch in
            // the pose is what makes keyframe triangulation and the ground projection
            // agree; without it structure points pick up a systematic depth error.
            currentPose.Rotation = new double[,]
            {
                { c, -s * normalZ, s * normalY },
                { 0, normalY, normalZ },
                { -s, -c * normalZ, c * normalY }
            };

            currentPose.Translation = new[] { centerX, -groundHeight, centerZ };
        }

        // ---------------------------------------------------------------------
        // Map construction
        // ---------------------------------------------------------------------

        private void AddRoadPoints(List<Point2f> current, bool[] inlierFlags, Mat colorFrame)
        {
            double c = Math.Cos(heading);
            double s = Math.Sin(heading);
            int added = 0;

            for (int i = 0; i < current.Count && added < MaxRoadPointsPerFrame; i++)
            {
                if (i >= inlierFlags.Length || !inlierFlags[i]) continue;
                if (!TryProjectToGround(current[i], out double gx, out double gy, out double gz)) continue;

                // Only trust the plane projection at a moderate range. Very near and
                // very far points are dominated by pixel noise.
                if (gz < minGroundDepth || gz > maxGroundDepth) continue;

                // Convert the actual (pitched) camera-frame ground point into level
                // road coordinates. The plane intersection guarantees Y == groundHeight,
                // so the world height of a road point is exactly zero; using the raw
                // camera y would place the strip below the road at far distances.
                double levelZ = -normalZ * gy + normalY * gz;

                double wx = c * gx + s * levelZ + centerX;
                double wy = 0.0;
                double wz = -s * gx + c * levelZ + centerZ;

                float glx = (float)wx;
                float gly = (float)-wy;
                float glz = (float)-wz;

                // Count repeated observations of the same road structure instead of
                // silently dropping the duplicate. The cleanup bake pass uses the
                // count to tell stable road points from one-off noise.
                if (spatialIndex.TryFindNear(map.Points, glx, gly, glz, out int existing))
                {
                    map.Points[existing].Observations++;
                    continue;
                }

                AddMapPoint(new MapPoint
                {
                    Position = new GLVector3(glx, gly, glz),
                    Color = SampleColor(colorFrame, current[i]),
                    Observations = 1,
                    // Road points lie on the ground plane; their normal points up
                    // in world coordinates. GL y points down, so use (0, -1, 0).
                    Normal = new GLVector3(0f, -1f, 0f),
                    Confidence = 0.5f,
                });

                added++;
            }
        }

        private int AddStructurePoints(Mat colorFrame)
        {
            int totalAdded = 0;
            int callIndex = structureCalls++;
            bool trace = callIndex < TracedCalls;

            var recentKeyframes = keyframes.Recent(maxStructureKeyframes);

            if (trace)
                Log.Information("AddStructurePoints #{Call}: {Keyframes} keyframes.", callIndex, recentKeyframes.Count);

            // Triangulate structure from the most recent keyframes (newest first).
            // Multiple baselines give a denser, more consistent cloud on complex city
            // roads and high-speed tracks where a single baseline is too short or too
            // noisy.
            foreach (var keyframe in recentKeyframes)
            {
                if (keyframe.Gray is null || keyframe.Points2D.Count < KeyframeManager.MinKeyframePoints)
                {
                    if (trace)
                        Log.Information(
                            "AddStructurePoints #{Call}: keyframe {Id} skipped (gray {HasGray}, points {Points}).",
                            callIndex,
                            keyframe.Id,
                            keyframe.Gray is not null,
                            keyframe.Points2D.Count);
                    continue;
                }

                using var structureMask = BuildTrackingMask();
                var (from, to) = TrackAndValidate(keyframe.Gray, gray, keyframe.Points2D, structureMask);

                if (trace)
                    Log.Information(
                        "AddStructurePoints #{Call}: keyframe {Id} tracked {Tracked} points.",
                        callIndex,
                        keyframe.Id,
                        from.Count);

                if (from.Count < 12)
                    continue;

                var (relativeRotation, relativeTranslation) = ComputeRelativePose(keyframe.Pose, currentPose);

                double translationLength = Math.Sqrt(
                    relativeTranslation[0] * relativeTranslation[0] +
                    relativeTranslation[1] * relativeTranslation[1] +
                    relativeTranslation[2] * relativeTranslation[2]);

                // With a (near) zero baseline triangulation is pure noise.
                if (translationLength < 1e-3)
                    continue;

                using var p1 = Mat.FromArray(from.ToArray());
                using var p2 = Mat.FromArray(to.ToArray());
                using var proj1 = CreateProjectionMatrix(
                    new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } }, new double[3]);
                using var proj2 = CreateProjectionMatrix(relativeRotation, relativeTranslation);
                using var points4D = new Mat();

                if (trace)
                    Log.Information("AddStructurePoints #{Call}: triangulating {Count} points.", callIndex, from.Count);

                Cv2.TriangulatePoints(proj1, proj2, p1, p2, points4D);

                if (trace)
                    Log.Information("AddStructurePoints #{Call}: triangulation done.", callIndex);

                var camera2In1 = new double[3];
                for (int r = 0; r < 3; r++)
                    for (int k = 0; k < 3; k++)
                        camera2In1[r] -= relativeRotation[k, r] * relativeTranslation[k];

                int perKeyframeLimit = maxStructurePointsPerFrame / maxStructureKeyframes;
                if (perKeyframeLimit < 1) perKeyframeLimit = 1;

                int added = 0;

                for (int i = 0; i < from.Count && added < perKeyframeLimit; i++)
                {
                    float w = points4D.At<float>(3, i);
                    if (Math.Abs(w) < 1e-5f) continue;

                    double x = points4D.At<float>(0, i) / w;
                    double y = points4D.At<float>(1, i) / w;
                    double z = points4D.At<float>(2, i) / w;

                    if (!double.IsFinite(x) || !double.IsFinite(y) || !double.IsFinite(z)) continue;
                    if (z <= MinStructureDepth || z > maxStructureDepth) continue;

                    double xc = relativeRotation[0, 0] * x + relativeRotation[0, 1] * y + relativeRotation[0, 2] * z + relativeTranslation[0];
                    double yc = relativeRotation[1, 0] * x + relativeRotation[1, 1] * y + relativeRotation[1, 2] * z + relativeTranslation[1];
                    double zc = relativeRotation[2, 0] * x + relativeRotation[2, 1] * y + relativeRotation[2, 2] * z + relativeTranslation[2];
                    if (zc <= MinStructureDepth) continue;

                    double dx = x - camera2In1[0], dy = y - camera2In1[1], dz = z - camera2In1[2];
                    double n1 = Math.Sqrt(x * x + y * y + z * z);
                    double n2 = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                    if (n1 < 1e-6 || n2 < 1e-6) continue;

                    double cosAngle = (x * dx + y * dy + z * dz) / (n1 * n2);
                    double parallaxDegrees = Math.Acos(Math.Clamp(cosAngle, -1, 1)) * 180.0 / Math.PI;
                    if (parallaxDegrees < MinParallaxDeg) continue;

                    double u1 = fx * x / z + cx, v1 = fy * y / z + cy;
                    double e1x = u1 - from[i].X, e1y = v1 - from[i].Y;
                    if (e1x * e1x + e1y * e1y > MaxReprojErrPx * MaxReprojErrPx) continue;

                    double u2 = fx * xc / zc + cx, v2 = fy * yc / zc + cy;
                    double e2x = u2 - to[i].X, e2y = v2 - to[i].Y;
                    if (e2x * e2x + e2y * e2y > MaxReprojErrPx * MaxReprojErrPx) continue;

                    double wx = keyframe.Pose.Rotation[0, 0] * x + keyframe.Pose.Rotation[0, 1] * y + keyframe.Pose.Rotation[0, 2] * z + keyframe.Pose.Translation[0];
                    double wy = keyframe.Pose.Rotation[1, 0] * x + keyframe.Pose.Rotation[1, 1] * y + keyframe.Pose.Rotation[1, 2] * z + keyframe.Pose.Translation[1];
                    double wz = keyframe.Pose.Rotation[2, 0] * x + keyframe.Pose.Rotation[2, 1] * y + keyframe.Pose.Rotation[2, 2] * z + keyframe.Pose.Translation[2];

                    // World y points down, so structure above the road has y < 0.
                    if (wy > -MinStructureHeight) continue;
                    // Reject points far from the camera: at long range the triangulated
                    // depth is too uncertain and the cloud turns into a lateral spray.
                    if (Math.Abs(wx - centerX) > 30.0 || Math.Abs(wz - centerZ) > 80.0) continue;

                    float glx = (float)wx;
                    float gly = (float)-wy;
                    float glz = (float)-wz;

                    // The same structure seen again counts as another observation instead
                    // of a duplicate point. The bake passes use this to confirm points
                    // across keyframes and passes.
                    if (spatialIndex.TryFindNear(map.Points, glx, gly, glz, out int existing))
                    {
                        map.Points[existing].Observations++;
                        continue;
                    }

                    AddMapPoint(new MapPoint
                    {
                        Position = new GLVector3(glx, gly, glz),
                        Color = SampleColor(colorFrame, to[i]),
                        Observations = 1,
                        // Approximate surface normal from the viewing direction:
                        // points seen from the camera tend to face the camera.
                        Normal = new GLVector3(
                            (float)(-x / n1),
                            (float)(-y / n1),
                            (float)(-z / n1)),
                        Confidence = 0.5f,
                    });

                    added++;
                }

                totalAdded += added;
            }

            return totalAdded;
        }

        private static GLVector3 SampleColor(Mat colorFrame, Point2f point)
        {
            int px = Math.Clamp((int)Math.Round(point.X), 0, colorFrame.Cols - 1);
            int py = Math.Clamp((int)Math.Round(point.Y), 0, colorFrame.Rows - 1);

            if (colorFrame.Channels() >= 3)
            {
                var pixel = colorFrame.At<Vec3b>(py, px);
                return new GLVector3(pixel.Item2 / 255.0f, pixel.Item1 / 255.0f, pixel.Item0 / 255.0f);
            }

            float intensity = colorFrame.At<byte>(py, px) / 255.0f;
            return new GLVector3(intensity, intensity, intensity);
        }

        private void AddMapPoint(MapPoint point)
        {
            int index = map.Points.Count;
            map.Points.Add(point);
            spatialIndex.IndexPoint(map.Points, index);
        }

        private void TrimMapIfNeeded()
        {
            if (map.Points.Count <= MaxMapPoints)
                return;

            int toRemove = map.Points.Count - (int)(MaxMapPoints * 2.0 / 3.0);
            map.Points.RemoveRange(0, toRemove);
            spatialIndex.Rebuild(map.Points);
        }

        private void MaybeCreateKeyframe(Mat mask)
        {
            var current = keyframes.Current;
            bool needsKeyframe =
                current is null ||
                processedFrames - lastKeyframeFrame >= maxFramesPerKeyframe ||
                RotationAngleDeg(current.Pose, currentPose) >= 15.0 ||
                TranslationDistance(current.Pose, currentPose) >= 1.5;

            if (needsKeyframe && keyframes.CreateNewKeyframe(gray, mask, currentPose))
                lastKeyframeFrame = processedFrames;
        }

        // ---------------------------------------------------------------------
        // Helpers
        // ---------------------------------------------------------------------

        private Mat BuildTrackingMask()
        {
            var mask = new Mat(gray.Rows, gray.Cols, MatType.CV_8UC1, Scalar.All(0));
            int top = (int)(gray.Rows * SkyMargin);
            if (top < gray.Rows - 1)
                Cv2.Rectangle(mask, new Rect(0, top, gray.Cols, gray.Rows - top), Scalar.All(255), -1);
            return mask;
        }

        /// <summary>
        /// Near-field road band used for ground-plane odometry. Restricting detection
        /// to this band keeps every tracked feature inside the metric depth gate
        /// (minGroundDepth..maxGroundDepth), which is what makes the planar fit
        /// numerically stable on dashcam footage.
        /// </summary>
        private Mat BuildGroundMask()
        {
            var mask = new Mat(gray.Rows, gray.Cols, MatType.CV_8UC1, Scalar.All(0));
            int top = (int)(gray.Rows * groundBandTop);
            int bottom = (int)(gray.Rows * groundBandBottom);
            bottom = Math.Min(bottom, gray.Rows);
            if (bottom - top > 1)
                Cv2.Rectangle(mask, new Rect(0, top, gray.Cols, bottom - top), Scalar.All(255), -1);
            return mask;
        }

        /// <summary>
        /// Returns the relative transform from <paramref name="from"/> to
        /// <paramref name="to"/> in camera coordinates: X_to = R * X_from + t.
        /// </summary>
        private static (double[,] R, double[] t) ComputeRelativePose(CameraPose from, CameraPose to)
        {
            var rotation = new double[3, 3];
            var translation = new double[3];

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    double sum = 0;
                    for (int k = 0; k < 3; k++)
                        sum += to.Rotation[k, i] * from.Rotation[k, j];

                    rotation[i, j] = sum;
                }

                double offset = 0;
                for (int k = 0; k < 3; k++)
                    offset += to.Rotation[k, i] * (from.Translation[k] - to.Translation[k]);

                translation[i] = offset;
            }

            return (rotation, translation);
        }

        private static double RotationAngleDeg(CameraPose from, CameraPose to)
        {
            double trace = 0;
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    double value = 0;
                    for (int k = 0; k < 3; k++)
                        value += from.Rotation[k, r] * to.Rotation[k, c];

                    if (r == c) trace += value;
                }
            }

            double cosine = Math.Clamp((trace - 1.0) * 0.5, -1.0, 1.0);
            return Math.Acos(cosine) * 180.0 / Math.PI;
        }

        private static double TranslationDistance(CameraPose from, CameraPose to)
        {
            double dx = to.Translation[0] - from.Translation[0];
            double dy = to.Translation[1] - from.Translation[1];
            double dz = to.Translation[2] - from.Translation[2];
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        private Mat CreateProjectionMatrix(double[,] rotation, double[] translation)
        {
            using var intrinsic = new Mat(3, 3, MatType.CV_32FC1, Scalar.All(0));
            intrinsic.Set(0, 0, (float)fx); intrinsic.Set(0, 2, (float)cx);
            intrinsic.Set(1, 1, (float)fy); intrinsic.Set(1, 2, (float)cy);
            intrinsic.Set(2, 2, 1f);

            using var extrinsic = new Mat(3, 4, MatType.CV_32FC1, Scalar.All(0));
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++) extrinsic.Set(r, c, (float)rotation[r, c]);
                extrinsic.Set(r, 3, (float)translation[r]);
            }

            var projection = new Mat();
            using var empty = new Mat();
            Cv2.Gemm(intrinsic, extrinsic, 1.0, empty, 0.0, projection);
            return projection;
        }

        /// <summary>
        /// Wraps an angle to the [-180, 180] interval.
        /// </summary>
        private static double WrapAngleDeg(double degrees)
        {
            while (degrees > 180.0) degrees -= 360.0;
            while (degrees < -180.0) degrees += 360.0;
            return degrees;
        }

        private int NextRandom(int bound)
        {
            rngState ^= rngState >> 12;
            rngState ^= rngState << 25;
            rngState ^= rngState >> 27;
            ulong value = rngState * 0x2545F4914F6CDD1DUL;
            return (int)(value % (ulong)bound);
        }

        public void Dispose()
        {
            gray.Dispose();
            previousGray?.Dispose();
            cameraMatrix?.Dispose();
            distortion?.Dispose();
            undistorted?.Dispose();
        }
    }
}
