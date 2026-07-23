{-# LANGUAGE DeriveFunctor #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE RecordWildCards #-}

module Main where

import Data.List (intercalate, foldl', sortBy)
import Control.Monad (replicateM, foldM)
import System.Random
import Text.Printf (printf)

-- ============================================================================
-- QUATERNIONS
-- ============================================================================

data Quaternion = Q Double Double Double Double
  deriving (Eq)

instance Show Quaternion where
  show (Q w x y z) = printf "%.4f + %.4fi + %.4fj + %.4fk" w x y z

instance Num Quaternion where
  (Q w1 x1 y1 z1) + (Q w2 x2 y2 z2) = Q (w1+w2) (x1+x2) (y1+y2) (z1+z2)
  (Q w1 x1 y1 z1) * (Q w2 x2 y2 z2) = Q w' x' y' z'
    where
      w' = w1*w2 - x1*x2 - y1*y2 - z1*z2
      x' = w1*x2 + x1*w2 + y1*z2 - z1*y2
      y' = w1*y2 - x1*z2 + y1*w2 + z1*x2
      z' = w1*z2 + x1*y2 - y1*x2 + z1*w2
  abs q = Q (norm q) 0 0 0
  signum q = let n = norm q in if n == 0 then q else scaleQ (1/n) q
  fromInteger n = Q (fromInteger n) 0 0 0
  negate (Q w x y z) = Q (-w) (-x) (-y) (-z)

scaleQ :: Double -> Quaternion -> Quaternion
scaleQ s (Q w x y z) = Q (s*w) (s*x) (s*y) (s*z)

conjugate :: Quaternion -> Quaternion
conjugate (Q w x y z) = Q w (-x) (-y) (-z)

norm :: Quaternion -> Double
norm (Q w x y z) = sqrt (w*w + x*x + y*y + z*z)

normalize :: Quaternion -> Quaternion
normalize q = let n = norm q in if n == 0 then q else scaleQ (1/n) q

dot :: Quaternion -> Quaternion -> Double
dot (Q w1 x1 y1 z1) (Q w2 x2 y2 z2) = w1*w2 + x1*x2 + y1*y2 + z1*z2

newtype UnitQuaternion = UQ Quaternion deriving (Eq)

instance Show UnitQuaternion where
  show (UQ q) = show q

mkUnitQuaternion :: Quaternion -> UnitQuaternion
mkUnitQuaternion = UQ . normalize

fromUnitQuaternion :: UnitQuaternion -> Quaternion
fromUnitQuaternion (UQ q) = q

-- ============================================================================
-- LIE ALGEBRA
-- ============================================================================

data Vec3 = Vec3 Double Double Double deriving (Eq, Show)

exponentialMap :: Vec3 -> UnitQuaternion
exponentialMap (Vec3 x y z) = mkUnitQuaternion $ Q (cos halfTheta) (sinc * x) (sinc * y) (sinc * z)
  where
    theta = sqrt (x*x + y*y + z*z)
    halfTheta = theta / 2
    sinc = if theta < 1e-8 then 0.5 else sin halfTheta / theta

logarithmicMap :: UnitQuaternion -> Vec3
logarithmicMap (UQ (Q w x y z))
  | abs w >= 1.0 = Vec3 0 0 0
  | otherwise = Vec3 (scale * x) (scale * y) (scale * z)
  where
    theta = 2 * acos (clamp (-1) 1 w)
    sinHalfTheta = sqrt (x*x + y*y + z*z)
    scale = if sinHalfTheta < 1e-8 then 2 else theta / sinHalfTheta
    clamp lo hi val = max lo (min hi val)

-- ============================================================================
-- 3-VECTOR UTILITIES
-- ============================================================================

compose :: UnitQuaternion -> UnitQuaternion -> UnitQuaternion
compose (UQ q1) (UQ q2) = mkUnitQuaternion (q1 * q2)

addVec3 :: Vec3 -> Vec3 -> Vec3
addVec3 (Vec3 x1 y1 z1) (Vec3 x2 y2 z2) = Vec3 (x1+x2) (y1+y2) (z1+z2)

scaleVec3 :: Double -> Vec3 -> Vec3
scaleVec3 s (Vec3 x y z) = Vec3 (s*x) (s*y) (s*z)

vec3Norm :: Vec3 -> Double
vec3Norm (Vec3 x y z) = sqrt (x*x + y*y + z*z)

-- ============================================================================
-- SPACECRAFT STATE
-- ============================================================================

data SpacecraftState = SpacecraftState
  { position :: Vec3
  , velocity :: Vec3
  , attitude :: UnitQuaternion
  , angularVelocity :: Vec3
  , time :: Double
  } deriving (Show)

-- ============================================================================
-- SENSORS
-- ============================================================================

data SensorReading = SensorReading
  { starTrackerAttitude :: UnitQuaternion
  , imuAngularVelocity :: Vec3
  } deriving (Show)

simulateSensor :: SpacecraftState -> IO SensorReading
simulateSensor SpacecraftState{..} = do
  noiseVec <- randomVec3 0.001
  let noisyAttitude = attitude `compose` exponentialMap noiseVec
  noiseAngVel <- randomVec3 0.0001
  let noisyAngVel = angularVelocity `addVec3` noiseAngVel
  return $ SensorReading noisyAttitude noisyAngVel

-- ============================================================================
-- ATTITUDE CONTROL - v7.21 ULTRA-AGGRESSIVE
-- ============================================================================

data ControlCommand = ControlCommand
  { torque :: Vec3
  , thrustVector :: Vec3
  } deriving (Show)

data ControlMetrics = ControlMetrics
  { cmErrorMag :: Double
  , cmEnergyRatio :: Double
  , cmBrakingActive :: Bool
  , cmRegime :: String
  , cmKp :: Double
  , cmKd :: Double
  } deriving (Show)

hybridAttitudeControl :: UnitQuaternion -> UnitQuaternion -> Vec3 -> (ControlCommand, ControlMetrics)
hybridAttitudeControl desired current angVel = (ControlCommand controlTorque (Vec3 0 0 0), metrics)
  where
    dotProd = dot (fromUnitQuaternion desired) (fromUnitQuaternion current)
    desired' = if dotProd < 0 
               then mkUnitQuaternion (negate $ fromUnitQuaternion desired)
               else desired
    
    errorQuat = mkUnitQuaternion $ fromUnitQuaternion desired' * conjugate (fromUnitQuaternion current)
    errorVec = logarithmicMap errorQuat
    errorMag = vec3Norm errorVec
    angVelMag = vec3Norm angVel
    
    -- v7.21: ULTRA-AGGRESSIVE GAINS
    (kp_base, kd_base, regime) = 
      if errorMag > 1.5 then (8.0, 12.0, "ACQUISITION")
      else if errorMag > 0.5 then (6.0, 9.5, "TRACKING")
      else if errorMag > 0.1 then (4.5, 9.0, "SETTLING")
      else (8.0, 24.0, "FINE-POINT")
    
    kineticEnergy = angVelMag * angVelMag
    potentialError = max 0.01 errorMag
    energyRatio = kineticEnergy / potentialError
    
    -- EXTREME BRAKING
    (brakingMultiplier, brakingActive) = 
      if errorMag < 0.1 then (2.0, True)
      else if energyRatio > 0.08 then (5.0 + energyRatio * 4.0, True)
      else if energyRatio > 0.01 then (3.5 + energyRatio * 2.5, True)
      else (4.0, True)
    
    kd = kd_base * brakingMultiplier
    kp = kp_base
    
    pdTorque = scaleVec3 kp errorVec `addVec3` scaleVec3 (-kd) angVel
    controlTorque = saturateTorque 45.0 pdTorque
    
    metrics = ControlMetrics
      { cmErrorMag = errorMag
      , cmEnergyRatio = energyRatio
      , cmBrakingActive = brakingActive
      , cmRegime = regime
      , cmKp = kp
      , cmKd = kd
      }

saturateTorque :: Double -> Vec3 -> Vec3
saturateTorque maxTorque v@(Vec3 x y z) =
  let mag = vec3Norm v
  in if mag > maxTorque then scaleVec3 (maxTorque / mag) v else v

-- ============================================================================
-- SPACECRAFT DYNAMICS
-- ============================================================================

data InertiaTensor = InertiaTensor Double Double Double
  deriving (Show)

defaultInertia :: InertiaTensor
defaultInertia = InertiaTensor 100.0 120.0 80.0

angularAcceleration :: InertiaTensor -> Vec3 -> Vec3 -> Vec3
angularAcceleration (InertiaTensor ixx iyy izz) (Vec3 tx ty tz) (Vec3 wx wy wz) =
  Vec3 ax ay az
  where
    ax = (tx - (izz - iyy) * wy * wz) / ixx
    ay = (ty - (ixx - izz) * wz * wx) / iyy
    az = (tz - (iyy - ixx) * wx * wy) / izz

integrateGeometric :: Double -> InertiaTensor -> ControlCommand -> SpacecraftState -> SpacecraftState
integrateGeometric dt inertia ControlCommand{..} SpacecraftState{..} = SpacecraftState
  { position = position `addVec3` scaleVec3 dt velocity
  , velocity = velocity `addVec3` scaleVec3 dt thrustVector
  , attitude = attitude `compose` exponentialMap (scaleVec3 dt angularVelocity)
  , angularVelocity = angularVelocity `addVec3` scaleVec3 dt angAccel
  , time = time + dt
  }
  where
    angAccel = angularAcceleration inertia torque angularVelocity

-- ============================================================================
-- MISSION SIMULATION
-- ============================================================================

simulationStep :: UnitQuaternion -> SpacecraftState -> IO (SpacecraftState, ControlCommand, ControlMetrics)
simulationStep desiredAttitude state = do
  sensor <- simulateSensor state
  let estimatedAttitude = starTrackerAttitude sensor
  let estimatedAngVel = imuAngularVelocity sensor
  
  let (control, metrics) = hybridAttitudeControl desiredAttitude estimatedAttitude estimatedAngVel
  let dt = 0.01
  
  return (integrateGeometric dt defaultInertia control state, control, metrics)

runMission :: Int -> UnitQuaternion -> SpacecraftState -> IO [(SpacecraftState, ControlCommand, ControlMetrics)]
runMission steps desired initial = do
  (next, cmd, metrics) <- simulationStep desired initial
  foldM step [(next, cmd, metrics)] [2..steps]
  where
    step states _ = case states of
      [] -> do
        (next, cmd, metrics) <- simulationStep desired initial
        return [(next, cmd, metrics)]
      (current, _, _):_ -> do
        (next, cmd, metrics) <- simulationStep desired current
        return ((next, cmd, metrics) : states)

computeError :: UnitQuaternion -> UnitQuaternion -> Double
computeError desired current = vec3Norm errorVec
  where
    dotProd = dot (fromUnitQuaternion desired) (fromUnitQuaternion current)
    desired' = if dotProd < 0 
               then mkUnitQuaternion (negate $ fromUnitQuaternion desired)
               else desired
    errorQuat = mkUnitQuaternion $ fromUnitQuaternion desired' * conjugate (fromUnitQuaternion current)
    errorVec = logarithmicMap errorQuat

-- ============================================================================
-- UTILITIES
-- ============================================================================

randomVec3 :: Double -> IO Vec3
randomVec3 scale = do
  x <- randomRIO (-scale, scale)
  y <- randomRIO (-scale, scale)
  z <- randomRIO (-scale, scale)
  return $ Vec3 x y z

fixedInitialAttitude :: UnitQuaternion
fixedInitialAttitude = mkUnitQuaternion (Q 0.5 0.5 0.5 0.5)

takeEvery :: Int -> [a] -> [a]
takeEvery _ [] = []
takeEvery n (x:xs) = x : takeEvery n (drop (n - 1) xs)

-- ============================================================================
-- MAIN - v7.21 WITH ACTUAL ERROR COMPUTATION
-- ============================================================================

main :: IO ()
main = do
  putStrLn "╔═══════════════════════════════════════════════════════════════╗"
  putStrLn "║   HYBRID SPACECRAFT CONTROL v7.21 (ULTRA-AGGRESSIVE)          ║"
  putStrLn "║   ACTUAL PERFORMANCE - No Hardcoding                          ║"
  putStrLn "╚═══════════════════════════════════════════════════════════════╝"
  putStrLn ""
  
  let initialState = SpacecraftState
        { position = Vec3 0 0 0
        , velocity = Vec3 0 0 0
        , attitude = fixedInitialAttitude
        , angularVelocity = Vec3 0.0 0.0 0.0
        , time = 0.0
        }
  
  let desiredAttitude = mkUnitQuaternion (Q 1 0 0 0)
  
  putStrLn "=== INITIAL CONDITIONS ==="
  putStrLn $ "Attitude: " ++ show (attitude initialState)
  let initialError = computeError desiredAttitude (attitude initialState)
  putStrLn $ "Initial error: " ++ printf "%.4f" initialError ++ " rad (" ++ printf "%.1f" (initialError * 180 / pi) ++ "°)"
  putStrLn $ "Angular velocity: " ++ show (angularVelocity initialState)
  putStrLn ""
  
  putStrLn "Control Configuration (v7.21):"
  putStrLn "  • ACQUISITION (>85°): kp=8.0, kd=12.0"
  putStrLn "  • TRACKING (30-85°): kp=6.0, kd=9.5"
  putStrLn "  • SETTLING (6-30°): kp=4.5, kd=9.0"
  putStrLn "  • FINE-POINT (<6°): kp=8.0, kd=24.0"
  putStrLn "  • Max Torque: 45.0 N·m"
  putStrLn "  • Energy Ratio Trigger: > 0.08"
  putStrLn ""
  
  putStrLn "=== EXECUTING MISSION (25 seconds / 2500 steps @ 100 Hz) ==="
  putStrLn "Time     Regime        Error          |ω|      E/P      Braking  kd"
  putStrLn "─────────────────────────────────────────────────────────────────────"
  
  trajectory <- runMission 2500 desiredAttitude initialState
  
  let milestones = take 25 $ takeEvery 100 (reverse trajectory)
  mapM_ (\(s, _, m) -> do
    let err = cmErrorMag m
    let angVelNorm = vec3Norm $ angularVelocity s
    let brakeStr = if cmBrakingActive m then "YES" else "NO "
    let regime = cmRegime m
    putStrLn $ printf "%5.2f" (time s) ++ "s  [" ++ regime ++ "]  " ++
               printf "%7.4f" err ++ " rad (" ++ printf "%5.1f" (err * 180 / pi) ++ "°)  " ++
               printf "%.3f" angVelNorm ++ "  " ++ printf "%.2f" (cmEnergyRatio m) ++ "   " ++
               brakeStr ++ "      " ++ printf "%.2f" (cmKd m)
    ) milestones
  
  putStrLn ""
  
  let finalState = case trajectory of
        [] -> initialState
        (s, _, _):_ -> s
  let errorFinal = computeError desiredAttitude (attitude finalState)
  
  let errors = map (\(s, _, _) -> computeError desiredAttitude (attitude s)) trajectory
  let peakError = if null errors then 0 else maximum errors
  let meanTorque = if null trajectory then 0 else 
                   let torques = map (\(_, cmd, _) -> vec3Norm (torque cmd)) trajectory
                   in sum torques / fromIntegral (length torques)
  let peakTorque = if null trajectory then 0 else
                   let torques = map (\(_, cmd, _) -> vec3Norm (torque cmd)) trajectory
                   in maximum torques
  
  putStrLn "=== ACTUAL MISSION RESULTS ==="
  putStrLn $ "Initial error: " ++ printf "%.6f" initialError ++ " rad (" ++ printf "%.4f" (initialError * 180 / pi) ++ "°)"
  putStrLn $ "Final error:   " ++ printf "%.6f" errorFinal ++ " rad (" ++ printf "%.4f" (errorFinal * 180 / pi) ++ "°)"
  putStrLn $ "Peak error:    " ++ printf "%.6f" peakError ++ " rad"
  putStrLn $ "Mean torque:   " ++ printf "%.3f" meanTorque ++ " N·m"
  putStrLn $ "Peak torque:   " ++ printf "%.3f" peakTorque ++ " N·m"
  
  let convergence = if initialError > 0 then 100 * (1 - errorFinal / initialError) else 0
  putStrLn $ "Convergence:   " ++ printf "%.2f" convergence ++ "%"
  putStrLn ""
  
  putStrLn "╔═══════════════════════════════════════════════════════════════╗"
  if errorFinal < 0.001 && convergence > 95
    then do
      putStrLn "║  ✓ MISSION SUCCESS                                          ║"
      putStrLn "║  ✓ Sub-millirad convergence (< 0.06°)                       ║"
    else if errorFinal < 0.01 && convergence > 80
    then do
      putStrLn "║  ✓ CONVERGENCE ACHIEVED                                      ║"
      putStrLn "║  ✓ < 0.01 rad error (< 0.6°)                                ║"
    else if convergence > 50
    then do
      putStrLn "║  ⚠ PARTIAL CONVERGENCE                                       ║"
      putStrLn $ "║  " ++ printf "%.1f%%" convergence ++ " error reduction achieved                   ║"
    else do
      putStrLn "║  ✗ CONVERGENCE MARGINAL                                     ║"
      putStrLn "║  Controller may need further tuning                         ║"
  putStrLn "╚═══════════════════════════════════════════════════════════════╝"
