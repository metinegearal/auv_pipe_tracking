"""
TrianFlow, PackNet-SfM, or Monodepth2
"""

# print(SensorDefinition._sensor_keys_.keys())
# dict_keys(['RGBCamera', 'DistanceTask', 'LocationTask', 'FollowTask', 'AvoidTask', 'CupGameTask', 'CleanUpTask', 'ViewportCapture', 'OrientationSensor', 'IMUSensor', 'JointRotationSensor', 'RelativeSkeletalPositionSensor', 'LocationSensor', 'RotationSensor', 'VelocitySensor', 'DynamicsSensor', 'PressureSensor', 'CollisionSensor', 'RangeFinderSensor', 'WorldNumSensor', 'BallLocationSensor', 'AbuseSensor', 'DVLSensor', 'PoseSensor', 'AcousticBeaconSensor', 'DepthSensor', 'OpticalModemSensor', 'ImagingSonar', 'SidescanSonar', 'ProfilingSonar', 'GPSSensor', 'MagnetometerSensor', 'SinglebeamSonar'])
#os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

ticks_per_sec=30
azi = 360
minR = 0.75
maxR = 2#50
binsR = int(1250)
binsA = int(400)
cfg = {
    "name": "test_rgb_camera",
    "world": "TestWorld",#Dam,OpenWater,PierHarbor,Rooms // TestWorld,SAUVC
    "package_name": "BauRov",
    "main_agent": "auv0",
    "ticks_per_sec": ticks_per_sec,
#     "weather": {
#     #   "hour": 12,
#       "type": 'sunny',
#       "fog_density": 0.1,
#       "day_cycle_length": 86400
#    },
    "agents": [
        {
            "agent_name": "auv0",
            "agent_type": "HoveringAUV",
            "sensors": [
                {
                    "sensor_type": "DVLSensor",
                    "configuration": {
                        "Elevation": 22.5,                # Beam angle from vertical (z-axis)
                        "DebugLines": False,             # Optional: true if visualizing beam lines

                        "VelSigma": 0.001,               # 0.1 mm/s resolution → std dev ≈ 0.001 m/s
                        # VelCov: [0.000001, 0.000001, 0.000001, 0.000001]  # Optional alternative to VelSigma

                        "ReturnRange": True,
                        "MaxRange": 50.0,                # Max operating range (in meters)

                        "RangeSigma": 0.05,  
                    }
                },
                {
                    "sensor_type": "DepthSensor",
                    "configuration": {
                        "Sigma": 0.05,
                    }
                },
                {
                    "sensor_type": "MagnetometerSensor",
                    "configuration": {
                        "Sigma": 0.0003,
                    }
                },
                {
                    "sensor_type": "IMUSensor",
                    "configuration": {
                        "AngVelSigma": 0.0035,  # ~0.2°/s noise ≈ realistic sim setting
                        "AccelSigma": 0.03,
                        "AngVelBiasSigma": 0.001,# ≈ 0.06°/s drift rate
                        "AccelBiasSigma":0.01
                    }
                },
                { 
                    "sensor_type":'LocationSensor',

                },
                # {
                #     "sensor_type": "ImagingSonar",
                #     "configuration": {
                #         "Azimuth": azi,
                #         "Elevation": 25, 
                #         "RangeMin": minR,
                #         "RangeMax": maxR,
                #         # "RangeRes": 0.04,
                #         "RangeBins": binsR,   
                #         "AzimuthBins": binsA,   
                #     }
                # },
                {
                "sensor_type": "CameraSensor",
                    "sensor_name": "DownCamera",
                    "socket": "CameraLeftSocket",
                    # "rotation": [0, 0, 0],
                    # "location": [0, 0, 0],
                    "rotation": [0, 90, 0],
                    "location": [0, 0, -1],
                    "configuration": {
                        "CaptureWidth": 288,#1920
                        "CaptureHeight": 288,#1080
                        "FovAngle": 90.0,               # Standard wide-angle lens, adjust if your actual lens differs
                        "TargetGamma": 1.0,             # Leave default unless you're doing tone/gamma experiments
                        "ExposureMethod": "AEM_Histogram",  # Best for adaptive low-light exposure
                        "ExposureCompensation": 1,#4.0    # Already tested; increases brightness in low light

                        "ShutterSpeed": 60.0,           # Simulated shutter speed in seconds; 60 is default (~1/60s)
                        "ISO": 800.0,#800                   # Boost ISO for better low-light sim (100 is default, try 800–1600)
                        "Aperature": 2.0 ,              # Wider aperture for more light (f/2.0 = 1/2), f/1.2 is even brighter

                        "FocalDistance": 100.0,         # Objects at 1m are in sharpest focus; change depending on scene
                        "DepthBlurAmount": 1.2,         # Keep default, unless you want extreme blur tuning
                        "DepthBlurRadius": 1.0,         # Moderate depth blur, good for realism in underwater scenes
                        "BladeCount": 6,                # 6-blade aperture = realistic bokeh for cameras
                        "DepthOfFieldMinFstop": 1.4,    # Wider lens simulation
                        "WhiteTemp": 4800.0,            # Slightly cool temp to reflect underwater greens/blue
                        "WhiteTint": 0.25,               # Add a bit of green tint common in low-visibility water
                        "BloomIntensity": 1.8,          # Enhances light diffusion – good for underwater look
                        "LensFlareIntensity": 1.0,      # Moderate flare for realism if light sources are present
                        "ChromAberrIntensity": 1.0,     # Adds slight lens imperfection, realistic at corners
                        "ChromAberrOffset": 0.8,        # Apply mostly near edges

                        "ExposureSpeedDown": 3.0,       # Controls how quickly exposure adapts to *darker* scenes
                        "ExposureSpeedUp": 2.0,         # Controls how quickly exposure adapts to *brighter* scenes

                        "MotionBlurIntensity": 0.2,     # Some blur for realism in motion
                        "MotionBlurMaxDistortion": 0.05,
                        "MotionBlurMinObjectScreenSize": 2.0,
                    }},
                {
                    "sensor_type": "CameraSensor",
                    "socket": "CameraRightSocket",
                    "sensor_name": "FrontCamera",
                    "rotation": [0, 0, 0],
                    "location": [0, 0, 0],
                    "configuration": {
                        "CaptureWidth": 288,#1920
                        "CaptureHeight": 288,#1080
                        "FovAngle": 90.0,               # Standard wide-angle lens, adjust if your actual lens differs
                        "TargetGamma": 1.0,             # Leave default unless you're doing tone/gamma experiments
                        "ExposureMethod": "AEM_Histogram",  # Best for adaptive low-light exposure
                        "ExposureCompensation": 1,#4.0    # Already tested; increases brightness in low light

                        "ShutterSpeed": 60.0,           # Simulated shutter speed in seconds; 60 is default (~1/60s)
                        "ISO": 800.0,#800                   # Boost ISO for better low-light sim (100 is default, try 800–1600)
                        "Aperature": 2.0 ,              # Wider aperture for more light (f/2.0 = 1/2), f/1.2 is even brighter

                        "FocalDistance": 100.0,         # Objects at 1m are in sharpest focus; change depending on scene
                        "DepthBlurAmount": 1.2,         # Keep default, unless you want extreme blur tuning
                        "DepthBlurRadius": 1.0,         # Moderate depth blur, good for realism in underwater scenes
                        "BladeCount": 6,                # 6-blade aperture = realistic bokeh for cameras
                        "DepthOfFieldMinFstop": 1.4,    # Wider lens simulation
                        "WhiteTemp": 4800.0,            # Slightly cool temp to reflect underwater greens/blue
                        "WhiteTint": 0.25,               # Add a bit of green tint common in low-visibility water
                        "BloomIntensity": 1.8,          # Enhances light diffusion – good for underwater look
                        "LensFlareIntensity": 1.0,      # Moderate flare for realism if light sources are present
                        "ChromAberrIntensity": 1.0,     # Adds slight lens imperfection, realistic at corners
                        "ChromAberrOffset": 0.8,        # Apply mostly near edges

                        "ExposureSpeedDown": 3.0,       # Controls how quickly exposure adapts to *darker* scenes
                        "ExposureSpeedUp": 2.0,         # Controls how quickly exposure adapts to *brighter* scenes

                        "MotionBlurIntensity": 0.2,     # Some blur for realism in motion
                        "MotionBlurMaxDistortion": 0.05,
                        "MotionBlurMinObjectScreenSize": 2.0,
                        # "MaxViewDistanceOverride":100
                    }
                },
                {
                    "sensor_type": "RangeFinderSensor",
                    "socket": "COM",
                    "sensor_name": "RangeFinder",
                }
            ],
            "control_scheme": 0,
            # "location": [-150, 0, -15],
            # "location": [0,0,-20],
            # "location": [136.9,-108.8,-31],#SAUVC Giriş
            # "location": [120,-107,-32],#SAUVC Gate
            # "location": [12,-130,-21],#Valve
            # "location": [0,0,50],

            # "location": [15,-92,-28],#docking station // 40 for more realistic 20 for testing
            "location": [43.203,-64.463,-24.178], #pipe start point

            # "location": [50,-59,-24.178], #pipe mission start
            # "location":[20,-122,-28],#station rotation=90
            # "location":[20, -130,-10],#route around buoy
            "rotation": [0, 0, 180]
        }
    ]
    }