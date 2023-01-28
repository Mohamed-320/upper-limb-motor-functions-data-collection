#pragma once


#define TimeToWaitBeforeMotionStopDetectionStarts			75		/* Unit is X/30 sec where 30 = 1 sec, as the kinect operates with 30 Hz frequency	*/
#define PercentageToDetectMovementStop						0.02	/* Unit is percentage from 0 to 1 (which means 100%), 0.2 is 20% for example		*/
#define CountTimeToDetectMotionStop							15		/* Unit is X/30 sec where 30 = 1 sec, as the kinect operates with 30 Hz frequency	*/