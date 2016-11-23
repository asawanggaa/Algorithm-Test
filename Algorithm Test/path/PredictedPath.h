//
//  PredictedPath.h
//  This function is used to generated a predicted path based on the input sample points. Based on the shape of the samples,
//  the result shape will be straight line, ellipse or smoothed kcurve.
//  Created by ning on 4/19/16.
//
//

#ifndef PredictedPath_h
#define PredictedPath_h

#include "ConfigurablePath.h"

namespace PredictedPath {
    
    /// this function return the re-configured sample list
    std::vector<Sample> SolvePath(
                        const std::vector<Sample>& inputSamples,    // samples to be configured.
                        float segmentSize = 10.0f,                  // resample distance in pixels, make the samples even distributed by segmentSize pixels.
                        float smoothness = 0.04f                   // value [0,1], how much strength to smooth the curve (only work for kcurve).
    );
}


#endif /* PredictedPath_h */
