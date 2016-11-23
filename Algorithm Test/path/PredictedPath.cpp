//
//  PredictedPath.cpp
//  PredictedPath
//
//  Created by ning on 4/19/16.
//
//

#include "PredictedPath.h"

namespace PredictedPath
{
    std::vector<Sample> SolvePath(
                                  const std::vector<Sample>& inputSamples,
                                  float segmentSize,
                                  float smoothness)
    {
        Kcurve kcurvePath(inputSamples, segmentSize, smoothness * 200); // scale smoothness
        
        ConfigurablePath *path;
        path = &kcurvePath;
        
        return path->getRawSamples();
    }
}
